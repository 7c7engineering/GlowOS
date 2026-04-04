#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <sys/param.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include "glow_context.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "freertos/semphr.h"

#include "esp_check.h"
#include "esp_event.h"
#include "esp_http_server.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include "nvs_flash.h"

#include "glow_storage.h"
#include "glow_web.h"

static const char *TAG = "GLOW_WEB";

extern glow_context_t *g_context;

#define GLOW_WEB_AP_SSID "GlowOS"
#define GLOW_WEB_AP_PASS "glowdriver"
#define GLOW_WEB_AP_CHANNEL 1
#define GLOW_WEB_AP_MAX_CONN 4
// ESP-IDF uses quarter-dBm units: 8 = 2 dBm, 20 = 5 dBm, 84 = 21 dBm.
#define GLOW_WEB_AP_TX_POWER_QDBM 20

#define DNS_PORT 53
#define DNS_PACKET_SIZE 512
#define WEB_CONFIG_JSON_MAX_LEN 1024

static const uint8_t s_ap_ip[4] = {192, 168, 4, 1};
static httpd_handle_t s_http_server;
static bool s_initialized;

extern const uint8_t _binary_index_html_start[] asm("_binary_index_html_start");
extern const uint8_t _binary_index_html_end[] asm("_binary_index_html_end");

static void glow_state_to_string(glow_system_state_t state, char *out_str, size_t out_size)
{
	switch (state)
	{
	case GLOW_STATE_OFF:
		strncpy(out_str, "OFF", out_size);
		break;
	case GLOW_STATE_BOOST:
		strncpy(out_str, "BOOST", out_size);
		break;
	case GLOW_STATE_GLOWING:
		strncpy(out_str, "GLOWING", out_size);
		break;
	case GLOW_STATE_MANUAL:
		strncpy(out_str, "MANUAL", out_size);
		break;
	case GLOW_STATE_LOW_BATTERY:
		strncpy(out_str, "LOW_BATTERY", out_size);
		break;
		case GLOW_STATE_BROKEN_WIRE:
		strncpy(out_str, "BROKEN_WIRE", out_size);
		break;
	case GLOW_STATE_ERROR:
		strncpy(out_str, "ERROR", out_size);
		break;
	default:
		strncpy(out_str, "UNKNOWN", out_size);
		break;
	}
	// out_str[out_size - 1] = '\0';
}

static esp_err_t web_send_control_command(httpd_req_t *req, const glow_command_t *cmd)
{
	if (!g_context || !g_context->control_queue)
	{
		httpd_resp_set_status(req, "503 Service Unavailable");
		httpd_resp_set_type(req, "application/json");
		return httpd_resp_sendstr(req, "{\"ok\":false,\"error\":\"control_queue_not_ready\"}");
	}

	if (xQueueSend(g_context->control_queue, cmd, 0) != pdTRUE)
	{
		httpd_resp_set_status(req, "503 Service Unavailable");
		httpd_resp_set_type(req, "application/json");
		return httpd_resp_sendstr(req, "{\"ok\":false,\"error\":\"control_queue_full\"}");
	}

	httpd_resp_set_type(req, "application/json");
	return httpd_resp_sendstr(req, "{\"ok\":true}");
}

static esp_err_t web_read_request_body(httpd_req_t *req, char *buffer, size_t buffer_size)
{
	if (req->content_len <= 0 || (size_t)req->content_len >= buffer_size)
	{
		return ESP_ERR_INVALID_SIZE;
	}

	int total = 0;
	while (total < req->content_len)
	{
		int ret = httpd_req_recv(req, buffer + total, req->content_len - total);
		if (ret <= 0)
		{
			return ESP_FAIL;
		}
		total += ret;
	}

	buffer[total] = '\0';
	return ESP_OK;
}

static esp_err_t web_led_set_handler(httpd_req_t *req)
{
	char query[96] = {0};
	char r_str[8] = {0};
	char g_str[8] = {0};
	char b_str[8] = {0};

	if (httpd_req_get_url_query_str(req, query, sizeof(query)) != ESP_OK ||
		httpd_query_key_value(query, "r", r_str, sizeof(r_str)) != ESP_OK ||
		httpd_query_key_value(query, "g", g_str, sizeof(g_str)) != ESP_OK ||
		httpd_query_key_value(query, "b", b_str, sizeof(b_str)) != ESP_OK)
	{
		httpd_resp_set_status(req, "400 Bad Request");
		httpd_resp_set_type(req, "application/json");
		return httpd_resp_sendstr(req, "{\"ok\":false,\"error\":\"missing_query_params\"}");
	}

	int r = atoi(r_str);
	int g = atoi(g_str);
	int b = atoi(b_str);
	if (r < 0 || r > 255 || g < 0 || g > 255 || b < 0 || b > 255)
	{
		httpd_resp_set_status(req, "400 Bad Request");
		httpd_resp_set_type(req, "application/json");
		return httpd_resp_sendstr(req, "{\"ok\":false,\"error\":\"value_out_of_range\"}");
	}

	glow_command_t cmd = {
		.command_id = GLOW_CMD_SET_LED_COLOR,
		.data = {(uint8_t)r, (uint8_t)g, (uint8_t)b},
	};
	return web_send_control_command(req, &cmd);
}

static esp_err_t web_power_set_handler(httpd_req_t *req)
{
	char query[64] = {0};
	char en_str[8] = {0};

	if (httpd_req_get_url_query_str(req, query, sizeof(query)) != ESP_OK ||
		httpd_query_key_value(query, "en", en_str, sizeof(en_str)) != ESP_OK)
	{
		httpd_resp_set_status(req, "400 Bad Request");
		httpd_resp_set_type(req, "application/json");
		return httpd_resp_sendstr(req, "{\"ok\":false,\"error\":\"missing_query_params\"}");
	}

	int en = atoi(en_str);
	if (en != 0 && en != 1)
	{
		httpd_resp_set_status(req, "400 Bad Request");
		httpd_resp_set_type(req, "application/json");
		return httpd_resp_sendstr(req, "{\"ok\":false,\"error\":\"invalid_enable_value\"}");
	}

	glow_command_t cmd = {
		.command_id = GLOW_CMD_ENABLE_POWER,
		.data = {(uint8_t)en},
	};
	return web_send_control_command(req, &cmd);
}

static esp_err_t web_voltage_set_handler(httpd_req_t *req)
{
	char query[80] = {0};
	char v_str[16] = {0};
	char *endptr = NULL;

	if (httpd_req_get_url_query_str(req, query, sizeof(query)) != ESP_OK ||
		httpd_query_key_value(query, "v", v_str, sizeof(v_str)) != ESP_OK)
	{
		httpd_resp_set_status(req, "400 Bad Request");
		httpd_resp_set_type(req, "application/json");
		return httpd_resp_sendstr(req, "{\"ok\":false,\"error\":\"missing_query_params\"}");
	}

	float voltage = strtof(v_str, &endptr);
	if (endptr == v_str || *endptr != '\0' || voltage < 0.0f || voltage > 5.5f)
	{
		httpd_resp_set_status(req, "400 Bad Request");
		httpd_resp_set_type(req, "application/json");
		return httpd_resp_sendstr(req, "{\"ok\":false,\"error\":\"invalid_voltage\"}");
	}

	glow_command_t cmd = {
		.command_id = GLOW_CMD_SET_VOLTAGE,
	};
	memcpy(cmd.data, &voltage, sizeof(voltage));
	return web_send_control_command(req, &cmd);
}

static esp_err_t web_measurement_get_handler(httpd_req_t *req)
{
	if (!g_context || !g_context->measurement_mutex)
	{
		httpd_resp_set_status(req, "503 Service Unavailable");
		httpd_resp_set_type(req, "application/json");
		return httpd_resp_sendstr(req, "{\"ok\":false,\"error\":\"measurement_not_ready\"}");
	}

	bool measurement_available = false;
	if (xSemaphoreTake(g_context->measurement_mutex, pdMS_TO_TICKS(50)) == pdTRUE)
	{
		measurement_available = true;
	}
	if (!measurement_available)
	{
		httpd_resp_set_status(req, "503 Service Unavailable");
		httpd_resp_set_type(req, "application/json");
		return httpd_resp_sendstr(req, "{\"ok\":false,\"error\":\"measurement_queue_not_ready\"}");
	}
	char response[240];
	snprintf(response, sizeof(response),
			 "{\"ok\":true,\"vout_mV\":%u,\"iout_mA\":%u,\"vbus_mV\":%u,\"vbat_mV\":%u,\"rc_pwm_percentage\":%u,\"rc_pwm_valid\":%s}",
			 (unsigned)g_context->latest_measurement.vout_mV,
			 (unsigned)g_context->latest_measurement.iout_mA,
			 (unsigned)g_context->latest_measurement.vbus_mV,
			 (unsigned)g_context->latest_measurement.vbat_mV,
			 (unsigned)g_context->latest_measurement.rc_pwm_percentage,
			 g_context->latest_measurement.rc_pwm_signal_valid ? "true" : "false");
	httpd_resp_set_type(req, "application/json");
	xSemaphoreGive(g_context->measurement_mutex);
	return httpd_resp_sendstr(req, response);
}

static esp_err_t web_live_get_handler(httpd_req_t *req)
{
	if (!g_context || !g_context->measurement_mutex || !g_context->system_status)
	{
		httpd_resp_set_status(req, "503 Service Unavailable");
		httpd_resp_set_type(req, "application/json");
		return httpd_resp_sendstr(req, "{\"ok\":false,\"error\":\"context_not_ready\"}");
	}

	bool has_measurement = false;
	// Wait for the measurement mutex to not display half-formed data, but don't block if it's currently locked (e.g. during a measurement update)
	if (xSemaphoreTake(g_context->measurement_mutex, pdMS_TO_TICKS(100)) == pdTRUE)
	{
		has_measurement = true;
	}

	char response[320];
	bool s_manual_mode = (xEventGroupGetBits(g_context->system_status) & GLOW_MANUAL_MODE_BIT) != 0;
	char system_state[32];
	glow_state_to_string(g_context->system_state, system_state, sizeof(system_state));
	uint32_t rc_pwm_percentage = has_measurement ? g_context->latest_measurement.rc_pwm_percentage : 0;
	char rc_measurement[16];
	if (has_measurement && g_context->latest_measurement.rc_pwm_signal_valid)
	{
		sprintf(rc_measurement, "%u%%", (unsigned)rc_pwm_percentage);
	}
	else
	{
		strcpy(rc_measurement, "no_signal");
	}

	snprintf(
		response,
		sizeof(response),
		"{\"ok\":true,\"system_state\":\"%s\",\"manual_mode\":%s,\"vout_mV\":%u,\"iout_mA\":%u,\"vbus_mV\":%u,\"vbat_mV\":%u,\"rc_measurement\":\"%s\",\"rc_pwm_percentage\":%u,\"rc_pwm_valid\":%s}",
		system_state,
		s_manual_mode ? "true" : "false",
		(unsigned)(has_measurement ? g_context->latest_measurement.vout_mV : 0),
		(unsigned)(has_measurement ? g_context->latest_measurement.iout_mA : 0),
		(unsigned)(has_measurement ? g_context->latest_measurement.vbus_mV : 0),
		(unsigned)(has_measurement ? g_context->latest_measurement.vbat_mV : 0),
		rc_measurement,
		(unsigned)rc_pwm_percentage,
		(has_measurement && g_context->latest_measurement.rc_pwm_signal_valid) ? "true" : "false");
	if (has_measurement)
	{
		xSemaphoreGive(g_context->measurement_mutex);
	}

	httpd_resp_set_type(req, "application/json");
	return httpd_resp_sendstr(req, response);
}

static esp_err_t web_mode_set_handler(httpd_req_t *req)
{
	char query[64] = {0};
	char manual_str[8] = {0};

	if (httpd_req_get_url_query_str(req, query, sizeof(query)) != ESP_OK ||
		httpd_query_key_value(query, "manual", manual_str, sizeof(manual_str)) != ESP_OK)
	{
		httpd_resp_set_status(req, "400 Bad Request");
		httpd_resp_set_type(req, "application/json");
		return httpd_resp_sendstr(req, "{\"ok\":false,\"error\":\"missing_query_params\"}");
	}

	int manual = atoi(manual_str);
	if (manual != 0 && manual != 1)
	{
		httpd_resp_set_status(req, "400 Bad Request");
		httpd_resp_set_type(req, "application/json");
		return httpd_resp_sendstr(req, "{\"ok\":false,\"error\":\"invalid_manual_value\"}");
	}

	glow_command_t cmd = {
		.command_id = GLOW_CMD_MANUAL_MODE,
		.data = {(uint8_t)manual},
	};
	xQueueSend(g_context->control_queue, &cmd, 0);

	httpd_resp_set_type(req, "application/json");
	return httpd_resp_sendstr(req, (manual == 1) ? "{\"ok\":true,\"manual_mode\":true}" : "{\"ok\":true,\"manual_mode\":false}");
}

static esp_err_t web_settings_load_handler(httpd_req_t *req)
{
	glow_device_config_t config = {0};
	esp_err_t err = glow_storage_get_device_config(&config);
	if (err != ESP_OK)
	{
		httpd_resp_set_status(req, "500 Internal Server Error");
		httpd_resp_set_type(req, "application/json");
		return httpd_resp_sendstr(req, "{\"ok\":false,\"error\":\"settings_not_available\"}");
	}

	char response[512];
	snprintf(
		response,
		sizeof(response),
		"{\"ok\":true,\"battery\":{\"ncells\":%u,\"low_voltage\":%.3f,\"crit_voltage\":%.3f},\"leds\":{\"num_leds\":%u,\"brightness\":%.3f},\"glow\":{\"voltage\":%.3f,\"boost\":%.3f,\"boost_time\":%.3f},\"receiver\":{\"type\":\"%s\",\"failsafe_timeout\":%.3f,\"threshold\":%d}}",
		(unsigned)config.battery_ncells,
		(double)config.battery_low_voltage,
		(double)config.battery_crit_voltage,
		(unsigned)config.leds_num_leds,
		(double)config.leds_brightness,
		(double)config.glow_voltage,
		(double)config.glow_boost,
		(double)config.glow_boost_time,
		config.receiver_type,
		(double)config.receiver_failsafe_timeout,
		config.receiver_threshold);

	httpd_resp_set_type(req, "application/json");
	return httpd_resp_sendstr(req, response);
}

static esp_err_t web_settings_save_handler(httpd_req_t *req)
{
	char query[256] = {0};
	char led_count_str[16] = {0};
	char warning_str[24] = {0};
	char cutoff_str[24] = {0};
	char cell_count_str[24] = {0};
	char *endptr = NULL;

	if (httpd_req_get_url_query_str(req, query, sizeof(query)) != ESP_OK ||
		httpd_query_key_value(query, "led_count", led_count_str, sizeof(led_count_str)) != ESP_OK ||
		httpd_query_key_value(query, "battery_low_warning_v", warning_str, sizeof(warning_str)) != ESP_OK ||
		httpd_query_key_value(query, "battery_low_cutoff_v", cutoff_str, sizeof(cutoff_str)) != ESP_OK ||
		httpd_query_key_value(query, "cell_count", cell_count_str, sizeof(cell_count_str)) != ESP_OK)
	{
		httpd_resp_set_status(req, "400 Bad Request");
		httpd_resp_set_type(req, "application/json");
		return httpd_resp_sendstr(req, "{\"ok\":false,\"error\":\"missing_query_params\"}");
	}

	endptr = NULL;
	long led_count = strtol(led_count_str, &endptr, 10);
	if (endptr == led_count_str || *endptr != '\0')
	{
		httpd_resp_set_status(req, "400 Bad Request");
		httpd_resp_set_type(req, "application/json");
		return httpd_resp_sendstr(req, "{\"ok\":false,\"error\":\"invalid_led_count\"}");
	}

	endptr = NULL;
	float warning_v = strtof(warning_str, &endptr);
	if (endptr == warning_str || *endptr != '\0')
	{
		httpd_resp_set_status(req, "400 Bad Request");
		httpd_resp_set_type(req, "application/json");
		return httpd_resp_sendstr(req, "{\"ok\":false,\"error\":\"invalid_warning_voltage\"}");
	}

	endptr = NULL;
	float cutoff_v = strtof(cutoff_str, &endptr);
	if (endptr == cutoff_str || *endptr != '\0')
	{
		httpd_resp_set_status(req, "400 Bad Request");
		httpd_resp_set_type(req, "application/json");
		return httpd_resp_sendstr(req, "{\"ok\":false,\"error\":\"invalid_cutoff_voltage\"}");
	}

	uint8_t cell_count = 0;
	if (strcmp(cell_count_str, "auto") != 0)
	{
		endptr = NULL;
		long cell_count_num = strtol(cell_count_str, &endptr, 10);
		if (endptr == cell_count_str || *endptr != '\0')
		{
			httpd_resp_set_status(req, "400 Bad Request");
			httpd_resp_set_type(req, "application/json");
			return httpd_resp_sendstr(req, "{\"ok\":false,\"error\":\"invalid_cell_count\"}");
		}
		cell_count = (uint8_t)cell_count_num;
	}

	glow_device_config_t config = {0};
	ESP_RETURN_ON_ERROR(glow_storage_get_device_config(&config), TAG, "Failed to load current config");

	config.leds_num_leds = (uint8_t)led_count;
	config.battery_low_voltage = warning_v;
	config.battery_crit_voltage = cutoff_v;
	config.battery_ncells = cell_count;

	ESP_RETURN_ON_ERROR(glow_storage_set_device_config(&config), TAG, "Failed to set settings");
	ESP_RETURN_ON_ERROR(glow_storage_save_config(), TAG, "Failed to save settings");
	

	httpd_resp_set_type(req, "application/json");
	return httpd_resp_sendstr(req, "{\"ok\":true}");
}

static esp_err_t web_config_get_handler(httpd_req_t *req)
{
	char json[WEB_CONFIG_JSON_MAX_LEN] = {0};
	esp_err_t err = glow_storage_get_config_json(json, sizeof(json));
	if (err != ESP_OK)
	{
		httpd_resp_set_status(req, "500 Internal Server Error");
		httpd_resp_set_type(req, "application/json");
		return httpd_resp_sendstr(req, "{\"ok\":false,\"error\":\"config_not_available\"}");
	}

	httpd_resp_set_type(req, "application/json");
	return httpd_resp_sendstr(req, json);
}

static esp_err_t web_config_post_handler(httpd_req_t *req)
{
	char json[WEB_CONFIG_JSON_MAX_LEN] = {0};
	esp_err_t read_err = web_read_request_body(req, json, sizeof(json));
	if (read_err != ESP_OK)
	{
		httpd_resp_set_status(req, "400 Bad Request");
		httpd_resp_set_type(req, "application/json");
		return httpd_resp_sendstr(req, "{\"ok\":false,\"error\":\"invalid_request_body\"}");
	}

	esp_err_t save_err = glow_storage_set_config_json(json);
	if (save_err != ESP_OK)
	{
		httpd_resp_set_status(req, "400 Bad Request");
		httpd_resp_set_type(req, "application/json");
		return httpd_resp_sendstr(req, "{\"ok\":false,\"error\":\"config_validation_failed\"}");
	}

	httpd_resp_set_type(req, "application/json");
	return httpd_resp_sendstr(req, "{\"ok\":true}");
}

static esp_err_t web_root_get_handler(httpd_req_t *req)
{
	httpd_resp_set_type(req, "text/html");
	const char *html = (const char *)_binary_index_html_start;
	const size_t html_len = (size_t)(_binary_index_html_end - _binary_index_html_start);
	return httpd_resp_send(req, html, html_len);
}

static esp_err_t web_captive_get_handler(httpd_req_t *req)
{
	if (strcmp(req->uri, "/") == 0)
	{
		return web_root_get_handler(req);
	}

	httpd_resp_set_status(req, "302 Found");
	httpd_resp_set_hdr(req, "Location", "http://192.168.4.1/");
	return httpd_resp_send(req, "", 0);
}

static esp_err_t web_start_http_server(void)
{
	httpd_config_t config = HTTPD_DEFAULT_CONFIG();
	config.uri_match_fn = httpd_uri_match_wildcard;
	config.max_uri_handlers = 16;

	esp_err_t err = httpd_start(&s_http_server, &config);
	if (err != ESP_OK)
	{
		ESP_LOGE(TAG, "Failed to start http server: %s", esp_err_to_name(err));
		return err;
	}

	const httpd_uri_t captive_uri = {
		.uri = "/*",
		.method = HTTP_GET,
		.handler = web_captive_get_handler,
		.user_ctx = NULL,
	};
	const httpd_uri_t led_uri = {
		.uri = "/api/led",
		.method = HTTP_GET,
		.handler = web_led_set_handler,
		.user_ctx = NULL,
	};
	const httpd_uri_t power_uri = {
		.uri = "/api/power",
		.method = HTTP_GET,
		.handler = web_power_set_handler,
		.user_ctx = NULL,
	};
	const httpd_uri_t voltage_uri = {
		.uri = "/api/voltage",
		.method = HTTP_GET,
		.handler = web_voltage_set_handler,
		.user_ctx = NULL,
	};
	const httpd_uri_t measurement_uri = {
		.uri = "/api/measurement",
		.method = HTTP_GET,
		.handler = web_measurement_get_handler,
		.user_ctx = NULL,
	};
	const httpd_uri_t live_uri = {
		.uri = "/api/live",
		.method = HTTP_GET,
		.handler = web_live_get_handler,
		.user_ctx = NULL,
	};
	const httpd_uri_t mode_uri = {
		.uri = "/api/mode",
		.method = HTTP_GET,
		.handler = web_mode_set_handler,
		.user_ctx = NULL,
	};
	const httpd_uri_t settings_load_uri = {
		.uri = "/api/settings/load",
		.method = HTTP_GET,
		.handler = web_settings_load_handler,
		.user_ctx = NULL,
	};
	const httpd_uri_t settings_save_uri = {
		.uri = "/api/settings/save",
		.method = HTTP_GET,
		.handler = web_settings_save_handler,
		.user_ctx = NULL,
	};
	const httpd_uri_t config_get_uri = {
		.uri = "/api/config",
		.method = HTTP_GET,
		.handler = web_config_get_handler,
		.user_ctx = NULL,
	};
	const httpd_uri_t config_post_uri = {
		.uri = "/api/config",
		.method = HTTP_POST,
		.handler = web_config_post_handler,
		.user_ctx = NULL,
	};

	err = httpd_register_uri_handler(s_http_server, &led_uri);
	if (err != ESP_OK)
	{
		ESP_LOGE(TAG, "Failed to register LED API handler: %s", esp_err_to_name(err));
		httpd_stop(s_http_server);
		s_http_server = NULL;
		return err;
	}
	err = httpd_register_uri_handler(s_http_server, &power_uri);
	if (err != ESP_OK)
	{
		ESP_LOGE(TAG, "Failed to register power API handler: %s", esp_err_to_name(err));
		httpd_stop(s_http_server);
		s_http_server = NULL;
		return err;
	}
	err = httpd_register_uri_handler(s_http_server, &voltage_uri);
	if (err != ESP_OK)
	{
		ESP_LOGE(TAG, "Failed to register voltage API handler: %s", esp_err_to_name(err));
		httpd_stop(s_http_server);
		s_http_server = NULL;
		return err;
	}
	err = httpd_register_uri_handler(s_http_server, &measurement_uri);
	if (err != ESP_OK)
	{
		ESP_LOGE(TAG, "Failed to register measurement API handler: %s", esp_err_to_name(err));
		httpd_stop(s_http_server);
		s_http_server = NULL;
		return err;
	}
	err = httpd_register_uri_handler(s_http_server, &live_uri);
	if (err != ESP_OK)
	{
		ESP_LOGE(TAG, "Failed to register live API handler: %s", esp_err_to_name(err));
		httpd_stop(s_http_server);
		s_http_server = NULL;
		return err;
	}
	err = httpd_register_uri_handler(s_http_server, &mode_uri);
	if (err != ESP_OK)
	{
		ESP_LOGE(TAG, "Failed to register mode API handler: %s", esp_err_to_name(err));
		httpd_stop(s_http_server);
		s_http_server = NULL;
		return err;
	}
	err = httpd_register_uri_handler(s_http_server, &settings_load_uri);
	if (err != ESP_OK)
	{
		ESP_LOGE(TAG, "Failed to register settings load API handler: %s", esp_err_to_name(err));
		httpd_stop(s_http_server);
		s_http_server = NULL;
		return err;
	}
	err = httpd_register_uri_handler(s_http_server, &settings_save_uri);
	if (err != ESP_OK)
	{
		ESP_LOGE(TAG, "Failed to register settings save API handler: %s", esp_err_to_name(err));
		httpd_stop(s_http_server);
		s_http_server = NULL;
		return err;
	}
	err = httpd_register_uri_handler(s_http_server, &config_get_uri);
	if (err != ESP_OK)
	{
		ESP_LOGE(TAG, "Failed to register config GET API handler: %s", esp_err_to_name(err));
		httpd_stop(s_http_server);
		s_http_server = NULL;
		return err;
	}
	err = httpd_register_uri_handler(s_http_server, &config_post_uri);
	if (err != ESP_OK)
	{
		ESP_LOGE(TAG, "Failed to register config POST API handler: %s", esp_err_to_name(err));
		httpd_stop(s_http_server);
		s_http_server = NULL;
		return err;
	}

	err = httpd_register_uri_handler(s_http_server, &captive_uri);
	if (err != ESP_OK)
	{
		ESP_LOGE(TAG, "Failed to register URI handler: %s", esp_err_to_name(err));
		httpd_stop(s_http_server);
		s_http_server = NULL;
		return err;
	}

	ESP_LOGI(TAG, "HTTP server started");
	return ESP_OK;
}

static esp_err_t web_start_wifi_ap(void)
{
	wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
	ESP_RETURN_ON_ERROR(esp_wifi_init(&cfg), TAG, "esp_wifi_init failed");
	ESP_RETURN_ON_ERROR(esp_wifi_set_mode(WIFI_MODE_AP), TAG, "esp_wifi_set_mode failed");

	wifi_config_t wifi_config = {
		.ap = {
			.ssid = GLOW_WEB_AP_SSID,
			.password = GLOW_WEB_AP_PASS,
			.ssid_len = strlen(GLOW_WEB_AP_SSID),
			.channel = GLOW_WEB_AP_CHANNEL,
			.max_connection = GLOW_WEB_AP_MAX_CONN,
			.authmode = WIFI_AUTH_WPA2_PSK,
			.pmf_cfg = {
				.required = false,
			},
		},
	};

	ESP_RETURN_ON_ERROR(esp_wifi_set_config(WIFI_IF_AP, &wifi_config), TAG, "esp_wifi_set_config failed");
	ESP_RETURN_ON_ERROR(esp_wifi_start(), TAG, "esp_wifi_start failed");
	ESP_RETURN_ON_ERROR(esp_wifi_set_max_tx_power(GLOW_WEB_AP_TX_POWER_QDBM), TAG, "esp_wifi_set_max_tx_power failed");

	ESP_LOGI(TAG, "Wi-Fi AP started: SSID=%s tx_power=%d qdBm", GLOW_WEB_AP_SSID, GLOW_WEB_AP_TX_POWER_QDBM);
	return ESP_OK;
}

static size_t dns_skip_name(const uint8_t *buf, size_t len, size_t offset)
{
	while (offset < len)
	{
		uint8_t label_len = buf[offset];
		offset++;
		if (label_len == 0)
		{
			return offset;
		}
		if ((label_len & 0xC0) == 0xC0)
		{
			if (offset < len)
			{
				return offset + 1;
			}
			return 0;
		}
		if (offset + label_len > len)
		{
			return 0;
		}
		offset += label_len;
	}
	return 0;
}

static int dns_build_response(const uint8_t *query, size_t query_len, uint8_t *resp, size_t resp_len)
{
	if (query_len < 12 || resp_len < 32)
	{
		return -1;
	}

	uint16_t qdcount = ((uint16_t)query[4] << 8) | query[5];
	if (qdcount == 0)
	{
		return -1;
	}

	size_t question_end = dns_skip_name(query, query_len, 12);
	if (question_end == 0 || question_end + 4 > query_len)
	{
		return -1;
	}

	size_t base_len = question_end + 4;
	size_t answer_len = 2 + 2 + 2 + 4 + 2 + 4;
	if (base_len + answer_len > resp_len)
	{
		return -1;
	}

	memcpy(resp, query, base_len);
	resp[2] = 0x81;
	resp[3] = 0x80;
	resp[6] = 0x00;
	resp[7] = 0x01;
	resp[8] = 0x00;
	resp[9] = 0x00;
	resp[10] = 0x00;
	resp[11] = 0x00;

	size_t pos = base_len;
	resp[pos++] = 0xC0;
	resp[pos++] = 0x0C;
	resp[pos++] = 0x00;
	resp[pos++] = 0x01;
	resp[pos++] = 0x00;
	resp[pos++] = 0x01;
	resp[pos++] = 0x00;
	resp[pos++] = 0x00;
	resp[pos++] = 0x00;
	resp[pos++] = 0x1E;
	resp[pos++] = 0x00;
	resp[pos++] = 0x04;
	memcpy(&resp[pos], s_ap_ip, sizeof(s_ap_ip));
	pos += sizeof(s_ap_ip);

	return (int)pos;
}

static void dns_captive_task(void *arg)
{
	(void)arg;
	int sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
	if (sock < 0)
	{
		ESP_LOGE(TAG, "DNS socket create failed");
		vTaskDelete(NULL);
		return;
	}

	struct sockaddr_in addr = {
		.sin_family = AF_INET,
		.sin_port = htons(DNS_PORT),
		.sin_addr.s_addr = htonl(INADDR_ANY),
	};

	if (bind(sock, (struct sockaddr *)&addr, sizeof(addr)) < 0)
	{
		ESP_LOGE(TAG, "DNS bind failed");
		close(sock);
		vTaskDelete(NULL);
		return;
	}

	ESP_LOGI(TAG, "DNS captive server started on UDP/53");

	uint8_t query[DNS_PACKET_SIZE];
	uint8_t response[DNS_PACKET_SIZE];
	while (1)
	{
		struct sockaddr_in from_addr;
		socklen_t from_len = sizeof(from_addr);
		int len = recvfrom(sock, query, sizeof(query), 0, (struct sockaddr *)&from_addr, &from_len);
		if (len <= 0)
		{
			continue;
		}

		int resp_len = dns_build_response(query, (size_t)len, response, sizeof(response));
		if (resp_len > 0)
		{
			sendto(sock, response, (size_t)resp_len, 0, (struct sockaddr *)&from_addr, from_len);
		}
	}
}

esp_err_t glow_web_init(void)
{
	if (s_initialized)
	{
		return ESP_OK;
	}

	if (g_context == NULL)
	{
		ESP_LOGE(TAG, "Glow_web cannot work without context!");
		return ESP_ERR_INVALID_STATE;
	}

	ESP_RETURN_ON_ERROR(esp_netif_init(), TAG, "esp_netif_init failed");

	esp_netif_create_default_wifi_ap();

	ESP_RETURN_ON_ERROR(web_start_wifi_ap(), TAG, "failed to start wifi ap");
	ESP_RETURN_ON_ERROR(web_start_http_server(), TAG, "failed to start web server");

	BaseType_t task_ok = xTaskCreate(dns_captive_task, "dns_captive", 4096, NULL, 5, NULL);
	if (task_ok != pdPASS)
	{
		ESP_LOGE(TAG, "Failed to create DNS task");
		return ESP_ERR_NO_MEM;
	}

	s_initialized = true;
	ESP_LOGI(TAG, "Web captive portal initialized");
	return ESP_OK;
}
