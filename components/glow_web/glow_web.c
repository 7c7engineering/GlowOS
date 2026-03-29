#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <sys/param.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include "glow_context.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_check.h"
#include "esp_event.h"
#include "esp_http_server.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include "nvs_flash.h"

#include "glow_web.h"

static const char *TAG = "GLOW_WEB";

extern glow_context_t *g_context;

#define GLOW_WEB_AP_SSID      "GlowOS"
#define GLOW_WEB_AP_PASS      "glowdriver"
#define GLOW_WEB_AP_CHANNEL   1
#define GLOW_WEB_AP_MAX_CONN  4
// ESP-IDF uses quarter-dBm units: 8 = 2 dBm, 20 = 5 dBm, 84 = 21 dBm.
#define GLOW_WEB_AP_TX_POWER_QDBM 20

#define DNS_PORT                    53
#define DNS_PACKET_SIZE             512

static const uint8_t s_ap_ip[4] = {192, 168, 4, 1};
static httpd_handle_t s_http_server;
static bool s_initialized;

static const char s_index_html[] =
	"<!doctype html><html><head><meta charset=\"utf-8\">"
	"<meta name=\"viewport\" content=\"width=device-width,initial-scale=1\">"
	"<title>GlowOS</title>"
	"<style>body{font-family:Arial,sans-serif;background:#f4f6f8;margin:0;padding:24px;color:#222;}"
	".card{max-width:720px;margin:0 auto;background:#fff;padding:24px;border-radius:12px;"
	"box-shadow:0 8px 24px rgba(0,0,0,.08);}h1{margin-top:0;}code{background:#eef2f6;padding:2px 6px;"
	"border-radius:6px;}label{display:block;margin-top:12px;}input[type=range]{width:100%;}"
	".swatch{height:56px;border-radius:10px;margin-top:14px;border:1px solid #d9e1e8;}"
	"button{margin-top:16px;padding:10px 16px;border:0;border-radius:8px;background:#0b6efd;color:#fff;font-weight:700;}"
	".row{display:flex;gap:8px;align-items:center;margin-top:10px;}"
	"input[type=number]{width:120px;padding:8px;border:1px solid #c9d3dd;border-radius:8px;}"
	".status{margin-top:10px;color:#4a5560;font-size:14px;}</style></head><body><div class=\"card\"><h1>GlowOS Captive Portal</h1>"
	"<p>Web interface is running.</p><p>Next step: connect control endpoints and queues.</p>"
	"<p>AP IP: <code>192.168.4.1</code></p>"
	"<h2>LED Control</h2>"
	"<label>Red: <span id=\"rv\">0</span><input id=\"r\" type=\"range\" min=\"0\" max=\"255\" value=\"0\"></label>"
	"<label>Green: <span id=\"gv\">0</span><input id=\"g\" type=\"range\" min=\"0\" max=\"255\" value=\"0\"></label>"
	"<label>Blue: <span id=\"bv\">127</span><input id=\"b\" type=\"range\" min=\"0\" max=\"255\" value=\"127\"></label>"
	"<div class=\"swatch\" id=\"sw\"></div>"
	"<button id=\"send\">Set LED</button>"
	"<div class=\"status\" id=\"st\">LED: Idle</div>"
	"<h2>Power</h2>"
	"<div class=\"row\"><button id=\"pon\">POWER ON</button><button id=\"poff\">POWER OFF</button></div>"
	"<div class=\"status\" id=\"stp\">Power: Idle</div>"
	"<h2>Voltage</h2>"
	"<div class=\"row\"><label for=\"v\" style=\"display:inline\">Voltage (V)</label><input id=\"v\" type=\"number\" step=\"0.05\" min=\"0\" max=\"5.5\" value=\"2.0\"></div>"
	"<button id=\"sendv\">Set Voltage</button>"
	"<div class=\"status\" id=\"stv\">Voltage: Idle</div>"
	"<h2>Measurements</h2>"
	"<div class=\"status\" id=\"sm\">Waiting for data...</div>"
	"<div class=\"row\"><code id=\"mvout\">Vout: -- mV</code></div>"
	"<div class=\"row\"><code id=\"miout\">Iout: -- mA</code></div>"
	"<div class=\"row\"><code id=\"mvbus\">VBUS: -- mV</code></div>"
	"<div class=\"row\"><code id=\"mvbat\">VBAT: -- mV</code></div>"
	"<script>"
	"const r=document.getElementById('r'),g=document.getElementById('g'),b=document.getElementById('b');"
	"const rv=document.getElementById('rv'),gv=document.getElementById('gv'),bv=document.getElementById('bv');"
	"const sw=document.getElementById('sw'),st=document.getElementById('st');"
	"const stp=document.getElementById('stp');"
	"const v=document.getElementById('v'),stv=document.getElementById('stv');"
	"const sm=document.getElementById('sm');"
	"const mvout=document.getElementById('mvout'),miout=document.getElementById('miout');"
	"const mvbus=document.getElementById('mvbus'),mvbat=document.getElementById('mvbat');"
	"function upd(){rv.textContent=r.value;gv.textContent=g.value;bv.textContent=b.value;sw.style.background='rgb('+r.value+','+g.value+','+b.value+')';}"
	"r.oninput=upd;g.oninput=upd;b.oninput=upd;upd();"
	"document.getElementById('send').onclick=async()=>{"
	"st.textContent='Sending...';"
	"try{const u='/api/led?r='+r.value+'&g='+g.value+'&b='+b.value;const res=await fetch(u);"
	"st.textContent=res.ok?'LED command queued':'Failed: '+res.status;}"
	"catch(e){st.textContent='Request error';}};"
	"document.getElementById('pon').onclick=async()=>{"
	"stp.textContent='Sending...';"
	"try{const res=await fetch('/api/power?en=1');"
	"stp.textContent=res.ok?'Power ON command queued':'Failed: '+res.status;}"
	"catch(e){stp.textContent='Request error';}};"
	"document.getElementById('poff').onclick=async()=>{"
	"stp.textContent='Sending...';"
	"try{const res=await fetch('/api/power?en=0');"
	"stp.textContent=res.ok?'Power OFF command queued':'Failed: '+res.status;}"
	"catch(e){stp.textContent='Request error';}};"
	"document.getElementById('sendv').onclick=async()=>{"
	"stv.textContent='Sending...';"
	"try{const u='/api/voltage?v='+encodeURIComponent(v.value);const res=await fetch(u);"
	"stv.textContent=res.ok?'Voltage command queued':'Failed: '+res.status;}"
	"catch(e){stv.textContent='Request error';}};"
	"async function refreshMeasurements(){"
	"try{const res=await fetch('/api/measurement');if(!res.ok){sm.textContent='Measurement API unavailable: '+res.status;return;}"
	"const d=await res.json();"
	"mvout.textContent='Vout: '+d.vout_mV+' mV';"
	"miout.textContent='Iout: '+d.iout_mA+' mA';"
	"mvbus.textContent='VBUS: '+d.vbus_mV+' mV';"
	"mvbat.textContent='VBAT: '+d.vbat_mV+' mV';"
	"sm.textContent='Updated: '+new Date().toLocaleTimeString();}"
	"catch(e){sm.textContent='Measurement request error';}}"
	"refreshMeasurements();setInterval(refreshMeasurements,1000);"
	"</script></div></body></html>";

static esp_err_t web_send_control_command(httpd_req_t *req, const glow_command_t *cmd)
{
	if (!g_context || !g_context->control_queue) {
		httpd_resp_set_status(req, "503 Service Unavailable");
		httpd_resp_set_type(req, "application/json");
		return httpd_resp_sendstr(req, "{\"ok\":false,\"error\":\"control_queue_not_ready\"}");
	}

	if (xQueueSend(g_context->control_queue, cmd, 0) != pdTRUE) {
		httpd_resp_set_status(req, "503 Service Unavailable");
		httpd_resp_set_type(req, "application/json");
		return httpd_resp_sendstr(req, "{\"ok\":false,\"error\":\"control_queue_full\"}");
	}

	httpd_resp_set_type(req, "application/json");
	return httpd_resp_sendstr(req, "{\"ok\":true}");
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
		httpd_query_key_value(query, "b", b_str, sizeof(b_str)) != ESP_OK) {
		httpd_resp_set_status(req, "400 Bad Request");
		httpd_resp_set_type(req, "application/json");
		return httpd_resp_sendstr(req, "{\"ok\":false,\"error\":\"missing_query_params\"}");
	}

	int r = atoi(r_str);
	int g = atoi(g_str);
	int b = atoi(b_str);
	if (r < 0 || r > 255 || g < 0 || g > 255 || b < 0 || b > 255) {
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
		httpd_query_key_value(query, "en", en_str, sizeof(en_str)) != ESP_OK) {
		httpd_resp_set_status(req, "400 Bad Request");
		httpd_resp_set_type(req, "application/json");
		return httpd_resp_sendstr(req, "{\"ok\":false,\"error\":\"missing_query_params\"}");
	}

	int en = atoi(en_str);
	if (en != 0 && en != 1) {
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
		httpd_query_key_value(query, "v", v_str, sizeof(v_str)) != ESP_OK) {
		httpd_resp_set_status(req, "400 Bad Request");
		httpd_resp_set_type(req, "application/json");
		return httpd_resp_sendstr(req, "{\"ok\":false,\"error\":\"missing_query_params\"}");
	}

	float voltage = strtof(v_str, &endptr);
	if (endptr == v_str || *endptr != '\0' || voltage < 0.0f || voltage > 5.5f) {
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
	if (!g_context || !g_context->measurement_queue) {
		httpd_resp_set_status(req, "503 Service Unavailable");
		httpd_resp_set_type(req, "application/json");
		return httpd_resp_sendstr(req, "{\"ok\":false,\"error\":\"measurement_queue_not_ready\"}");
	}

	glow_measurement_t measurement = {0};
	if (xQueuePeek(g_context->measurement_queue, &measurement, 0) != pdTRUE) {
		httpd_resp_set_status(req, "503 Service Unavailable");
		httpd_resp_set_type(req, "application/json");
		return httpd_resp_sendstr(req, "{\"ok\":false,\"error\":\"no_measurement_available\"}");
	}

	char response[160];
	snprintf(response, sizeof(response),
		"{\"ok\":true,\"vout_mV\":%u,\"iout_mA\":%u,\"vbus_mV\":%u,\"vbat_mV\":%u}",
		(unsigned)measurement.vout_mV,
		(unsigned)measurement.iout_mA,
		(unsigned)measurement.vbus_mV,
		(unsigned)measurement.vbat_mV);

	httpd_resp_set_type(req, "application/json");
	return httpd_resp_sendstr(req, response);
}

static esp_err_t web_root_get_handler(httpd_req_t *req)
{
	httpd_resp_set_type(req, "text/html");
	return httpd_resp_send(req, s_index_html, HTTPD_RESP_USE_STRLEN);
}

static esp_err_t web_captive_get_handler(httpd_req_t *req)
{
	if (strcmp(req->uri, "/") == 0) {
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

	esp_err_t err = httpd_start(&s_http_server, &config);
	if (err != ESP_OK) {
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

	err = httpd_register_uri_handler(s_http_server, &led_uri);
	if (err != ESP_OK) {
		ESP_LOGE(TAG, "Failed to register LED API handler: %s", esp_err_to_name(err));
		httpd_stop(s_http_server);
		s_http_server = NULL;
		return err;
	}
	err = httpd_register_uri_handler(s_http_server, &power_uri);
	if (err != ESP_OK) {
		ESP_LOGE(TAG, "Failed to register power API handler: %s", esp_err_to_name(err));
		httpd_stop(s_http_server);
		s_http_server = NULL;
		return err;
	}
	err = httpd_register_uri_handler(s_http_server, &voltage_uri);
	if (err != ESP_OK) {
		ESP_LOGE(TAG, "Failed to register voltage API handler: %s", esp_err_to_name(err));
		httpd_stop(s_http_server);
		s_http_server = NULL;
		return err;
	}
	err = httpd_register_uri_handler(s_http_server, &measurement_uri);
	if (err != ESP_OK) {
		ESP_LOGE(TAG, "Failed to register measurement API handler: %s", esp_err_to_name(err));
		httpd_stop(s_http_server);
		s_http_server = NULL;
		return err;
	}

	err = httpd_register_uri_handler(s_http_server, &captive_uri);
	if (err != ESP_OK) {
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
	while (offset < len) {
		uint8_t label_len = buf[offset];
		offset++;
		if (label_len == 0) {
			return offset;
		}
		if ((label_len & 0xC0) == 0xC0) {
			if (offset < len) {
				return offset + 1;
			}
			return 0;
		}
		if (offset + label_len > len) {
			return 0;
		}
		offset += label_len;
	}
	return 0;
}

static int dns_build_response(const uint8_t *query, size_t query_len, uint8_t *resp, size_t resp_len)
{
	if (query_len < 12 || resp_len < 32) {
		return -1;
	}

	uint16_t qdcount = ((uint16_t)query[4] << 8) | query[5];
	if (qdcount == 0) {
		return -1;
	}

	size_t question_end = dns_skip_name(query, query_len, 12);
	if (question_end == 0 || question_end + 4 > query_len) {
		return -1;
	}

	size_t base_len = question_end + 4;
	size_t answer_len = 2 + 2 + 2 + 4 + 2 + 4;
	if (base_len + answer_len > resp_len) {
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
	if (sock < 0) {
		ESP_LOGE(TAG, "DNS socket create failed");
		vTaskDelete(NULL);
		return;
	}

	struct sockaddr_in addr = {
		.sin_family = AF_INET,
		.sin_port = htons(DNS_PORT),
		.sin_addr.s_addr = htonl(INADDR_ANY),
	};

	if (bind(sock, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
		ESP_LOGE(TAG, "DNS bind failed");
		close(sock);
		vTaskDelete(NULL);
		return;
	}

	ESP_LOGI(TAG, "DNS captive server started on UDP/53");

	uint8_t query[DNS_PACKET_SIZE];
	uint8_t response[DNS_PACKET_SIZE];
	while (1) {
		struct sockaddr_in from_addr;
		socklen_t from_len = sizeof(from_addr);
		int len = recvfrom(sock, query, sizeof(query), 0, (struct sockaddr *)&from_addr, &from_len);
		if (len <= 0) {
			continue;
		}

		int resp_len = dns_build_response(query, (size_t)len, response, sizeof(response));
		if (resp_len > 0) {
			sendto(sock, response, (size_t)resp_len, 0, (struct sockaddr *)&from_addr, from_len);
		}
	}
}

esp_err_t glow_web_init(void)
{
	if (s_initialized) {
		return ESP_OK;
	}


	ESP_RETURN_ON_ERROR(esp_netif_init(), TAG, "esp_netif_init failed");
	
	esp_netif_create_default_wifi_ap();

	ESP_RETURN_ON_ERROR(web_start_wifi_ap(), TAG, "failed to start wifi ap");
	ESP_RETURN_ON_ERROR(web_start_http_server(), TAG, "failed to start web server");

	BaseType_t task_ok = xTaskCreate(dns_captive_task, "dns_captive", 4096, NULL, 5, NULL);
	if (task_ok != pdPASS) {
		ESP_LOGE(TAG, "Failed to create DNS task");
		return ESP_ERR_NO_MEM;
	}

	s_initialized = true;
	ESP_LOGI(TAG, "Web captive portal initialized");
	return ESP_OK;
}
