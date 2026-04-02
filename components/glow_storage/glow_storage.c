#include <stdio.h>
#include <stdbool.h>
#include <string.h>
#include <stdlib.h>
#include <errno.h>

#include "esp_err.h"
#include "esp_log.h"
#include "esp_vfs_fat.h"
#include "wear_levelling.h"
#include "esp_check.h"
#include "soc/soc_caps.h"
#include "nvs.h"

#include "glow_storage.h"

static const char *TAG = "GLOW_STORAGE";

#define GLOW_STORAGE_PARTITION_LABEL "fat"
#define GLOW_STORAGE_MOUNT_POINT "/storage"

#define GLOW_STORAGE_NVS_NAMESPACE "glow_storage"
#define GLOW_STORAGE_NVS_KEY_CONFIG "device_config_json"
#define GLOW_STORAGE_CONFIG_JSON_MAX_LEN 1024
#define GLOW_STORAGE_RECEIVER_TYPE_MAX_LEN 16

typedef struct {
    uint8_t battery_ncells;
    float battery_low_voltage;
    float battery_crit_voltage;
    uint8_t leds_num_leds;
    float leds_brightness;
    float glow_voltage;
    float glow_boost;
    float glow_boost_time;
    char receiver_type[GLOW_STORAGE_RECEIVER_TYPE_MAX_LEN];
    float receiver_failsafe_timeout;
    int receiver_threshold;
} glow_device_config_t;

static wl_handle_t s_wl_handle = WL_INVALID_HANDLE;
static bool s_mounted;
static glow_settings_t s_settings;
static glow_device_config_t s_device_config;
static char s_config_json[GLOW_STORAGE_CONFIG_JSON_MAX_LEN];

extern const uint8_t _binary_default_json_start[] asm("_binary_default_json_start");
extern const uint8_t _binary_default_json_end[] asm("_binary_default_json_end");

static void glow_storage_apply_legacy_settings(const glow_device_config_t *config)
{
    s_settings.led_count = config->leds_num_leds;
    s_settings.battery_low_warning_v = config->battery_low_voltage;
    s_settings.battery_low_cutoff_v = config->battery_crit_voltage;
    s_settings.cell_count = config->battery_ncells;
}

static bool glow_storage_is_receiver_type_valid(const char *type)
{
    return strcmp(type, "PPM") == 0 ||
           strcmp(type, "SBUS") == 0 ||
           strcmp(type, "IBUS") == 0 ||
           strcmp(type, "CRSF") == 0;
}

static bool glow_storage_find_key_value_start(const char *json, const char *key, const char **out_value_start)
{
    char pattern[64];
    snprintf(pattern, sizeof(pattern), "\"%s\"", key);

    const char *found = strstr(json, pattern);
    if (found == NULL) {
        return false;
    }

    const char *colon = strchr(found, ':');
    if (colon == NULL) {
        return false;
    }

    const char *value_start = colon + 1;
    while (*value_start == ' ' || *value_start == '\t' || *value_start == '\n' || *value_start == '\r') {
        value_start++;
    }

    *out_value_start = value_start;
    return true;
}

static bool glow_storage_parse_int_field(const char *json, const char *key, int *out_value)
{
    const char *value_start = NULL;
    if (!glow_storage_find_key_value_start(json, key, &value_start)) {
        return false;
    }

    errno = 0;
    char *endptr = NULL;
    long parsed = strtol(value_start, &endptr, 10);
    if (value_start == endptr || errno != 0) {
        return false;
    }

    *out_value = (int)parsed;
    return true;
}

static bool glow_storage_parse_float_field(const char *json, const char *key, float *out_value)
{
    const char *value_start = NULL;
    if (!glow_storage_find_key_value_start(json, key, &value_start)) {
        return false;
    }

    errno = 0;
    char *endptr = NULL;
    float parsed = strtof(value_start, &endptr);
    if (value_start == endptr || errno != 0) {
        return false;
    }

    *out_value = parsed;
    return true;
}

static bool glow_storage_parse_string_field(const char *json, const char *key, char *out_str, size_t out_size)
{
    const char *value_start = NULL;
    if (!glow_storage_find_key_value_start(json, key, &value_start)) {
        return false;
    }

    if (*value_start != '"') {
        return false;
    }

    value_start++;
    const char *value_end = strchr(value_start, '"');
    if (value_end == NULL) {
        return false;
    }

    size_t len = (size_t)(value_end - value_start);
    if (len + 1 > out_size) {
        return false;
    }

    memcpy(out_str, value_start, len);
    out_str[len] = '\0';
    return true;
}

static bool glow_storage_validate_device_config(const glow_device_config_t *config)
{
    if (!(config->battery_ncells == 0 || (config->battery_ncells >= 2 && config->battery_ncells <= 5))) {
        return false;
    }
    if (config->battery_low_voltage < 2.5f || config->battery_low_voltage > 3.7f) {
        return false;
    }
    if (config->battery_crit_voltage < 2.5f || config->battery_crit_voltage > 3.5f) {
        return false;
    }
    if (config->battery_crit_voltage > config->battery_low_voltage) {
        return false;
    }
    if (config->leds_num_leds > 5) {
        return false;
    }
    if (config->leds_brightness < 0.0f || config->leds_brightness > 1.0f) {
        return false;
    }
    if (config->glow_voltage < 0.0f || config->glow_voltage > 5.5f) {
        return false;
    }
    if (config->glow_boost < 0.0f || config->glow_boost > 6.0f) {
        return false;
    }
    if (config->glow_boost_time < 0.0f || config->glow_boost_time > 10.0f) {
        return false;
    }
    if (!glow_storage_is_receiver_type_valid(config->receiver_type)) {
        return false;
    }
    if (config->receiver_failsafe_timeout < 0.1f || config->receiver_failsafe_timeout > 5.0f) {
        return false;
    }
    if (config->receiver_threshold < 0 || config->receiver_threshold > 100) {
        return false;
    }

    return true;
}

static esp_err_t glow_storage_parse_device_config_json(const char *json, glow_device_config_t *out_config)
{
    int battery_ncells = 0;
    float battery_low_voltage = 0.0f;
    float battery_crit_voltage = 0.0f;
    int leds_num_leds = 0;
    float leds_brightness = 0.0f;
    float glow_voltage = 0.0f;
    float glow_boost = 0.0f;
    float glow_boost_time = 0.0f;
    char receiver_type[GLOW_STORAGE_RECEIVER_TYPE_MAX_LEN] = {0};
    float receiver_failsafe_timeout = 0.0f;
    int receiver_threshold = 0;

    if (!glow_storage_parse_int_field(json, "ncells", &battery_ncells) ||
        !glow_storage_parse_float_field(json, "low_voltage", &battery_low_voltage) ||
        !glow_storage_parse_float_field(json, "crit_voltage", &battery_crit_voltage) ||
        !glow_storage_parse_int_field(json, "num_leds", &leds_num_leds) ||
        !glow_storage_parse_float_field(json, "brightness", &leds_brightness) ||
        !glow_storage_parse_float_field(json, "voltage", &glow_voltage) ||
        !glow_storage_parse_float_field(json, "boost", &glow_boost) ||
        !glow_storage_parse_float_field(json, "boost_time", &glow_boost_time) ||
        !glow_storage_parse_string_field(json, "type", receiver_type, sizeof(receiver_type)) ||
        !glow_storage_parse_float_field(json, "failsafe_timeout", &receiver_failsafe_timeout) ||
        !glow_storage_parse_int_field(json, "threshold", &receiver_threshold)) {
        return ESP_ERR_INVALID_ARG;
    }

    glow_device_config_t parsed = {
        .battery_ncells = (uint8_t)battery_ncells,
        .battery_low_voltage = battery_low_voltage,
        .battery_crit_voltage = battery_crit_voltage,
        .leds_num_leds = (uint8_t)leds_num_leds,
        .leds_brightness = leds_brightness,
        .glow_voltage = glow_voltage,
        .glow_boost = glow_boost,
        .glow_boost_time = glow_boost_time,
        .receiver_failsafe_timeout = receiver_failsafe_timeout,
        .receiver_threshold = receiver_threshold,
    };
    strncpy(parsed.receiver_type, receiver_type, sizeof(parsed.receiver_type) - 1);

    if (!glow_storage_validate_device_config(&parsed)) {
        return ESP_ERR_INVALID_ARG;
    }

    *out_config = parsed;
    return ESP_OK;
}

static esp_err_t glow_storage_serialize_device_config_json(const glow_device_config_t *config, char *out_json, size_t out_size)
{
    int written = snprintf(
        out_json,
        out_size,
        "{\n"
        "  \"battery\": {\n"
        "    \"ncells\": %u,\n"
        "    \"low_voltage\": %.3f,\n"
        "    \"crit_voltage\": %.3f\n"
        "  },\n"
        "  \"leds\": {\n"
        "    \"num_leds\": %u,\n"
        "    \"brightness\": %.3f\n"
        "  },\n"
        "  \"glow\": {\n"
        "    \"voltage\": %.3f,\n"
        "    \"boost\": %.3f,\n"
        "    \"boost_time\": %.3f\n"
        "  },\n"
        "  \"receiver\": {\n"
        "    \"type\": \"%s\",\n"
        "    \"failsafe_timeout\": %.3f,\n"
        "    \"threshold\": %d\n"
        "  }\n"
        "}\n",
        (unsigned)config->battery_ncells,
        (double)config->battery_low_voltage,
        (double)config->battery_crit_voltage,
        (unsigned)config->leds_num_leds,
        (double)config->leds_brightness,
        (double)config->glow_voltage,
        (double)config->glow_boost,
        (double)config->glow_boost_time,
        config->receiver_type,
        (double)config->receiver_failsafe_timeout,
        config->receiver_threshold
    );

    if (written <= 0 || (size_t)written >= out_size) {
        return ESP_ERR_INVALID_SIZE;
    }

    return ESP_OK;
}

static esp_err_t glow_storage_nvs_save_json(const char *json)
{
    nvs_handle_t handle;
    esp_err_t err = nvs_open(GLOW_STORAGE_NVS_NAMESPACE, NVS_READWRITE, &handle);
    if (err != ESP_OK) {
        return err;
    }

    err = nvs_set_str(handle, GLOW_STORAGE_NVS_KEY_CONFIG, json);
    if (err == ESP_OK) {
        err = nvs_commit(handle);
    }

    nvs_close(handle);
    return err;
}

static esp_err_t glow_storage_nvs_load_json(char *out_json, size_t out_size)
{
    nvs_handle_t handle;
    esp_err_t err = nvs_open(GLOW_STORAGE_NVS_NAMESPACE, NVS_READONLY, &handle);
    if (err != ESP_OK) {
        return err;
    }

    size_t required = 0;
    err = nvs_get_str(handle, GLOW_STORAGE_NVS_KEY_CONFIG, NULL, &required);
    if (err != ESP_OK) {
        nvs_close(handle);
        return err;
    }
    if (required > out_size) {
        nvs_close(handle);
        return ESP_ERR_INVALID_SIZE;
    }

    err = nvs_get_str(handle, GLOW_STORAGE_NVS_KEY_CONFIG, out_json, &required);
    nvs_close(handle);
    return err;
}

static esp_err_t glow_storage_get_default_json(char *out_json, size_t out_size)
{
    size_t default_len = (size_t)(_binary_default_json_end - _binary_default_json_start);
    if (default_len + 1 > out_size) {
        return ESP_ERR_INVALID_SIZE;
    }

    memcpy(out_json, _binary_default_json_start, default_len);
    out_json[default_len] = '\0';
    return ESP_OK;
}

static esp_err_t glow_storage_set_config_json_internal(const char *json, bool persist)
{
    if (json == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    glow_device_config_t parsed;
    ESP_RETURN_ON_ERROR(glow_storage_parse_device_config_json(json, &parsed), TAG, "Config JSON parse/validation failed");

    char *canonical = malloc(GLOW_STORAGE_CONFIG_JSON_MAX_LEN);
    if (canonical == NULL) {
        ESP_LOGE(TAG, "Failed to allocate temporary config buffer");
        return ESP_ERR_NO_MEM;
    }

    esp_err_t err = glow_storage_serialize_device_config_json(&parsed, canonical, GLOW_STORAGE_CONFIG_JSON_MAX_LEN);
    if (err != ESP_OK) {
        free(canonical);
        ESP_LOGE(TAG, "Failed to serialize config");
        return err;
    }

    if (strlen(canonical) + 1 > sizeof(s_config_json)) {
        free(canonical);
        return ESP_ERR_INVALID_SIZE;
    }

    s_device_config = parsed;
    memcpy(s_config_json, canonical, strlen(canonical) + 1);
    glow_storage_apply_legacy_settings(&s_device_config);

    if (persist) {
        err = glow_storage_nvs_save_json(s_config_json);
        if (err != ESP_OK) {
            free(canonical);
            ESP_LOGE(TAG, "Failed to persist config JSON to NVS");
            return err;
        }
    }

    free(canonical);
    return ESP_OK;
}

static esp_err_t glow_storage_mount_fat_partition(void)
{
    if (s_mounted) {
        return ESP_OK;
    }

    esp_vfs_fat_mount_config_t mount_config = {
        .format_if_mount_failed = false,
        .max_files = 4,
        .allocation_unit_size = 4096,
        .disk_status_check_enable = false,
        .use_one_fat = false,
    };

    esp_err_t err = esp_vfs_fat_spiflash_mount_rw_wl(
        GLOW_STORAGE_MOUNT_POINT,
        GLOW_STORAGE_PARTITION_LABEL,
        &mount_config,
        &s_wl_handle
    );
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "FAT mount failed on '%s': %s", GLOW_STORAGE_PARTITION_LABEL, esp_err_to_name(err));
        ESP_LOGW(TAG, "Retrying with format_if_mount_failed=true (first-boot recovery path)");

        mount_config.format_if_mount_failed = true;
        err = esp_vfs_fat_spiflash_mount_rw_wl(
            GLOW_STORAGE_MOUNT_POINT,
            GLOW_STORAGE_PARTITION_LABEL,
            &mount_config,
            &s_wl_handle
        );
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "Failed to mount/format FAT partition '%s': %s", GLOW_STORAGE_PARTITION_LABEL, esp_err_to_name(err));
            return err;
        }

        ESP_LOGW(TAG, "FAT partition was formatted and mounted at %s", GLOW_STORAGE_MOUNT_POINT);
    }

    s_mounted = true;
    ESP_LOGI(TAG, "Mounted FAT partition '%s' at %s", GLOW_STORAGE_PARTITION_LABEL, GLOW_STORAGE_MOUNT_POINT);
    return ESP_OK;
}

esp_err_t glow_storage_init(void)
{
    ESP_RETURN_ON_ERROR(glow_storage_mount_fat_partition(), TAG, "FAT mount failed");

    char *nvs_json = malloc(GLOW_STORAGE_CONFIG_JSON_MAX_LEN);
    if (nvs_json == NULL) {
        ESP_LOGE(TAG, "Failed to allocate NVS JSON buffer");
        return ESP_ERR_NO_MEM;
    }
    nvs_json[0] = '\0';

    esp_err_t nvs_err = glow_storage_nvs_load_json(nvs_json, GLOW_STORAGE_CONFIG_JSON_MAX_LEN);
    if (nvs_err == ESP_OK) {
        esp_err_t parse_err = glow_storage_set_config_json_internal(nvs_json, false);
        if (parse_err != ESP_OK) {
            ESP_LOGW(TAG, "Stored NVS config invalid, falling back to default JSON");
            nvs_err = ESP_ERR_INVALID_RESPONSE;
        }
    }

    free(nvs_json);

    if (nvs_err != ESP_OK) {
        char *default_json = malloc(GLOW_STORAGE_CONFIG_JSON_MAX_LEN);
        if (default_json == NULL) {
            ESP_LOGE(TAG, "Failed to allocate default JSON buffer");
            return ESP_ERR_NO_MEM;
        }

        esp_err_t err = glow_storage_get_default_json(default_json, GLOW_STORAGE_CONFIG_JSON_MAX_LEN);
        if (err != ESP_OK) {
            free(default_json);
            return err;
        }

        err = glow_storage_set_config_json_internal(default_json, true);
        free(default_json);
        if (err != ESP_OK) {
            return err;
        }

        ESP_LOGI(TAG, "Initialized config from embedded default JSON");
    }

    return ESP_OK;
}

esp_err_t glow_storage_get_settings(glow_settings_t *out_settings)
{
    if (out_settings == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    *out_settings = s_settings;
    return ESP_OK;
}

esp_err_t glow_storage_set_settings(const glow_settings_t *settings)
{
    if (settings == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    glow_settings_t normalized = *settings;
    if (normalized.led_count > GLOW_SETTINGS_LED_COUNT_MAX) {
        normalized.led_count = GLOW_SETTINGS_LED_COUNT_MAX;
    }
    if (normalized.battery_low_warning_v < GLOW_SETTINGS_BAT_LOW_WARNING_V_MIN) {
        normalized.battery_low_warning_v = GLOW_SETTINGS_BAT_LOW_WARNING_V_MIN;
    }
    if (normalized.battery_low_warning_v > GLOW_SETTINGS_BAT_LOW_WARNING_V_MAX) {
        normalized.battery_low_warning_v = GLOW_SETTINGS_BAT_LOW_WARNING_V_MAX;
    }
    if (normalized.battery_low_cutoff_v < GLOW_SETTINGS_BAT_LOW_CUTOFF_V_MIN) {
        normalized.battery_low_cutoff_v = GLOW_SETTINGS_BAT_LOW_CUTOFF_V_MIN;
    }
    if (normalized.battery_low_cutoff_v > GLOW_SETTINGS_BAT_LOW_CUTOFF_V_MAX) {
        normalized.battery_low_cutoff_v = GLOW_SETTINGS_BAT_LOW_CUTOFF_V_MAX;
    }
    if (normalized.battery_low_cutoff_v > normalized.battery_low_warning_v) {
        normalized.battery_low_cutoff_v = normalized.battery_low_warning_v;
    }
    if (!(normalized.cell_count == GLOW_SETTINGS_CELL_COUNT_AUTO ||
        (normalized.cell_count >= GLOW_SETTINGS_CELL_COUNT_MIN && normalized.cell_count <= GLOW_SETTINGS_CELL_COUNT_MAX))) {
        normalized.cell_count = GLOW_SETTINGS_CELL_COUNT_AUTO;
    }

    glow_device_config_t updated = s_device_config;
    updated.leds_num_leds = normalized.led_count;
    updated.battery_low_voltage = normalized.battery_low_warning_v;
    updated.battery_crit_voltage = normalized.battery_low_cutoff_v;
    updated.battery_ncells = normalized.cell_count;

    char json[GLOW_STORAGE_CONFIG_JSON_MAX_LEN] = {0};
    ESP_RETURN_ON_ERROR(glow_storage_serialize_device_config_json(&updated, json, sizeof(json)), TAG, "Failed to serialize updated settings");
    return glow_storage_set_config_json_internal(json, true);
}

esp_err_t glow_storage_save_settings(void)
{
    return glow_storage_nvs_save_json(s_config_json);
}

esp_err_t glow_storage_reset_settings(void)
{
    char default_json[GLOW_STORAGE_CONFIG_JSON_MAX_LEN] = {0};
    ESP_RETURN_ON_ERROR(glow_storage_get_default_json(default_json, sizeof(default_json)), TAG, "Default config missing/too large");
    return glow_storage_set_config_json_internal(default_json, true);
}

esp_err_t glow_storage_get_config_json(char *out_json, size_t out_size)
{
    if (out_json == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    size_t len = strlen(s_config_json);
    if (out_size <= len) {
        return ESP_ERR_INVALID_SIZE;
    }

    memcpy(out_json, s_config_json, len + 1);
    return ESP_OK;
}

esp_err_t glow_storage_set_config_json(const char *json)
{
    return glow_storage_set_config_json_internal(json, true);
}
