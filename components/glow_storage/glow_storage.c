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

#include "glow_storage.h"

static const char *TAG = "GLOW_STORAGE";

#define GLOW_STORAGE_PARTITION_LABEL "fat"
#define GLOW_STORAGE_MOUNT_POINT "/storage"
#define GLOW_STORAGE_SETTINGS_PATH GLOW_STORAGE_MOUNT_POINT "/settings.json"

static wl_handle_t s_wl_handle = WL_INVALID_HANDLE;
static bool s_mounted;
static glow_settings_t s_settings;

static float glow_storage_clampf(float value, float min_value, float max_value)
{
    if (value < min_value) {
        return min_value;
    }
    if (value > max_value) {
        return max_value;
    }
    return value;
}

static uint8_t glow_storage_clamp_u8(uint8_t value, uint8_t min_value, uint8_t max_value)
{
    if (value < min_value) {
        return min_value;
    }
    if (value > max_value) {
        return max_value;
    }
    return value;
}

static void glow_storage_set_defaults(glow_settings_t *settings)
{
    settings->led_count = GLOW_SETTINGS_LED_COUNT_DEFAULT;
    settings->battery_low_warning_v = GLOW_SETTINGS_BAT_LOW_WARNING_V_DEFAULT;
    settings->battery_low_cutoff_v = GLOW_SETTINGS_BAT_LOW_CUTOFF_V_DEFAULT;
    settings->cell_count = GLOW_SETTINGS_CELL_COUNT_AUTO;
}

static void glow_storage_normalize_settings(glow_settings_t *settings)
{
    settings->led_count = glow_storage_clamp_u8(
        settings->led_count,
        GLOW_SETTINGS_LED_COUNT_MIN,
        GLOW_SETTINGS_LED_COUNT_MAX
    );

    settings->battery_low_warning_v = glow_storage_clampf(
        settings->battery_low_warning_v,
        GLOW_SETTINGS_BAT_LOW_WARNING_V_MIN,
        GLOW_SETTINGS_BAT_LOW_WARNING_V_MAX
    );

    settings->battery_low_cutoff_v = glow_storage_clampf(
        settings->battery_low_cutoff_v,
        GLOW_SETTINGS_BAT_LOW_CUTOFF_V_MIN,
        GLOW_SETTINGS_BAT_LOW_CUTOFF_V_MAX
    );

    if (settings->battery_low_cutoff_v > settings->battery_low_warning_v) {
        settings->battery_low_cutoff_v = settings->battery_low_warning_v;
    }

    if (settings->cell_count != GLOW_SETTINGS_CELL_COUNT_AUTO) {
        settings->cell_count = glow_storage_clamp_u8(
            settings->cell_count,
            GLOW_SETTINGS_CELL_COUNT_MIN,
            GLOW_SETTINGS_CELL_COUNT_MAX
        );
    }
}

static bool glow_storage_parse_int_field(const char *json, const char *key, int *out_value)
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

    errno = 0;
    char *endptr = NULL;
    long parsed = strtol(colon + 1, &endptr, 10);
    if (colon + 1 == endptr || errno != 0) {
        return false;
    }

    *out_value = (int)parsed;
    return true;
}

static bool glow_storage_parse_float_field(const char *json, const char *key, float *out_value)
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

    errno = 0;
    char *endptr = NULL;
    float parsed = strtof(colon + 1, &endptr);
    if (colon + 1 == endptr || errno != 0) {
        return false;
    }

    *out_value = parsed;
    return true;
}

static bool glow_storage_parse_cell_count(const char *json, uint8_t *out_cell_count)
{
    char pattern[64];
    snprintf(pattern, sizeof(pattern), "\"%s\"", "cell_count");

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

    if (strncmp(value_start, "\"auto\"", 6) == 0) {
        *out_cell_count = GLOW_SETTINGS_CELL_COUNT_AUTO;
        return true;
    }

    errno = 0;
    char *endptr = NULL;
    long parsed = strtol(value_start, &endptr, 10);
    if (value_start == endptr || errno != 0) {
        return false;
    }

    *out_cell_count = (uint8_t)parsed;
    return true;
}

static esp_err_t glow_storage_save_settings_internal(const glow_settings_t *settings)
{
    if (!s_mounted) {
        return ESP_ERR_INVALID_STATE;
    }

    FILE *f = fopen(GLOW_STORAGE_SETTINGS_PATH, "wb");
    if (f == NULL) {
        ESP_LOGE(TAG, "Failed to open %s for write", GLOW_STORAGE_SETTINGS_PATH);
        return ESP_FAIL;
    }

    int written;
    if (settings->cell_count == GLOW_SETTINGS_CELL_COUNT_AUTO) {
        written = fprintf(
            f,
            "{\n"
            "  \"led_count\": %u,\n"
            "  \"battery_low_warning_v\": %.3f,\n"
            "  \"battery_low_cutoff_v\": %.3f,\n"
            "  \"cell_count\": \"auto\"\n"
            "}\n",
            (unsigned)settings->led_count,
            (double)settings->battery_low_warning_v,
            (double)settings->battery_low_cutoff_v
        );
    } else {
        written = fprintf(
            f,
            "{\n"
            "  \"led_count\": %u,\n"
            "  \"battery_low_warning_v\": %.3f,\n"
            "  \"battery_low_cutoff_v\": %.3f,\n"
            "  \"cell_count\": %u\n"
            "}\n",
            (unsigned)settings->led_count,
            (double)settings->battery_low_warning_v,
            (double)settings->battery_low_cutoff_v,
            (unsigned)settings->cell_count
        );
    }

    fclose(f);

    if (written <= 0) {
        ESP_LOGE(TAG, "Failed to write %s", GLOW_STORAGE_SETTINGS_PATH);
        return ESP_FAIL;
    }

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

static esp_err_t glow_storage_read_settings_json(void)
{
    FILE *f = fopen(GLOW_STORAGE_SETTINGS_PATH, "rb");
    if (f == NULL) {
        ESP_LOGW(TAG, "settings.json not found at %s", GLOW_STORAGE_SETTINGS_PATH);
        return ESP_ERR_NOT_FOUND;
    }

    char content[1024] = {0};
    size_t bytes_read = fread(content, 1, sizeof(content) - 1, f);
    fclose(f);
    if (bytes_read == 0) {
        ESP_LOGW(TAG, "settings.json is empty, using defaults");
        return ESP_OK;
    }

    glow_settings_t parsed = s_settings;

    int led_count = 0;
    if (glow_storage_parse_int_field(content, "led_count", &led_count)) {
        parsed.led_count = (uint8_t)led_count;
    }

    float low_warning = 0.0f;
    if (glow_storage_parse_float_field(content, "battery_low_warning_v", &low_warning)) {
        parsed.battery_low_warning_v = low_warning;
    }

    float low_cutoff = 0.0f;
    if (glow_storage_parse_float_field(content, "battery_low_cutoff_v", &low_cutoff)) {
        parsed.battery_low_cutoff_v = low_cutoff;
    }

    uint8_t cell_count = 0;
    if (glow_storage_parse_cell_count(content, &cell_count)) {
        parsed.cell_count = cell_count;
    }

    glow_storage_normalize_settings(&parsed);
    s_settings = parsed;

    ESP_LOGI(TAG, "Loaded settings from %s", GLOW_STORAGE_SETTINGS_PATH);
    return ESP_OK;
}

esp_err_t glow_storage_init(void)
{
    glow_storage_set_defaults(&s_settings);

    ESP_RETURN_ON_ERROR(glow_storage_mount_fat_partition(), TAG, "FAT mount failed");

    esp_err_t settings_err = glow_storage_read_settings_json();
    if (settings_err == ESP_ERR_NOT_FOUND) {
        ESP_RETURN_ON_ERROR(glow_storage_save_settings_internal(&s_settings), TAG, "Failed to create default settings.json");
        ESP_LOGI(TAG, "Created default settings at %s", GLOW_STORAGE_SETTINGS_PATH);
        return ESP_OK;
    }

    ESP_RETURN_ON_ERROR(settings_err, TAG, "Failed to read settings.json");

    ESP_LOGI(
        TAG,
        "Active settings: led_count=%u warning=%.2fV cutoff=%.2fV cells=%s",
        (unsigned)s_settings.led_count,
        (double)s_settings.battery_low_warning_v,
        (double)s_settings.battery_low_cutoff_v,
        s_settings.cell_count == GLOW_SETTINGS_CELL_COUNT_AUTO ? "auto" : "manual"
    );

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
    glow_storage_normalize_settings(&normalized);
    s_settings = normalized;

    return ESP_OK;
}

esp_err_t glow_storage_save_settings(void)
{
    return glow_storage_save_settings_internal(&s_settings);
}

esp_err_t glow_storage_reset_settings(void)
{
    glow_storage_set_defaults(&s_settings);
    return glow_storage_save_settings_internal(&s_settings);
}
