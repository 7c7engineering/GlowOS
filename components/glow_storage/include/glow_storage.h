#ifndef GLOW_STORAGE_H
#define GLOW_STORAGE_H

#include <stdint.h>
#include <stddef.h>
#include "esp_err.h"

#define GLOW_SETTINGS_CELL_COUNT_AUTO 0

#define GLOW_SETTINGS_LED_COUNT_DEFAULT 1
#define GLOW_SETTINGS_LED_COUNT_MIN 0
#define GLOW_SETTINGS_LED_COUNT_MAX 5

#define GLOW_SETTINGS_BAT_LOW_WARNING_V_DEFAULT 3.2f
#define GLOW_SETTINGS_BAT_LOW_WARNING_V_MIN 2.5f
#define GLOW_SETTINGS_BAT_LOW_WARNING_V_MAX 3.7f

#define GLOW_SETTINGS_BAT_LOW_CUTOFF_V_DEFAULT 3.0f
#define GLOW_SETTINGS_BAT_LOW_CUTOFF_V_MIN 2.5f
#define GLOW_SETTINGS_BAT_LOW_CUTOFF_V_MAX 3.5f

#define GLOW_SETTINGS_CELL_COUNT_MIN 2
#define GLOW_SETTINGS_CELL_COUNT_MAX 5

typedef struct {
	uint8_t led_count;
	float battery_low_warning_v;
	float battery_low_cutoff_v;
	uint8_t cell_count;
} glow_settings_t;

esp_err_t glow_storage_init(void);
esp_err_t glow_storage_enumerate(void);
esp_err_t glow_storage_get_settings(glow_settings_t *out_settings);
esp_err_t glow_storage_set_settings(const glow_settings_t *settings);
esp_err_t glow_storage_save_settings(void);
esp_err_t glow_storage_reset_settings(void);
esp_err_t glow_storage_get_config_json(char *out_json, size_t out_size);
esp_err_t glow_storage_set_config_json(const char *json);

#endif // GLOW_STORAGE_H
