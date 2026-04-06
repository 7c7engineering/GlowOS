#ifndef GLOW_STORAGE_H
#define GLOW_STORAGE_H

#include <stdint.h>
#include <stddef.h>
#include "esp_err.h"

#define GLOW_STORAGE_CONFIG_JSON_MAX_LEN 1024
#define GLOW_STORAGE_GLOW_FUNCTION_MAX_LEN 16
#define GLOW_STORAGE_RECEIVER_TYPE_MAX_LEN 16

typedef struct {
	uint8_t battery_ncells;
	float battery_low_voltage;
	float battery_crit_voltage;
	uint8_t leds_num_leds;
	float leds_brightness;
	char glow_function[GLOW_STORAGE_GLOW_FUNCTION_MAX_LEN];
	uint8_t glow_cut_off_thrs_pct;
	uint8_t glow_max_ppm_pct;
	float glow_max_voltage;
	float glow_voltage;
	float glow_boost;
	float glow_boost_time;
	char receiver_type[GLOW_STORAGE_RECEIVER_TYPE_MAX_LEN];
	uint8_t receiver_channel;
	float receiver_failsafe_timeout;
	/* Deprecated compatibility mirror, equals glow_cut_off_thrs_pct. */
	int receiver_threshold;
} glow_device_config_t;

esp_err_t glow_storage_init(void);
esp_err_t glow_storage_enumerate(void);
esp_err_t glow_storage_get_device_config(glow_device_config_t *out_config);
esp_err_t glow_storage_set_device_config(const glow_device_config_t *config);
esp_err_t glow_storage_save_config(void);
esp_err_t glow_storage_reset_config(void);
esp_err_t glow_storage_get_config_json(char *out_json, size_t out_size);
esp_err_t glow_storage_set_config_json(const char *json);

#endif // GLOW_STORAGE_H
