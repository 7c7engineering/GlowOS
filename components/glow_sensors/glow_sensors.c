#include <stdio.h>
#include "glow_sensors.h"
#include "esp_log.h"
#include "esp_check.h"
#include "driver/temperature_sensor.h"

static const char *TAG = "GLOW_SENSORS";
static temperature_sensor_handle_t s_temp_sensor;

esp_err_t glow_sensors_init(void)
{
    if (s_temp_sensor != NULL) {
        return ESP_OK;
    }

    ESP_LOGI(TAG, "Initializing sensors...");
    temperature_sensor_config_t temp_config = TEMPERATURE_SENSOR_CONFIG_DEFAULT(-40, 125);
    ESP_RETURN_ON_ERROR(temperature_sensor_install(&temp_config, &s_temp_sensor), TAG, "Failed to install temperature sensor");
    ESP_RETURN_ON_ERROR(temperature_sensor_enable(s_temp_sensor), TAG, "Failed to enable temperature sensor");
    ESP_LOGI(TAG, "Temperature sensor initialized");
    return ESP_OK;
}