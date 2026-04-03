#pragma once

#include "esp_err.h"

esp_err_t glow_sensors_init(void);
esp_err_t glow_sensor_get_vbat_mV(uint32_t *vbat_mV);
esp_err_t glow_sensor_get_vbus_mV(uint32_t *vbus_mV);
esp_err_t glow_sensor_get_vout_mV(uint32_t *vout_mV);
esp_err_t glow_sensor_get_iout_mA(uint32_t *iout_mA);
esp_err_t glow_sensor_get_temperature_degC(float *temp_degC);
esp_err_t glow_sensor_get_rc_pwm(uint32_t *percentage, bool *signal_valid);