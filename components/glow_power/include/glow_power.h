#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"

esp_err_t glow_power_enable(bool enable);
esp_err_t glow_power_set(uint8_t value);
esp_err_t glow_power_set_voltage(float voltage);
esp_err_t glow_power_init_adc(void);
esp_err_t glow_power_get_vout_mV(uint32_t *vout_mv);
esp_err_t glow_power_get_iout_mA(uint32_t *iout_mA);
esp_err_t glow_power_get_vbus_mV(uint32_t *vbus_mV);
esp_err_t glow_power_get_vbat_mV(uint32_t *vbat_mV);
esp_err_t glow_power_init(void);
