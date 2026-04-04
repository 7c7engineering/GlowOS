#include <stdio.h>
#include "glow_sensors.h"
#include "glow_context.h"
#include "esp_log.h"
#include "esp_check.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/temperature_sensor.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "driver/gpio.h"
#include "driver/rmt_rx.h"
#include "esp_timer.h"

static const char *TAG = "GLOW_SENSORS";
static temperature_sensor_handle_t s_temp_sensor;

extern glow_context_t *g_context;

#define GLOW_ADC_UNIT             ADC_UNIT_1
#define GLOW_ADC_CHAN_IOUT        ADC_CHANNEL_0
#define GLOW_ADC_CHAN_VOUT        ADC_CHANNEL_1
#define GLOW_ADC_CHAN_VBUS        ADC_CHANNEL_2
#define GLOW_ADC_CHAN_VBAT        ADC_CHANNEL_3

#define GLOW_ADC_ATTEN            ADC_ATTEN_DB_12
#define GLOW_ADC_BITWIDTH         ADC_BITWIDTH_DEFAULT

static adc_oneshot_unit_handle_t s_adc_handle;
static adc_cali_handle_t s_adc_cali_iout;
static adc_cali_handle_t s_adc_cali_vout;
static adc_cali_handle_t s_adc_cali_vbus;
static adc_cali_handle_t s_adc_cali_vbat;

static bool s_adc_initialized;
static bool s_adc_iout_cali_curve;
static bool s_adc_vout_cali_curve;
static bool s_adc_vbus_cali_curve;
static bool s_adc_vbat_cali_curve;
static bool s_adc_warned_uncalibrated;

const float R_SENSE = 0.05f; // Sense resistor value in ohms
const float I_GAIN = 50.0f; // Current gain of the amplifier
const float V_GAIN = 1.111f; // 2,2Kohm/22Kohm voltage divider gain
const float VBUS_DIVIDER_GAIN = 1.453f; // Voltage divider gain for VBUS and VBAT measurements
const float VBAT_DIVIDER_GAIN = 11.001f; // Voltage divider gain for VBUS and VBAT measurements

const gpio_num_t PIN_RC_PWM = GPIO_NUM_5; // Input for the RC PWM signal.

#define RC_PWM_SIGNAL_TIMEOUT_US  100000
#define RC_PWM_MIN_PULSE_US       1000
#define RC_PWM_MAX_PULSE_US       2000
#define RC_PWM_MIN_PERIOD_US      10000
#define RC_PWM_MAX_PERIOD_US      30000
#define RC_PWM_RMT_RESOLUTION_HZ  1000000
#define RC_PWM_RMT_MEM_SYMBOLS    64
#define RC_PWM_RMT_RX_TIMEOUT_NS  4000000

static portMUX_TYPE s_rc_pwm_lock = portMUX_INITIALIZER_UNLOCKED;
static uint32_t s_rc_pwm_pulse_us;
static uint32_t s_rc_pwm_period_us;
static int64_t s_rc_pwm_last_update_us;
static int64_t s_rc_pwm_prev_update_us;
static rmt_channel_handle_t s_rc_pwm_rmt_chan;
static TaskHandle_t s_rc_pwm_rmt_task_handle;
static volatile size_t s_rc_pwm_rmt_num_symbols;
static rmt_symbol_word_t s_rc_pwm_rmt_symbols[RC_PWM_RMT_MEM_SYMBOLS];


static esp_err_t glow_sensor_adc_create_cali_handle(adc_channel_t channel, adc_cali_handle_t *out_handle, bool *out_is_curve)
{
    adc_cali_scheme_ver_t scheme_mask;
    esp_err_t ret = adc_cali_check_scheme(&scheme_mask);
    if (ret != ESP_OK) {
        return ret;
    }

    #if ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
    if (scheme_mask & ADC_CALI_SCHEME_VER_CURVE_FITTING) {
        adc_cali_curve_fitting_config_t cali_cfg = {
            .unit_id = GLOW_ADC_UNIT,
            .chan = channel,
            .atten = GLOW_ADC_ATTEN,
            .bitwidth = GLOW_ADC_BITWIDTH,
        };
        ESP_RETURN_ON_ERROR(adc_cali_create_scheme_curve_fitting(&cali_cfg, out_handle), TAG, "curve calibration create failed");
        *out_is_curve = true;
        return ESP_OK;
    }
    #else
    (void)channel;
    #endif

   #if ADC_CALI_SCHEME_LINE_FITTING_SUPPORTED
    if (scheme_mask & ADC_CALI_SCHEME_VER_LINE_FITTING) {
        adc_cali_line_fitting_config_t cali_cfg = {
            .unit_id = GLOW_ADC_UNIT,
            .atten = GLOW_ADC_ATTEN,
            .bitwidth = GLOW_ADC_BITWIDTH,
        };
        ESP_RETURN_ON_ERROR(adc_cali_create_scheme_line_fitting(&cali_cfg, out_handle), TAG, "line calibration create failed");
        *out_is_curve = false;
        return ESP_OK;
    }
   #endif

    return ESP_ERR_NOT_SUPPORTED;
}

static bool glow_sensor_rc_pwm_decode_symbol(const rmt_symbol_word_t *symbol, uint32_t *pulse_us)
{
    if (!symbol || !pulse_us) {
        return false;
    }

    uint32_t high_us = 0;
    if (symbol->level0 == 1 && symbol->level1 == 0) {
        high_us = symbol->duration0;
    } else if (symbol->level0 == 0 && symbol->level1 == 1) {
        high_us = symbol->duration1;
    } else {
        return false;
    }

    if (high_us < RC_PWM_MIN_PULSE_US || high_us > RC_PWM_MAX_PULSE_US) {
        return false;
    }

    *pulse_us = high_us;
    return true;
}

static bool IRAM_ATTR glow_sensor_rc_pwm_rmt_rx_done_cb(rmt_channel_handle_t channel, const rmt_rx_done_event_data_t *edata, void *user_data)
{
    (void)channel;
    (void)user_data;
    BaseType_t task_woken = pdFALSE;
    TaskHandle_t task_handle = s_rc_pwm_rmt_task_handle;

    if (edata) {
        s_rc_pwm_rmt_num_symbols = edata->num_symbols;
    }

    if (task_handle) {
        vTaskNotifyGiveFromISR(task_handle, &task_woken);
    }

    return task_woken == pdTRUE;
}

static void vtask_rc_pwm_rmt(void *arg)
{
    (void)arg;

    const rmt_receive_config_t rx_config = {
        .signal_range_min_ns = 1000,
        .signal_range_max_ns = RC_PWM_RMT_RX_TIMEOUT_NS,
    };

    while (1) {
        esp_err_t ret = rmt_receive(s_rc_pwm_rmt_chan, s_rc_pwm_rmt_symbols, sizeof(s_rc_pwm_rmt_symbols), &rx_config);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to start RMT receive: %s", esp_err_to_name(ret));
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }

        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        size_t num_symbols = s_rc_pwm_rmt_num_symbols;
        if (num_symbols > RC_PWM_RMT_MEM_SYMBOLS) {
            num_symbols = RC_PWM_RMT_MEM_SYMBOLS;
        }

        uint32_t pulse_us = 0;
        bool have_valid_measurement = false;

        for (size_t i = 0; i < num_symbols; i++) {
            uint32_t symbol_pulse_us = 0;
            if (glow_sensor_rc_pwm_decode_symbol(&s_rc_pwm_rmt_symbols[i], &symbol_pulse_us)) {
                pulse_us = symbol_pulse_us;
                have_valid_measurement = true;
            }
        }

        if (have_valid_measurement) {
            int64_t now_us = esp_timer_get_time();
            uint32_t measured_period_us = 0;

            if (s_rc_pwm_prev_update_us > 0 && now_us > s_rc_pwm_prev_update_us) {
                uint32_t candidate_period_us = (uint32_t)(now_us - s_rc_pwm_prev_update_us);
                if (candidate_period_us >= RC_PWM_MIN_PERIOD_US && candidate_period_us <= RC_PWM_MAX_PERIOD_US) {
                    measured_period_us = candidate_period_us;
                }
            }

            portENTER_CRITICAL(&s_rc_pwm_lock);
            s_rc_pwm_pulse_us = pulse_us;
            if (measured_period_us > 0) {
                s_rc_pwm_period_us = measured_period_us;
            }
            s_rc_pwm_prev_update_us = now_us;
            s_rc_pwm_last_update_us = now_us;
            portEXIT_CRITICAL(&s_rc_pwm_lock);
        }
    }
}

static esp_err_t glow_sensor_init_rc_pwm(void)
{
    rmt_rx_channel_config_t rx_channel_config = {
        .clk_src = RMT_CLK_SRC_DEFAULT,
        .resolution_hz = RC_PWM_RMT_RESOLUTION_HZ,
        .mem_block_symbols = RC_PWM_RMT_MEM_SYMBOLS,
        .gpio_num = PIN_RC_PWM,
        .flags.with_dma = false,
    };

    ESP_RETURN_ON_ERROR(rmt_new_rx_channel(&rx_channel_config, &s_rc_pwm_rmt_chan), TAG, "Failed to create RMT RX channel");

    rmt_rx_event_callbacks_t callbacks = {
        .on_recv_done = glow_sensor_rc_pwm_rmt_rx_done_cb,
    };
    ESP_RETURN_ON_ERROR(rmt_rx_register_event_callbacks(s_rc_pwm_rmt_chan, &callbacks, NULL), TAG, "Failed to register RMT RX callbacks");

    ESP_RETURN_ON_ERROR(rmt_enable(s_rc_pwm_rmt_chan), TAG, "Failed to enable RMT RX channel");

    BaseType_t task_created = xTaskCreate(vtask_rc_pwm_rmt, "rc_pwm_rmt", 3072, NULL, 6, &s_rc_pwm_rmt_task_handle);
    ESP_RETURN_ON_FALSE(task_created == pdPASS, ESP_ERR_NO_MEM, TAG, "Failed to create RC PWM RMT task");

    ESP_LOGI(TAG, "RC PWM RMT RX initialized on GPIO %d", (int)PIN_RC_PWM);
    return ESP_OK;
}

static void glow_sensor_get_rc_timing(uint32_t *pulse_width_us, uint32_t *period_us, bool *signal_valid)
{
    int64_t now_us = esp_timer_get_time();
    uint32_t pulse_us = 0;
    uint32_t period_local_us = 0;
    int64_t last_update_us = 0;

    portENTER_CRITICAL(&s_rc_pwm_lock);
    pulse_us = s_rc_pwm_pulse_us;
    period_local_us = s_rc_pwm_period_us;
    last_update_us = s_rc_pwm_last_update_us;
    portEXIT_CRITICAL(&s_rc_pwm_lock);

    *pulse_width_us = pulse_us;
    *period_us = period_local_us;
    *signal_valid = (last_update_us > 0) && ((now_us - last_update_us) <= RC_PWM_SIGNAL_TIMEOUT_US);
}

static void glow_sensor_adc_delete_cali_handle(adc_cali_handle_t handle, bool is_curve)
{
    if (!handle) {
        return;
    }

#if ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
    if (is_curve) {
        adc_cali_delete_scheme_curve_fitting(handle);
        return;
    }
#endif

#if ADC_CALI_SCHEME_LINE_FITTING_SUPPORTED
    if (!is_curve) {
        adc_cali_delete_scheme_line_fitting(handle);
        return;
    }
#endif

    (void)is_curve;
}

static esp_err_t glow_sensor_adc_read_mv(adc_channel_t channel, adc_cali_handle_t cali_handle, uint32_t *value_mv)
{
    ESP_RETURN_ON_FALSE(value_mv != NULL, ESP_ERR_INVALID_ARG, TAG, "value pointer is null");
    ESP_RETURN_ON_FALSE(s_adc_initialized, ESP_ERR_INVALID_STATE, TAG, "ADC not initialized");

    int measured_mv = 0;
    if (cali_handle) {
        ESP_RETURN_ON_ERROR(adc_oneshot_get_calibrated_result(s_adc_handle, cali_handle, channel, &measured_mv), TAG, "ADC calibrated read failed");
        *value_mv = (uint32_t)measured_mv;
        return ESP_OK;
    }

    int raw = 0;
    ESP_RETURN_ON_ERROR(adc_oneshot_read(s_adc_handle, channel, &raw), TAG, "ADC raw read failed");

    if (!s_adc_warned_uncalibrated) {
        ESP_LOGW(TAG, "ADC calibration not available, using approximate raw-to-mV conversion");
        s_adc_warned_uncalibrated = true;
    }

    // Approximate conversion for 12-bit ADC when calibration data is unavailable.
    *value_mv = (uint32_t)((raw * 3300) / 4095);
    return ESP_OK;
}


esp_err_t glow_sensor_init_adc(void)
{
    if (s_adc_initialized) {
        return ESP_OK;
    }

    adc_oneshot_unit_init_cfg_t unit_cfg = {
        .unit_id = GLOW_ADC_UNIT,
        .ulp_mode = ADC_ULP_MODE_DISABLE,
    };
    ESP_RETURN_ON_ERROR(adc_oneshot_new_unit(&unit_cfg, &s_adc_handle), TAG, "Failed to create ADC unit");

    adc_oneshot_chan_cfg_t chan_cfg = {
        .atten = GLOW_ADC_ATTEN,
        .bitwidth = GLOW_ADC_BITWIDTH,
    };

    esp_err_t ret = adc_oneshot_config_channel(s_adc_handle, GLOW_ADC_CHAN_IOUT, &chan_cfg);
    if (ret != ESP_OK) {
        adc_oneshot_del_unit(s_adc_handle);
        s_adc_handle = NULL;
        return ret;
    }

    ret = adc_oneshot_config_channel(s_adc_handle, GLOW_ADC_CHAN_VOUT, &chan_cfg);
    if (ret != ESP_OK) {
        adc_oneshot_del_unit(s_adc_handle);
        s_adc_handle = NULL;
        return ret;
    }

    ret = adc_oneshot_config_channel(s_adc_handle, GLOW_ADC_CHAN_VBUS, &chan_cfg);
    if (ret != ESP_OK) {
        adc_oneshot_del_unit(s_adc_handle);
        s_adc_handle = NULL;
        return ret;
    }

    ret = adc_oneshot_config_channel(s_adc_handle, GLOW_ADC_CHAN_VBAT, &chan_cfg);
    if (ret != ESP_OK) {
        adc_oneshot_del_unit(s_adc_handle);
        s_adc_handle = NULL;
        return ret;
    }

    ret = glow_sensor_adc_create_cali_handle(GLOW_ADC_CHAN_IOUT, &s_adc_cali_iout, &s_adc_iout_cali_curve);
    if (ret != ESP_OK && ret != ESP_ERR_NOT_SUPPORTED) {
        adc_oneshot_del_unit(s_adc_handle);
        s_adc_handle = NULL;
        return ret;
    }

    ret = glow_sensor_adc_create_cali_handle(GLOW_ADC_CHAN_VOUT, &s_adc_cali_vout, &s_adc_vout_cali_curve);
    if (ret != ESP_OK && ret != ESP_ERR_NOT_SUPPORTED) {
        glow_sensor_adc_delete_cali_handle(s_adc_cali_iout, s_adc_iout_cali_curve);
        s_adc_cali_iout = NULL;
        adc_oneshot_del_unit(s_adc_handle);
        s_adc_handle = NULL;
        return ret;
    }

    ret = glow_sensor_adc_create_cali_handle(GLOW_ADC_CHAN_VBUS, &s_adc_cali_vbus, &s_adc_vbus_cali_curve);
    if (ret != ESP_OK && ret != ESP_ERR_NOT_SUPPORTED) {
        glow_sensor_adc_delete_cali_handle(s_adc_cali_iout, s_adc_iout_cali_curve);
        glow_sensor_adc_delete_cali_handle(s_adc_cali_vout, s_adc_vout_cali_curve);
        s_adc_cali_iout = NULL;
        s_adc_cali_vout = NULL;
        adc_oneshot_del_unit(s_adc_handle);
        s_adc_handle = NULL;
        return ret;
    }

    ret = glow_sensor_adc_create_cali_handle(GLOW_ADC_CHAN_VBAT, &s_adc_cali_vbat, &s_adc_vbat_cali_curve);
    if (ret != ESP_OK && ret != ESP_ERR_NOT_SUPPORTED) {
        glow_sensor_adc_delete_cali_handle(s_adc_cali_iout, s_adc_iout_cali_curve);
        glow_sensor_adc_delete_cali_handle(s_adc_cali_vout, s_adc_vout_cali_curve);
        glow_sensor_adc_delete_cali_handle(s_adc_cali_vbus, s_adc_vbus_cali_curve);
        s_adc_cali_iout = NULL;
        s_adc_cali_vout = NULL;
        s_adc_cali_vbus = NULL;
        adc_oneshot_del_unit(s_adc_handle);
        s_adc_handle = NULL;
        return ret;
    }

    s_adc_initialized = true;
    return ESP_OK;
}

esp_err_t glow_sensor_get_temperature_degC(float *temp_degC)
{
    if (!s_temp_sensor || !temp_degC) {
        return ESP_ERR_INVALID_ARG;
    }
    return temperature_sensor_get_celsius(s_temp_sensor, temp_degC);
}

esp_err_t glow_sensor_get_vout_mV(uint32_t *vout_mV)
{
    uint32_t vout_mv = 0;
    esp_err_t ret = glow_sensor_adc_read_mv(GLOW_ADC_CHAN_VOUT, s_adc_cali_vout, &vout_mv);
    if (ret != ESP_OK) {
        return ret;
    }
    *vout_mV = (uint32_t)(vout_mv * V_GAIN);
    return ESP_OK;
}

esp_err_t glow_sensor_get_iout_mA(uint32_t *iout_mA)
{
    uint32_t iout_mv = 0;
    esp_err_t ret = glow_sensor_adc_read_mv(GLOW_ADC_CHAN_IOUT, s_adc_cali_iout, &iout_mv);
    if (ret != ESP_OK) {
        return ret;
    }
    *iout_mA = (uint32_t)((iout_mv / R_SENSE) / I_GAIN);
    return ESP_OK;
}

esp_err_t glow_sensor_get_vbus_mV(uint32_t *vbus_mV)
{
    uint32_t vbus_mv = 0;
    esp_err_t ret = glow_sensor_adc_read_mv(GLOW_ADC_CHAN_VBUS, s_adc_cali_vbus, &vbus_mv);
    if (ret != ESP_OK) {
        return ret;
    }
    *vbus_mV = (uint32_t)(vbus_mv * VBUS_DIVIDER_GAIN);
    return ESP_OK;
}

esp_err_t glow_sensor_get_vbat_mV(uint32_t *vbat_mV)
{
    uint32_t vbat_mv = 0;
    esp_err_t ret = glow_sensor_adc_read_mv(GLOW_ADC_CHAN_VBAT, s_adc_cali_vbat, &vbat_mv);
    if (ret != ESP_OK) {
        return ret;
    }
    *vbat_mV = (uint32_t)(vbat_mv * VBAT_DIVIDER_GAIN);
    return ESP_OK;
}

esp_err_t glow_sensor_get_rc_pwm(uint32_t *percentage, bool *signal_valid)
{
    uint32_t pulse_width_us = 0;
    uint32_t period_us = 0;
    glow_sensor_get_rc_timing(&pulse_width_us, &period_us, signal_valid);
    *percentage = (pulse_width_us - RC_PWM_MIN_PULSE_US) * 100 / (RC_PWM_MAX_PULSE_US - RC_PWM_MIN_PULSE_US);
    //ESP_LOGI(TAG, "RC PWM measurement: pulse=%u us, period=%u us, valid=%s", (unsigned)pulse_width_us, (unsigned)period_us, *signal_valid ? "true" : "false");
    return ESP_OK;
}


void vtask_sensor_gathering(void *arg)
{
    (void)arg;
    while (1) {
        if (s_temp_sensor) {
            float temp_degC = 0;
            uint32_t vout_mV = 0, iout_mA = 0, vbus_mV = 0, vbat_mV = 0;
            uint32_t rc_pwm_pulse_us = 0, rc_pwm_period_us = 0;
            bool rc_pwm_signal_valid = false;
            esp_err_t err = temperature_sensor_get_celsius(s_temp_sensor, &temp_degC);
            if (err == ESP_OK) {
                err = glow_sensor_get_vout_mV(&vout_mV);
            }
            if (err == ESP_OK) {
                err = glow_sensor_get_iout_mA(&iout_mA);
            }
            if(err == ESP_OK) {
                err = glow_sensor_get_vbus_mV(&vbus_mV);
            }
            if(err == ESP_OK) {
                err = glow_sensor_get_vbat_mV(&vbat_mV);
            }

            glow_sensor_get_rc_timing(&rc_pwm_pulse_us, &rc_pwm_period_us, &rc_pwm_signal_valid);
            // signal min = 150 us, max = 1100 us
            //ESP_LOGI(TAG, "RC PWM measurement: pulse=%u us, period=%u us, valid=%s", (unsigned)rc_pwm_pulse_us, (unsigned)rc_pwm_period_us, rc_pwm_signal_valid ? "true" : "false");
            uint32_t rc_pwm_percentage = 0;
            if (rc_pwm_signal_valid) {
                if (rc_pwm_pulse_us < RC_PWM_MIN_PULSE_US) {
                    rc_pwm_percentage = 0;
                } else if (rc_pwm_pulse_us > RC_PWM_MAX_PULSE_US) {
                    rc_pwm_percentage = 100;
                } else {
                    rc_pwm_percentage = (rc_pwm_pulse_us - RC_PWM_MIN_PULSE_US) * 100 / (RC_PWM_MAX_PULSE_US - RC_PWM_MIN_PULSE_US);
                }
            }
            if (err == ESP_OK) {
                if (g_context) {
                    xSemaphoreTake(g_context->measurement_mutex, portMAX_DELAY);
                    g_context->latest_measurement.temperature_degC = temp_degC;
                    g_context->latest_measurement.vout_mV = vout_mV;
                    g_context->latest_measurement.iout_mA = iout_mA;
                    g_context->latest_measurement.vbus_mV = vbus_mV;
                    g_context->latest_measurement.vbat_mV = vbat_mV;
                    g_context->latest_measurement.rc_pwm_percentage = rc_pwm_percentage;
                    g_context->latest_measurement.rc_pwm_signal_valid = rc_pwm_signal_valid ? 1U : 0U;
                    xSemaphoreGive(g_context->measurement_mutex);
                }
            } else {
                ESP_LOGE(TAG, "Failed to read temperature: %s", esp_err_to_name(err));
            }
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

esp_err_t glow_sensors_init(void)
{
    if (s_temp_sensor != NULL) {
        return ESP_OK;
    }

    ESP_LOGI(TAG, "Initializing sensors...");
    temperature_sensor_config_t temp_config = TEMPERATURE_SENSOR_CONFIG_DEFAULT(0, 70);
    ESP_RETURN_ON_ERROR(temperature_sensor_install(&temp_config, &s_temp_sensor), TAG, "Failed to install temperature sensor");
    ESP_RETURN_ON_ERROR(temperature_sensor_enable(s_temp_sensor), TAG, "Failed to enable temperature sensor");
    ESP_RETURN_ON_ERROR(glow_sensor_init_adc(), TAG, "Failed to initialize ADC for sensors");
    ESP_RETURN_ON_ERROR(glow_sensor_init_rc_pwm(), TAG, "Failed to initialize RC PWM sensor input");
    ESP_LOGI(TAG, "Temperature sensor initialized");
    //xTaskCreate(vtask_sensor_gathering, "sensor_gathering", 2048, NULL, 5, NULL);
    return ESP_OK;
}