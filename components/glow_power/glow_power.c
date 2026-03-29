#include <stdio.h>
#include "glow_power.h"

#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "esp_log.h"
#include "esp_check.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "GLOW_POWER";

static spi_device_handle_t spi_handle;
gpio_num_t POWER_ENABLE_PIN = GPIO_NUM_8; // Example GPIO pin for power enable
gpio_num_t DAC_CS_PIN = GPIO_NUM_10; // Example GPIO pin for DAC CS
gpio_num_t DAC_MOSI_PIN = GPIO_NUM_7; // Example GPIO pin for DAC MOSI
gpio_num_t DAC_SCLK_PIN = GPIO_NUM_6; // Example GPIO pin for DAC SCLK

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

const float R_SENSE = 0.1f; // Sense resistor value in ohms
const float I_GAIN = 50.0f; // Current gain of the amplifier
const float V_GAIN = 1.111f; // 2,2Kohm/22Kohm voltage divider gain
const float VBUS_DIVIDER_GAIN = 1.453f; // Voltage divider gain for VBUS and VBAT measurements
const float VBAT_DIVIDER_GAIN = 11.001f; // Voltage divider gain for VBUS and VBAT measurements

const float DAC_VOLTAGE_RECIPROCAL_A = 3.50435568e+02; // Coefficient a for the reciprocal fit
const float DAC_VOLTAGE_RECIPROCAL_B = 9.78695166e+01; // Coefficient b for the reciprocal fit
const float DAC_VOLTAGE_RECIPROCAL_C = 5.77986299e-03; // Coefficient c for the reciprocal fit

static esp_err_t glow_power_adc_create_cali_handle(adc_channel_t channel, adc_cali_handle_t *out_handle, bool *out_is_curve)
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

static void glow_power_adc_delete_cali_handle(adc_cali_handle_t handle, bool is_curve)
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

static esp_err_t glow_power_adc_read_mv(adc_channel_t channel, adc_cali_handle_t cali_handle, uint32_t *value_mv)
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


esp_err_t glow_power_enable(bool enable)
{
    gpio_set_level(POWER_ENABLE_PIN, enable ? 1 : 0);
    return ESP_OK;   
}


esp_err_t glow_power_set_voltage(float voltage)
{
    float dac_value_f = (DAC_VOLTAGE_RECIPROCAL_A / (voltage - DAC_VOLTAGE_RECIPROCAL_C)) - DAC_VOLTAGE_RECIPROCAL_B;
    if(dac_value_f < 0) {
        dac_value_f = 0;
    } else if (dac_value_f > 255) {
        dac_value_f = 255;
    }
    uint8_t dac_value = (uint8_t)dac_value_f;
    return glow_power_set(dac_value);
}

esp_err_t glow_power_set(uint8_t value)
{
    spi_transaction_t t = {
        .length = 8, // 8 bits
        .tx_buffer = &value,
    };
    gpio_set_level(DAC_CS_PIN, 0); // Assert CS (active low)
    esp_err_t ret = spi_device_transmit(spi_handle, &t);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to transmit SPI data: %s", esp_err_to_name(ret));
        gpio_set_level(DAC_CS_PIN, 1); // Deassert CS (inactive high)
        return ret;
    }
    // wait for the transmission to complete before deasserting CS
    vTaskDelay(pdMS_TO_TICKS(1));
    gpio_set_level(DAC_CS_PIN, 1); // Deassert CS (inactive high)
    return ESP_OK;
}

esp_err_t glow_power_init_adc(void)
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

    ret = glow_power_adc_create_cali_handle(GLOW_ADC_CHAN_IOUT, &s_adc_cali_iout, &s_adc_iout_cali_curve);
    if (ret != ESP_OK && ret != ESP_ERR_NOT_SUPPORTED) {
        adc_oneshot_del_unit(s_adc_handle);
        s_adc_handle = NULL;
        return ret;
    }

    ret = glow_power_adc_create_cali_handle(GLOW_ADC_CHAN_VOUT, &s_adc_cali_vout, &s_adc_vout_cali_curve);
    if (ret != ESP_OK && ret != ESP_ERR_NOT_SUPPORTED) {
        glow_power_adc_delete_cali_handle(s_adc_cali_iout, s_adc_iout_cali_curve);
        s_adc_cali_iout = NULL;
        adc_oneshot_del_unit(s_adc_handle);
        s_adc_handle = NULL;
        return ret;
    }

    ret = glow_power_adc_create_cali_handle(GLOW_ADC_CHAN_VBUS, &s_adc_cali_vbus, &s_adc_vbus_cali_curve);
    if (ret != ESP_OK && ret != ESP_ERR_NOT_SUPPORTED) {
        glow_power_adc_delete_cali_handle(s_adc_cali_iout, s_adc_iout_cali_curve);
        glow_power_adc_delete_cali_handle(s_adc_cali_vout, s_adc_vout_cali_curve);
        s_adc_cali_iout = NULL;
        s_adc_cali_vout = NULL;
        adc_oneshot_del_unit(s_adc_handle);
        s_adc_handle = NULL;
        return ret;
    }

    ret = glow_power_adc_create_cali_handle(GLOW_ADC_CHAN_VBAT, &s_adc_cali_vbat, &s_adc_vbat_cali_curve);
    if (ret != ESP_OK && ret != ESP_ERR_NOT_SUPPORTED) {
        glow_power_adc_delete_cali_handle(s_adc_cali_iout, s_adc_iout_cali_curve);
        glow_power_adc_delete_cali_handle(s_adc_cali_vout, s_adc_vout_cali_curve);
        glow_power_adc_delete_cali_handle(s_adc_cali_vbus, s_adc_vbus_cali_curve);
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

esp_err_t glow_power_get_vout_mV(uint32_t *vout_mV)
{
    uint32_t vout_mv = 0;
    esp_err_t ret = glow_power_adc_read_mv(GLOW_ADC_CHAN_VOUT, s_adc_cali_vout, &vout_mv);
    if (ret != ESP_OK) {
        return ret;
    }
    *vout_mV = (uint32_t)(vout_mv * V_GAIN);
    return ESP_OK;
}

esp_err_t glow_power_get_iout_mA(uint32_t *iout_mA)
{
    uint32_t iout_mv = 0;
    esp_err_t ret = glow_power_adc_read_mv(GLOW_ADC_CHAN_IOUT, s_adc_cali_iout, &iout_mv);
    if (ret != ESP_OK) {
        return ret;
    }
    *iout_mA = (uint32_t)((iout_mv / R_SENSE) / I_GAIN);
    return ESP_OK;
}

esp_err_t glow_power_get_vbus_mV(uint32_t *vbus_mV)
{
    uint32_t vbus_mv = 0;
    esp_err_t ret = glow_power_adc_read_mv(GLOW_ADC_CHAN_VBUS, s_adc_cali_vbus, &vbus_mv);
    if (ret != ESP_OK) {
        return ret;
    }
    *vbus_mV = (uint32_t)(vbus_mv * VBUS_DIVIDER_GAIN);
    return ESP_OK;
}

esp_err_t glow_power_get_vbat_mV(uint32_t *vbat_mV)
{
    uint32_t vbat_mv = 0;
    esp_err_t ret = glow_power_adc_read_mv(GLOW_ADC_CHAN_VBAT, s_adc_cali_vbat, &vbat_mv);
    if (ret != ESP_OK) {
        return ret;
    }
    *vbat_mV = (uint32_t)(vbat_mv * VBAT_DIVIDER_GAIN);
    return ESP_OK;
}

esp_err_t glow_power_init(void)
{
    // Configure the power enable pin
    gpio_config_t io_conf = {
        .intr_type = GPIO_INTR_DISABLE,
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = (1ULL << POWER_ENABLE_PIN),
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .pull_up_en = GPIO_PULLUP_DISABLE,
    };
    ESP_RETURN_ON_ERROR(gpio_config(&io_conf), TAG, "Failed to configure GPIO");
    gpio_set_level(POWER_ENABLE_PIN, 0); // Start with power disabled
    
    // configure CS pin as output and set it high (inactive)
    io_conf.pin_bit_mask = (1ULL << DAC_CS_PIN); // Example GPIO pin for CS
    ESP_RETURN_ON_ERROR(gpio_config(&io_conf), TAG, "Failed to configure GPIO");
    gpio_set_level(DAC_CS_PIN, 1); // Set CS high (inactive)

    // SPI bus configuration
    spi_bus_config_t buscfg = {
        .miso_io_num = -1, // Not used
        .mosi_io_num = DAC_MOSI_PIN, // MOSI pin
        .sclk_io_num = DAC_SCLK_PIN, // SCLK pin
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 1, // We only need to send 1 byte
    };
    // SPI device configuration
    spi_device_interface_config_t devcfg = {
        .clock_speed_hz = 100000, // 100 kHz
        .mode = 1, // SPI mode 0
        .spics_io_num = -1, // CS pin
        .queue_size = 1
    };

    ESP_RETURN_ON_ERROR(spi_bus_initialize(SPI2_HOST, &buscfg, SPI_DMA_CH_AUTO), TAG, "Failed to initialize SPI bus");
    ESP_RETURN_ON_ERROR(spi_bus_add_device(SPI2_HOST, &devcfg, &spi_handle), TAG, "Failed to add SPI device");
    ESP_RETURN_ON_ERROR(glow_power_init_adc(), TAG, "Failed to initialize ADC");

    return ESP_OK;
}
