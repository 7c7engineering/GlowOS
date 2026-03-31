#include <stdio.h>
#include "glow_power.h"

#include "driver/spi_master.h"
#include "driver/gpio.h"

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

const float DAC_VOLTAGE_RECIPROCAL_A = 3.50435568e+02; // Coefficient a for the reciprocal fit
const float DAC_VOLTAGE_RECIPROCAL_B = 9.78695166e+01; // Coefficient b for the reciprocal fit
const float DAC_VOLTAGE_RECIPROCAL_C = 5.77986299e-03; // Coefficient c for the reciprocal fit



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
    return ESP_OK;
}
