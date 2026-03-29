#include <stdio.h>
#include <string.h>
#include "glow_context.h"
#include "glow_led.h"
#include "glow_power.h"
#include "glow_web.h"
#include "glow_storage.h"
#include "esp_check.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"


static const char *TAG = "Main";

glow_context_t *g_context;

void app_main(void)
{

    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ESP_ERROR_CHECK(nvs_flash_init());
    } else if (err != ESP_OK) {
        ESP_LOGE(TAG, "nvs_flash_init failed: %s", esp_err_to_name(err));
        return;
    }
    esp_event_loop_create_default();
    glow_context_init();
    glow_led_init();
    glow_power_init();
    glow_storage_init();

    vTaskDelay(5000 / portTICK_PERIOD_MS);
    glow_web_init();

    uint32_t vout_mV = 0;
    uint32_t iout_mA = 0;
    uint32_t vbus_mV = 0;
    uint32_t vbat_mV = 0;

    while(1)
    {
        // wait for command in the control queue
        glow_command_t cmd;
        if (xQueueReceive(g_context->control_queue, &cmd, pdMS_TO_TICKS(1000)) == pdTRUE) {
            ESP_LOGI(TAG, "Received command: %d", cmd.command_id);
            // Handle the command (this is where you would add logic to control the hardware based on the command)
            switch (cmd.command_id) {
                case GLOW_CMD_SET_LED_COLOR:
                    // Example: set LED color based on command data
                    glow_led_set_color(cmd.data[0], cmd.data[1], cmd.data[2]);
                    break;
                case GLOW_CMD_SET_VOLTAGE:
                    // Example: set power level based on command data
                    float voltage;
                    memcpy(&voltage, cmd.data, sizeof(float));
                    glow_power_set_voltage(voltage);
                    break;
                case GLOW_CMD_ENABLE_POWER:
                    glow_power_enable(cmd.data[0] != 0);
                    break;
                default:
                    ESP_LOGW(TAG, "Unknown command ID or not yet implemented: %d", cmd.command_id);
                    break;
            }
        }

        // Perform periodic measurements
        glow_power_get_vout_mV(&vout_mV);
        glow_power_get_iout_mA(&iout_mA);
        glow_power_get_vbus_mV(&vbus_mV);
        glow_power_get_vbat_mV(&vbat_mV);
        
        // Add measurements to the queue, overwrite if full
        glow_measurement_t measurement = {
            .vout_mV = vout_mV,
            .iout_mA = iout_mA,
            .vbus_mV = vbus_mV,
            .vbat_mV = vbat_mV,
        };
        xQueueOverwrite(g_context->measurement_queue, &measurement);
        ESP_LOGI(TAG, "Measured Vout: %d mV, Iout: %d mA, VBUS: %d mV, VBAT: %d mV", vout_mV, iout_mA, vbus_mV, vbat_mV);


    }
}
