#include <stdio.h>
#include <string.h>
#include "glow_context.h"
#include "glow_led.h"
#include "glow_power.h"
#include "glow_web.h"
#include "glow_storage.h"
#include "esp_check.h"
#include "glow_sensors.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "freertos/semphr.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "glow_core.h"

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
    glow_sensors_init();
    glow_core_init();
    vTaskDelay(5000 / portTICK_PERIOD_MS);
    glow_web_init();
}
