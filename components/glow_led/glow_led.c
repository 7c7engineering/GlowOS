#include <stdio.h>
#include "glow_led.h"
#include "esp_log.h"
#include "led_strip.h"
#include "esp_check.h"

static const char *TAG = "GLOW_LED";
#define LED_STRIP_RMT_RES_HZ  (10 * 1000 * 1000)
static led_strip_handle_t led_strip;

void glow_led_set_color(uint8_t red, uint8_t green, uint8_t blue) {
    // Set the color of the LED strip here
    led_strip_set_pixel(led_strip, 0, red, green, blue); // Set the first LED
    led_strip_set_pixel(led_strip, 1, red, green, blue); // Set the second LED
    led_strip_refresh(led_strip); // Refresh the strip to apply changes
}

esp_err_t glow_led_init(void) {
    // Initialize the LED hardware here
    ESP_LOGI(TAG, "Initializing GlowDriver LED...");
    // LED strip general initialization, according to your led board design
    led_strip_config_t strip_config = {
        .strip_gpio_num = GPIO_NUM_4, // The GPIO that connected to the LED strip's data line
        .max_leds = 2,      // The number of LEDs in the strip,
        .led_model = LED_MODEL_WS2812,        // LED strip model
        .color_component_format = LED_STRIP_COLOR_COMPONENT_FMT_GRB, // The color order of the strip: GRB
        .flags = {
            .invert_out = false, // don't invert the output signal
        }
    };

    // LED strip backend configuration: RMT
    led_strip_rmt_config_t rmt_config = {
        .clk_src = RMT_CLK_SRC_DEFAULT,        // different clock source can lead to different power consumption
        .resolution_hz = LED_STRIP_RMT_RES_HZ, // RMT counter clock frequency
        .mem_block_symbols = 48, // the memory block size used by the RMT channel
        .flags = {
            .with_dma = 0,     // Using DMA can improve performance when driving more LEDs
        }
    };
    ESP_RETURN_ON_ERROR(led_strip_new_rmt_device(&strip_config, &rmt_config, &led_strip), TAG, "Failed to initialize LED strip");
    return ESP_OK;
}
