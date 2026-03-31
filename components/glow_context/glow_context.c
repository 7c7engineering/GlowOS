#include <stdio.h>
#include <stdlib.h>
#include "glow_context.h"
#include "esp_log.h"

extern glow_context_t *g_context;

static const char *TAG = "glow_context";

esp_err_t glow_context_init(void)
{
    // Create the control queue
    g_context = malloc(sizeof(glow_context_t));
    if (!g_context) {
        ESP_LOGE(TAG, "Failed to allocate memory for context");
        return ESP_ERR_NO_MEM;
    }
    g_context->control_queue = xQueueCreate(10, sizeof(glow_command_t));
    g_context->system_status = xEventGroupCreate();
    g_context->measurement_mutex = xSemaphoreCreateMutex();
    ESP_LOGI(TAG, "GlowDriver context initialized");
    return ESP_OK;
}