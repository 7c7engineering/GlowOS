#pragma once

#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

typedef struct {
    QueueHandle_t control_queue;
    QueueHandle_t measurement_queue;
} glow_context_t;

typedef enum {
    GLOW_CMD_SET_LED_COLOR,
    GLOW_CMD_SET_POWER_LEVEL,
    GLOW_CMD_ENABLE_POWER,
    GLOW_CMD_SET_VOLTAGE,
} glow_command_id_t;

typedef struct{
    uint16_t vbus_mV;
    uint16_t vbat_mV;
    uint16_t vout_mV;
    uint16_t iout_mA;

}glow_measurement_t;

typedef struct {
    glow_command_id_t command_id;
    uint8_t data[16];
} glow_command_t;

esp_err_t glow_context_init(void);
