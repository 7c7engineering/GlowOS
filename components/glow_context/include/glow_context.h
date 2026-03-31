#pragma once

#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/event_groups.h"
#include "freertos/semphr.h"

typedef struct {
    uint32_t vout_mV;
    uint32_t iout_mA;
    uint32_t vbus_mV;
    uint32_t vbat_mV;
    float temperature_degC;
} glow_measurement_t;

typedef struct {
    QueueHandle_t control_queue;
    EventGroupHandle_t system_status;
    SemaphoreHandle_t measurement_mutex;
    glow_measurement_t latest_measurement;
} glow_context_t;

const static EventBits_t GLOW_POWER_ENABLED_BIT = BIT0;
const static EventBits_t GLOW_MANUAL_MODE_BIT = BIT1;
const static EventBits_t GLOW_BATTERY_LOW_BIT = BIT2;
const static EventBits_t GLOW_BATTERY_CRITICAL_BIT = BIT3;



typedef enum {
    GLOW_CMD_SET_LED_COLOR,
    GLOW_CMD_SET_POWER_LEVEL,
    GLOW_CMD_ENABLE_POWER,
    GLOW_CMD_SET_VOLTAGE,
} glow_command_id_t;

typedef struct {
    glow_command_id_t command_id;
    uint8_t data[16];
} glow_command_t;

esp_err_t glow_context_init(void);
