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
    uint32_t rc_pwm_percentage;
    uint32_t rc_pwm_signal_valid;
    float temperature_degC;
} glow_measurement_t;

// States of the statemachine
typedef enum {
    GLOW_STATE_OFF,
    GLOW_STATE_BOOST,
    GLOW_STATE_GLOWING,
    GLOW_STATE_MANUAL,
    GLOW_STATE_LOW_BATTERY,
    GLOW_STATE_BROKEN_WIRE,
    GLOW_STATE_RC_LOST,
    GLOW_STATE_ERROR,
} glow_system_state_t;

typedef struct {
    QueueHandle_t control_queue;
    EventGroupHandle_t system_status;
    SemaphoreHandle_t measurement_mutex;
    glow_measurement_t latest_measurement;
    glow_system_state_t system_state;
} glow_context_t;



const static EventBits_t GLOW_POWER_ENABLED_BIT = BIT0;
const static EventBits_t GLOW_MANUAL_MODE_BIT = BIT1;
const static EventBits_t GLOW_BATTERY_LOW_BIT = BIT2;
const static EventBits_t GLOW_BATTERY_CRITICAL_BIT = BIT3;
const static EventBits_t GLOW_RELOAD_CONFIG_BIT = BIT4;



typedef enum {
    GLOW_CMD_SET_LED_COLOR,
    GLOW_CMD_SET_POWER_LEVEL,
    GLOW_CMD_ENABLE_POWER,
    GLOW_CMD_SET_VOLTAGE,
    GLOW_CMD_MANUAL_MODE,
} glow_command_id_t;

typedef struct {
    glow_command_id_t command_id;
    uint8_t data[16];
} glow_command_t;

esp_err_t glow_context_init(void);
