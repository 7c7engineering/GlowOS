#include <stdio.h>
#include <string.h>
#include "glow_core.h"
#include "glow_storage.h"
#include "glow_power.h"
#include "glow_led.h"
#include "esp_log.h"
#include "esp_check.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "glow_sensors.h"
#include "rom/ets_sys.h"

const char *TAG = "GLOW_CORE";
extern glow_context_t *g_context;

static glow_device_config_t s_core_config;
static int64_t boost_start_us = 0;
static int64_t last_continuity_check_us = 0;
const int64_t continuity_check_interval_us = 2500000; // Check glow wire continuity every 2.5 seconds
const int64_t continuity_backoff_time_us = 1000000;   // If we just started glowing, backof this time before checking continuity.
static uint32_t low_battery_voltage_mV = 0;

static esp_err_t glow_core_detect_battery_cells(void)
{ // Try to auto-detect battery cell count based on voltage measurement
    // Assuming LiPo battery with nominal voltage of 3.7V per cell and fully charged voltage of 4.2V per cell
    if (g_context->latest_measurement.vbat_mV > 0)
    {
        uint8_t detected_cells = (g_context->latest_measurement.vbat_mV + 200) / 4200; // Add some margin and round to nearest integer
        if (detected_cells > 0 && detected_cells <= 6)
        { // Limit to reasonable cell counts
            s_core_config.battery_ncells = detected_cells;
            low_battery_voltage_mV = (uint32_t)(s_core_config.battery_ncells * s_core_config.battery_low_voltage * 1000.0f);
            ESP_LOGI(TAG, "Auto-detected battery cells: %u, low voltage threshold: %.3f V (%u mV)", s_core_config.battery_ncells, s_core_config.battery_low_voltage, low_battery_voltage_mV);
            return ESP_OK;
        }
        else
        {
            ESP_LOGW(TAG, "Auto-detected invalid battery cell count: %u", detected_cells);
        }
    }
    else
    {
        ESP_LOGW(TAG, "Battery voltage measurement not available for auto-detection");
    }
    return ESP_ERR_NOT_FOUND;
}

static void glow_core_load_active_config(void)
{
    glow_device_config_t cfg = {0};
    if (glow_storage_get_device_config(&cfg) == ESP_OK)
    {
        s_core_config = cfg;
        // Pre-calculate low battery voltage in mV: number of cells * low voltage per cell
        // If number of cells is 0 -> try to auto-detect based on battery voltage measurement
        if (s_core_config.battery_ncells > 0)
        {
            low_battery_voltage_mV = (uint32_t)(s_core_config.battery_ncells * s_core_config.battery_low_voltage * 1000.0f);
            ESP_LOGI(TAG, "Loaded config: %u cells, low voltage %.3f V (%.0f mV)", s_core_config.battery_ncells, s_core_config.battery_low_voltage, (float)low_battery_voltage_mV);
        }
        else
        {
            glow_core_detect_battery_cells();
        }
    }
}

esp_err_t glow_core_goto_state(glow_system_state_t new_state)
{
    if (g_context == NULL)
    {
        ESP_LOGE(TAG, "Glow context is not initialized");
        return ESP_ERR_INVALID_STATE;
    }
    switch (new_state)
    {
    case GLOW_STATE_OFF:
        glow_power_enable(false);
        glow_power_set_voltage(0.0f);
        xEventGroupClearBits(g_context->system_status, GLOW_MANUAL_MODE_BIT);
        glow_led_set_color(0, 0, 0);
        break;
    case GLOW_STATE_BOOST:
        boost_start_us = esp_timer_get_time();
        glow_power_set_voltage(s_core_config.glow_boost);
        glow_power_enable(true);
        glow_led_set_color(0, 255, 255);
        break;
    case GLOW_STATE_GLOWING:
        glow_power_set_voltage(s_core_config.glow_voltage);
        glow_power_enable(true);
        glow_led_set_color(0, 255, 0);
        break;
    case GLOW_STATE_MANUAL:
        xEventGroupSetBits(g_context->system_status, GLOW_MANUAL_MODE_BIT);
        break;
    case GLOW_STATE_LOW_BATTERY:
        glow_power_enable(false);
        glow_led_set_color(255, 165, 0);
        break;
    case GLOW_STATE_BROKEN_WIRE:
        glow_power_enable(false);
        glow_led_set_color(255, 0, 0);
        break;
    case GLOW_STATE_ERROR:
        glow_power_enable(false);
        glow_led_set_color(255, 0, 0);
        break;

    default:
        break;
    }
    g_context->system_state = new_state;
    return ESP_OK;
}

esp_err_t check_glow_wire_continuity(bool *out_continuity_ok)
{
    // at the lowest voltage, very quickly enable the glow plug and check the current
    glow_power_enable(false);
    vTaskDelay(1);
    glow_power_set_voltage(0.8f); // Set very low voltage, we don't want the wire to glow, only check the current.
    ets_delay_us(10);             // Short delay to allow voltage to stabilize
    glow_power_enable(true);
    ets_delay_us(2000); // Wait for 2 ms to allow current to stabilize
    uint32_t iout_mA = 0;
    esp_err_t err = glow_sensor_get_iout_mA(&iout_mA);
    glow_power_enable(false);
    // we are at risk of starving the watchdog. So quickly yield back to the scheduler to allow other tasks to run and reset the watchdog timer
    vTaskDelay(1);

    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to read current for continuity check: %s", esp_err_to_name(err));
        return err;
    }
    // ESP_LOGI(TAG, "Glow wire continuity check: measured current = %u mA", iout_mA);
    *out_continuity_ok = iout_mA > 10; // If current is above 10 mA, we assume the glow wire is continuous. This threshold may need to be adjusted based on actual measurements.
    return ESP_OK;
}

esp_err_t glow_core_get_measurements()
{
    if (g_context == NULL)
    {
        ESP_LOGE(TAG, "Glow context is not initialized");
        return ESP_ERR_INVALID_STATE;
    }
    uint32_t vout_mV = 0, iout_mA = 0, vbus_mV = 0, vbat_mV = 0, rc_pwm_percentage = 0;
    bool rc_pwm_signal_valid = false;
    float temp_degC = 0.0f;
    esp_err_t err = glow_sensor_get_temperature_degC(&temp_degC);
    if (err == ESP_OK)
    {
        err = glow_sensor_get_vout_mV(&vout_mV);
    }
    if (err == ESP_OK)
    {
        err = glow_sensor_get_iout_mA(&iout_mA);
    }
    if (err == ESP_OK)
    {
        err = glow_sensor_get_vbus_mV(&vbus_mV);
    }
    if (err == ESP_OK)
    {
        err = glow_sensor_get_vbat_mV(&vbat_mV);
    }
    if (err == ESP_OK)
    {
        err = glow_sensor_get_rc_pwm(&rc_pwm_percentage, &rc_pwm_signal_valid);
    }
    if (err == ESP_OK)
    {
        xSemaphoreTake(g_context->measurement_mutex, portMAX_DELAY);
        g_context->latest_measurement.vout_mV = vout_mV;
        g_context->latest_measurement.iout_mA = iout_mA;
        g_context->latest_measurement.vbus_mV = vbus_mV;
        g_context->latest_measurement.vbat_mV = vbat_mV;
        g_context->latest_measurement.temperature_degC = temp_degC;
        g_context->latest_measurement.rc_pwm_percentage = rc_pwm_percentage;
        g_context->latest_measurement.rc_pwm_signal_valid = rc_pwm_signal_valid ? 1U : 0U;
        xSemaphoreGive(g_context->measurement_mutex);
        // ESP_LOGI(TAG, "Measurements updated: vout=%u mV, iout=%u mA, vbus=%u mV, vbat=%u mV, temp=%.2f C, rc_pwm=%u%% (valid=%s)", vout_mV, iout_mA, vbus_mV, vbat_mV, temp_degC, rc_pwm_percentage, rc_pwm_signal_valid ? "true" : "false");
    }
    else
    {
        ESP_LOGE(TAG, "Failed to read sensors: %s", esp_err_to_name(err));
    }
    return ESP_OK;
}

void vtask_core(void *arg)
{
    (void)arg;
    bool glow_wire_continuity_ok = false;

    while (1)
    {
        // First check for low battery condition
        esp_err_t err = glow_core_get_measurements();
        if (err != ESP_OK)
        {
            ESP_LOGE(TAG, "Failed to get measurements: %s", esp_err_to_name(err));
            glow_core_goto_state(GLOW_STATE_ERROR);
        }
        if (g_context->latest_measurement.vbat_mV > 0 && g_context->latest_measurement.vbat_mV < low_battery_voltage_mV)
        {
            if (g_context->system_state != GLOW_STATE_LOW_BATTERY)
            {
                ESP_LOGW(TAG, "Battery voltage critically low: %u mV", g_context->latest_measurement.vbat_mV);
                glow_core_goto_state(GLOW_STATE_LOW_BATTERY);
            }
        }
        else if (g_context->system_state == GLOW_STATE_LOW_BATTERY)
        {
            ESP_LOGI(TAG, "Battery voltage recovered: %u mV", g_context->latest_measurement.vbat_mV);
            glow_core_goto_state(GLOW_STATE_OFF);
        }

        switch (g_context->system_state)
        {
        case GLOW_STATE_OFF: // Not glowing the glowplug, waiting for RC receiver or user command to start glowing
            // While in off state, periodically check glow wire continuity to detect if the glow wire is broken and update system status accordingly
            if (esp_timer_get_time() - last_continuity_check_us >= continuity_check_interval_us)
            {
                last_continuity_check_us = esp_timer_get_time();
                if (check_glow_wire_continuity(&glow_wire_continuity_ok) == ESP_OK)
                {
                    if (!glow_wire_continuity_ok)
                    {
                        ESP_LOGE(TAG, "Glow wire continuity check failed! Glow wire may be broken.");
                        glow_core_goto_state(GLOW_STATE_BROKEN_WIRE);
                    }
                }
                else
                {
                    ESP_LOGE(TAG, "Failed to perform glow wire continuity check");
                }
            }
            if (g_context->latest_measurement.rc_pwm_signal_valid &&
                g_context->latest_measurement.rc_pwm_percentage > (uint32_t)s_core_config.receiver_threshold)
            {
                glow_core_goto_state(GLOW_STATE_BOOST);
            }
            break;
        case GLOW_STATE_BOOST:
            // Check boost timer
            if ((esp_timer_get_time() - boost_start_us) >= (int64_t)(s_core_config.glow_boost_time * 1000000.0f))
            {
                glow_core_goto_state(GLOW_STATE_GLOWING);
            }
            // Check RC signal
            if (!g_context->latest_measurement.rc_pwm_signal_valid ||
                g_context->latest_measurement.rc_pwm_percentage <= (uint32_t)s_core_config.receiver_threshold)
            {
                glow_core_goto_state(GLOW_STATE_OFF);
            }
            // Check current (continuity) - if current drops significantly during boost phase, it's a sign of broken glow wire
            if (boost_start_us + continuity_backoff_time_us < esp_timer_get_time() && g_context->latest_measurement.iout_mA > 0 && g_context->latest_measurement.iout_mA < 10) // Threshold of 10 mA
            {
                ESP_LOGE(TAG, "Current dropped during boost phase: %u mA. This may indicate a broken glow wire.", g_context->latest_measurement.iout_mA);
                glow_core_goto_state(GLOW_STATE_BROKEN_WIRE);
            }
            break;
        case GLOW_STATE_GLOWING:
            // Check RC signal - if signal is lost or drops below threshold, turn off glow plug
            if (!g_context->latest_measurement.rc_pwm_signal_valid ||
                g_context->latest_measurement.rc_pwm_percentage <= (uint32_t)s_core_config.receiver_threshold)
            {
                glow_core_goto_state(GLOW_STATE_OFF);
            }
            // Check current (continuity) - if current drops significantly, it's a sign of broken glow wire
            if (boost_start_us + continuity_backoff_time_us < esp_timer_get_time() && g_context->latest_measurement.iout_mA > 0 && g_context->latest_measurement.iout_mA < 10) // Threshold of 10 mA
            {
                ESP_LOGE(TAG, "Current dropped during glowing phase: %u mA. This may indicate a broken glow wire.", g_context->latest_measurement.iout_mA);
                glow_core_goto_state(GLOW_STATE_BROKEN_WIRE);
            }
            break;
        case GLOW_STATE_MANUAL:
            // In MANUAL state, we might want to allow direct control over the hardware.
            break;
        case GLOW_STATE_LOW_BATTERY:
            // Stay in low battery state until voltage recovers
            break;
        case GLOW_STATE_BROKEN_WIRE: // Periodically check if the wire is attached again, if continuity is ok, go back to OFF state
            // Every interval:
            if (esp_timer_get_time() - last_continuity_check_us >= continuity_check_interval_us)
            {
                last_continuity_check_us = esp_timer_get_time();
                if (check_glow_wire_continuity(&glow_wire_continuity_ok) == ESP_OK && glow_wire_continuity_ok)
                {
                    ESP_LOGI(TAG, "Glow wire continuity restored. Returning to OFF state.");
                    glow_core_goto_state(GLOW_STATE_OFF);
                }
                break;
            }
        case GLOW_STATE_ERROR:
            break;
        default:
            ESP_LOGW(TAG, "Unknown system state: %d", g_context->system_state);
            break;
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

void vtask_core_cmd_handler(void *arg)
{
    (void)arg;
    glow_command_t cmd;
    while (1)
    {
        if (xQueueReceive(g_context->control_queue, &cmd, portMAX_DELAY) == pdTRUE)
        {
            // Handle the command (this is where you would add logic to control the hardware based on the command)
            switch (cmd.command_id)
            {
            case GLOW_CMD_SET_LED_COLOR:
                if ((xEventGroupGetBits(g_context->system_status) & GLOW_MANUAL_MODE_BIT) == 0)
                {
                    ESP_LOGW(TAG, "Ignoring LED color set command because not in MANUAL mode");
                    break;
                }
                glow_led_set_color(cmd.data[0], cmd.data[1], cmd.data[2]);
                break;
            case GLOW_CMD_SET_VOLTAGE:
                if ((xEventGroupGetBits(g_context->system_status) & GLOW_MANUAL_MODE_BIT) == 0)
                {
                    ESP_LOGW(TAG, "Ignoring voltage set command because not in MANUAL mode");
                    break;
                }
                float voltage;
                memcpy(&voltage, cmd.data, sizeof(float));
                glow_power_set_voltage(voltage);
                break;
            case GLOW_CMD_ENABLE_POWER:
                if ((xEventGroupGetBits(g_context->system_status) & GLOW_MANUAL_MODE_BIT) == 0)
                {
                    ESP_LOGW(TAG, "Ignoring power enable command because not in MANUAL mode");
                    break;
                }
                bool enable = cmd.data[0] != 0;
                glow_power_enable(enable);
                break;
            case GLOW_CMD_MANUAL_MODE:
                if (cmd.data[0] != 0)
                {
                    glow_core_goto_state(GLOW_STATE_MANUAL);
                }
                else
                {
                    glow_core_goto_state(GLOW_STATE_OFF);
                }
                break;
            default:
                ESP_LOGW(TAG, "Unknown command ID: %d", cmd.command_id);
                break;
            }
        }
    }
}

esp_err_t glow_core_init(void)
{
    if (g_context == NULL)
    {
        ESP_LOGE(TAG, "Glow context is not initialized");
        return ESP_ERR_INVALID_STATE;
    }
    g_context->system_state = GLOW_STATE_OFF;
    glow_core_load_active_config();
    BaseType_t task_created = xTaskCreate(vtask_core, "core_task", 4096, NULL, tskIDLE_PRIORITY + 1, NULL);
    ESP_RETURN_ON_FALSE(task_created == pdPASS, ESP_ERR_NO_MEM, TAG, "Failed to create core task");
    task_created = xTaskCreate(vtask_core_cmd_handler, "core_cmd_handler_task", 4096, NULL, tskIDLE_PRIORITY + 1, NULL);
    ESP_RETURN_ON_FALSE(task_created == pdPASS, ESP_ERR_NO_MEM, TAG, "Failed to create core command handler task");
    return ESP_OK;
}
