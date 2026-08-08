// safety.c
#include "safety.h"

#include "config.h"
#include "motion.h"
#include "hal_gpio.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

static const char *TAG = "safety";

static TaskHandle_t s_safety_task_handle = NULL;

static volatile system_state_t s_state = SYS_BOOTING;
static volatile uint32_t s_fault_flags = FAULT_NONE;

static volatile uint32_t s_last_cmd_tick = 0;

// Communication watchdog (optional). Set to 0 to disable.
#ifndef SAFETY_COMM_TIMEOUT_MS
#define SAFETY_COMM_TIMEOUT_MS 0
#endif

static inline void set_fault(uint32_t flag)
{
    s_fault_flags |= flag;
}

static inline bool has_faults(void)
{
    return s_fault_flags != FAULT_NONE;
}

static void enter_fault_state(system_state_t fault_state)
{
    // Stop motion first, then disable drivers
    motion_stop_all();
    hal_gpio_disable_all();

    s_state = fault_state;
}

static void safety_task(void *arg)
{
    (void)arg;
    ESP_LOGI(TAG, "Safety task started");

    while (1) {

        // 1) Configuration validity check
        if (!config_is_valid()) {
            set_fault(FAULT_CONFIG_INVALID);
        }

        // 2) Optional comm watchdog
        if (SAFETY_COMM_TIMEOUT_MS > 0) {
            uint32_t now = xTaskGetTickCount();
            uint32_t age_ticks = now - s_last_cmd_tick;
            uint32_t age_ms = age_ticks * portTICK_PERIOD_MS;

            if (s_state == SYS_READY || s_state == SYS_MOVING) {
                if (age_ms > SAFETY_COMM_TIMEOUT_MS) {
                    // Communication timeout policy:
                    // stop motion but keep motor drivers enabled so the arm keeps holding torque.
                    motion_stop_all();
                    s_state = SYS_READY;
                    s_last_cmd_tick = now;
                    ESP_LOGW(TAG, "Comm timeout (%lu ms): motion stopped, drivers remain enabled",
                             (unsigned long)age_ms);
                }
            }
        }

        // 3) If any faults appear while running, enter FAULT state
        if ((s_state == SYS_READY || s_state == SYS_MOVING) && has_faults()) {
            ESP_LOGE(TAG, "Fault detected (0x%08lx), stopping and disabling drivers",
                     (unsigned long)s_fault_flags);
            enter_fault_state(SYS_FAULT);
        }

        // 4) Update moving state (informational)
        if (s_state == SYS_READY && motion_is_any_moving()) {
            s_state = SYS_MOVING;
        } else if (s_state == SYS_MOVING && !motion_is_any_moving()) {
            s_state = SYS_READY;
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

void safety_init(void)
{
    s_state = SYS_BOOTING;
    s_fault_flags = FAULT_NONE;
    s_last_cmd_tick = xTaskGetTickCount();

    // If config already invalid at boot, flag it.
    if (!config_is_valid()) {
        set_fault(FAULT_CONFIG_INVALID);
    }

    // Keep motors disabled until explicitly READY
    hal_gpio_disable_all();

    ESP_LOGI(TAG, "Safety initialized");
}

void safety_start(void)
{
    if (s_safety_task_handle != NULL) {
        return;
    }

    const UBaseType_t prio = 6;

    xTaskCreatePinnedToCore(
        safety_task,
        "safety_task",
        4096,
        NULL,
        prio,
        &s_safety_task_handle,
        1
    );
}

void safety_set_ready(void)
{
    // Only allow READY if no faults and config valid
    if (!config_is_valid()) {
        set_fault(FAULT_CONFIG_INVALID);
    }

    if (has_faults()) {
        s_state = SYS_FAULT;
        hal_gpio_disable_all();
        return;
    }

    s_state = SYS_READY;
}

void safety_estop_trigger(void)
{
    ESP_LOGE(TAG, "E-STOP triggered");
    enter_fault_state(SYS_ESTOP);
}

bool safety_clear_faults(void)
{
    // Policy: allow clearing only if config valid and NOT in ESTOP (or require explicit reset)
    if (!config_is_valid()) {
        set_fault(FAULT_CONFIG_INVALID);
        return false;
    }

    // If you want ESTOP to require power cycle, return false here when SYS_ESTOP
    if (s_state == SYS_ESTOP) {
        return false;
    }

    s_fault_flags = FAULT_NONE;
    s_state = SYS_BOOTING;
    hal_gpio_disable_all();
    return true;
}

system_state_t safety_get_state(void)
{
    return s_state;
}

uint32_t safety_get_fault_flags(void)
{
    return s_fault_flags;
}

bool safety_motion_allowed(void)
{
    if (s_state != SYS_READY && s_state != SYS_MOVING) {
        return false;
    }
    if (has_faults()) {
        return false;
    }
    return true;
}

void safety_note_command_rx(void)
{
    s_last_cmd_tick = xTaskGetTickCount();
}
