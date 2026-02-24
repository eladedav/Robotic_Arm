// motion.c
#include "motion.h"

#include "config.h"
#include "hal_gpio.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

static const char *TAG = "motion";

// -------------------- Internal state --------------------

typedef struct {
    int32_t current_steps;
    int32_t target_steps;
    bool    moving;
} axis_motion_t;

static axis_motion_t s_axis[CONFIG_AXES];

// Motion task handle (optional, for future control)
static TaskHandle_t s_motion_task_handle = NULL;

// Tick rate for this *simple* stepping approach.
// For real multi-axis smooth motion, replace with timer/ISR pulse scheduling.
#ifndef MOTION_TASK_PERIOD_MS
#define MOTION_TASK_PERIOD_MS 1
#endif

static inline bool axis_valid(uint8_t axis) {
    return axis < CONFIG_AXES;
}

// -------------------- Core stepping primitive --------------------
// Very simple: if target != current, do one step toward target.
// Direction is set each time; later you can optimize by caching dir.
static void motion_step_once(uint8_t axis)
{
    axis_motion_t *m = &s_axis[axis];

    int32_t err = m->target_steps - m->current_steps;
    if (err == 0) {
        m->moving = false;
        return;
    }

    // Decide direction
    bool dir = (err > 0);
    hal_gpio_set_dir(axis, dir);

    // Issue one step pulse
    hal_gpio_step_pulse(axis);

    // Update position estimate
    m->current_steps += (dir ? 1 : -1);
    m->moving = true;
}

// -------------------- Motion task --------------------

static void motion_task(void *arg)
{
    (void)arg;
    ESP_LOGI(TAG, "Motion task started");

    for (;;) {

        // --------- Option B: sleep when idle ----------
        // If nothing is moving, block here until someone notifies us (new target/stop/etc.)
        if (!motion_is_any_moving()) {
            // Clear any pending notifications then block
            (void) ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        }

        // --------- Option A: deterministic loop while moving ----------
        TickType_t last_wake = xTaskGetTickCount();

        while (motion_is_any_moving()) {

            // One step max per axis per cycle (your current simple stepping model)
            for (uint8_t i = 0; i < CONFIG_AXES; i++) {
                motion_step_once(i);
            }

            // Fixed-rate pacing (use DelayUntil to reduce jitter)
            vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(MOTION_TASK_PERIOD_MS));

            // If multiple notifications came in while moving, consume them (non-blocking)
            (void) ulTaskNotifyTake(pdTRUE, 0);
        }

        // Loop back; if idle, we'll block again.
    }
}

// -------------------- Public API --------------------

void motion_init(void)
{
    // Initialize state; keep positions at 0 until homing is implemented.
    for (uint8_t i = 0; i < CONFIG_AXES; i++) {
        s_axis[i].current_steps = 0;
        s_axis[i].target_steps  = 0;
        s_axis[i].moving        = false;
    }

    ESP_LOGI(TAG, "Motion initialized");
}

void motion_start(void)
{
    // Create motion task once
    if (s_motion_task_handle != NULL) {
        return;
    }

    // High priority is typical for motion, but final value depends on your system
    const UBaseType_t prio = 5;

    xTaskCreatePinnedToCore(
        motion_task,
        "motion_task",
        4096,
        NULL,
        prio,
        &s_motion_task_handle,
        1  // core 1 is often used for application logic
    );
}

void motion_stop_all(void)
{
    for (uint8_t i = 0; i < CONFIG_AXES; i++) {
        s_axis[i].target_steps = s_axis[i].current_steps;
        s_axis[i].moving = false;
    }
    if (s_motion_task_handle != NULL) xTaskNotifyGive(s_motion_task_handle);
}

void motion_stop_axis(uint8_t axis)
{
    if (!axis_valid(axis)) return;
    s_axis[axis].target_steps = s_axis[axis].current_steps;
    s_axis[axis].moving = false;
}

bool motion_set_target_steps(uint8_t axis, int32_t target_steps)
{
    if (!axis_valid(axis)) return false;

    s_axis[axis].target_steps = target_steps;

    // Wake motion task if it exists (so it can start moving immediately)
    if (s_motion_task_handle != NULL) {
        xTaskNotifyGive(s_motion_task_handle);
    }
    return true;
}

int32_t motion_get_position_steps(uint8_t axis)
{
    if (!axis_valid(axis)) return 0;
    return s_axis[axis].current_steps;
}

bool motion_set_position_steps(uint8_t axis, int32_t pos_steps)
{
    if (!axis_valid(axis)) return false;
    s_axis[axis].current_steps = pos_steps;
    // Keep target aligned to prevent sudden jump
    s_axis[axis].target_steps  = pos_steps;
    s_axis[axis].moving        = false;
    return true;
}

bool motion_is_any_moving(void)
{
    for (uint8_t i = 0; i < CONFIG_AXES; i++) {
        if (s_axis[i].target_steps != s_axis[i].current_steps) return true;
    }
    return false;
}

bool motion_is_axis_moving(uint8_t axis)
{
    if (!axis_valid(axis)) return false;
    return (s_axis[axis].target_steps != s_axis[axis].current_steps);
}
