// motion.c
#include "motion.h"

#include "config.h"
#include "hal_gpio.h"
#include "pulse_gen.h"

#include <math.h>
#include <limits.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

static const char *TAG = "motion";

// -------------------- Internal state --------------------

typedef struct {
    int32_t     current_steps;
    int32_t     target_steps;
    bool        moving;
    int8_t      dir_sign;         // +1 or -1 when moving
    uint32_t    step_hz;          // requested pulse frequency to pulse_gen
    uint32_t    pulse_snapshot;   // last observed pulse counter
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

static int dir_to_sign(char dir)
{
    char d = dir;
    if (d >= 'a' && d <= 'z') {
        d = (char)(d - 'a' + 'A');
    }
    if (d == 'R') return 1;
    if (d == 'L') return -1;
    return 0;
}

static inline TickType_t motion_ms_to_ticks_min1(uint32_t ms)
{
    TickType_t t = pdMS_TO_TICKS(ms);
    return (t > 0U) ? t : 1U;
}

static inline bool reached_target(const axis_motion_t *m)
{
    return m->dir_sign > 0 ? (m->current_steps >= m->target_steps)
                           : (m->current_steps <= m->target_steps);
}

static uint32_t rpm_to_step_hz(const axis_config_t *a, float rpm)
{
    if (!(rpm > 0.0f) || !a || !(a->steps_per_output_rev > 0.0f)) {
        return 0U;
    }

    float sps = (rpm * a->steps_per_output_rev) / 60.0f;
    if (!(sps > 0.0f)) {
        return 0U;
    }

    if (a->max_speed_sps > 0.0f && sps > a->max_speed_sps) {
        sps = a->max_speed_sps;
    }

    uint32_t hz = (uint32_t)lrintf(sps);
    return (hz > 0U) ? hz : 1U;
}

static int32_t clamp_target_steps_to_limits(uint8_t axis, int32_t target_steps, const axis_config_t *a)
{
    int32_t lo = config_angle_deg_to_steps(axis, a->min_angle_deg);
    int32_t hi = config_angle_deg_to_steps(axis, a->max_angle_deg);
    if (lo > hi) {
        int32_t t = lo;
        lo = hi;
        hi = t;
    }
    if (target_steps < lo) return lo;
    if (target_steps > hi) return hi;
    return target_steps;
}

static bool start_axis_move(uint8_t axis, int32_t target_steps, float rpm)
{
    if (!axis_valid(axis) || !config_is_valid()) {
        return false;
    }

    axis_motion_t *m = &s_axis[axis];
    const axis_config_t *a = config_axis(axis);
    if (!a) {
        return false;
    }

    // Enforce configured axis soft limits for all motion commands.
    target_steps = clamp_target_steps_to_limits(axis, target_steps, a);

    int32_t delta = target_steps - m->current_steps;
    if (delta == 0) {
        (void)pulse_gen_stop(axis);
        m->target_steps = m->current_steps;
        m->moving = false;
        m->step_hz = 0;
        return true;
    }

    uint32_t hz = rpm_to_step_hz(a, rpm);
    if (hz == 0U) {
        return false;
    }

    m->target_steps = target_steps;
    m->dir_sign = (delta > 0) ? 1 : -1;
    m->step_hz = hz;
    m->pulse_snapshot = pulse_gen_get_pulse_count(axis);

    hal_gpio_set_dir(axis, m->dir_sign > 0);
    if (pulse_gen_start(axis, hz) != ESP_OK) {
        return false;
    }
    m->moving = true;
    return true;
}

static void motion_task(void *arg)
{
    (void)arg;
    ESP_LOGI(TAG, "Motion task started");

    for (;;) {
        if (!motion_is_any_moving()) {
            (void) ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        }

        TickType_t last_wake = xTaskGetTickCount();
        while (motion_is_any_moving()) {
            for (uint8_t i = 0; i < CONFIG_AXES; i++) {
                axis_motion_t *m = &s_axis[i];
                if (!m->moving) {
                    continue;
                }

                uint32_t now_count = pulse_gen_get_pulse_count(i);
                uint32_t dcount = now_count - m->pulse_snapshot;
                if (dcount > 0U) {
                    m->pulse_snapshot = now_count;
                    int64_t next = (int64_t)m->current_steps + (int64_t)m->dir_sign * (int64_t)dcount;
                    m->current_steps = (int32_t)next;
                }

                if (reached_target(m)) {
                    m->current_steps = m->target_steps;
                    (void)pulse_gen_stop(i);
                    m->moving = false;
                    m->step_hz = 0;
                }
            }

            vTaskDelayUntil(&last_wake, motion_ms_to_ticks_min1(MOTION_TASK_PERIOD_MS));
            (void) ulTaskNotifyTake(pdTRUE, 0);
        }
    }
}

// -------------------- Public API --------------------

void motion_init(void)
{
    // Initialize state; keep positions at 0 until homing is implemented.
    for (uint8_t i = 0; i < CONFIG_AXES; i++) {
        s_axis[i].current_steps   = 0;
        s_axis[i].target_steps    = 0;
        s_axis[i].moving          = false;
        s_axis[i].dir_sign        = 1;
        s_axis[i].step_hz         = 0;
        s_axis[i].pulse_snapshot  = 0;
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
        (void)pulse_gen_stop(i);
        s_axis[i].target_steps   = s_axis[i].current_steps;
        s_axis[i].moving         = false;
        s_axis[i].step_hz        = 0;
    }
    if (s_motion_task_handle != NULL) xTaskNotifyGive(s_motion_task_handle);
}

void motion_stop_axis(uint8_t axis)
{
    if (!axis_valid(axis)) return;
    (void)pulse_gen_stop(axis);
    s_axis[axis].target_steps   = s_axis[axis].current_steps;
    s_axis[axis].moving         = false;
    s_axis[axis].step_hz        = 0;
}

bool motion_set_target_steps(uint8_t axis, int32_t target_steps)
{
    return motion_set_target_steps_rpm(axis, target_steps, 2.0f);
}

bool motion_set_target_steps_rpm(uint8_t axis, int32_t target_steps, float rpm)
{
    if (!axis_valid(axis)) return false;

    int32_t cur = s_axis[axis].current_steps;
    if (!start_axis_move(axis, target_steps, rpm)) {
        return false;
    }

    if (target_steps != cur) {
        ESP_LOGI(TAG, "move start axis %u -> %ld steps (from %ld) @ %.2f rpm, %lu Hz",
                 (unsigned)axis, (long)target_steps, (long)cur, rpm, (unsigned long)s_axis[axis].step_hz);
    }

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
    (void)pulse_gen_stop(axis);
    s_axis[axis].current_steps   = pos_steps;
    s_axis[axis].target_steps    = pos_steps;
    s_axis[axis].moving          = false;
    s_axis[axis].step_hz         = 0;
    s_axis[axis].pulse_snapshot  = pulse_gen_get_pulse_count(axis);
    return true;
}

static bool move_relative_degrees_rpm(uint8_t axis, char dir, float degrees, float rpm, bool clamp_limits)
{
    if (!axis_valid(axis) || !(degrees > 0.0f) || !(rpm > 0.0f) || !config_is_valid()) {
        return false;
    }

    char d = dir;
    if (d >= 'a' && d <= 'z') {
        d = (char)(d - 'a' + 'A');
    }

    int sign = 0;
    if (d == 'R') {
        sign = 1;
    } else if (d == 'L') {
        sign = -1;
    } else {
        return false;
    }

    const axis_config_t *a = config_axis(axis);
    if (!a) {
        return false;
    }

    int32_t delta = (int32_t)lrintf((float)sign * degrees * a->steps_per_deg);
    if (delta == 0) {
        return true;
    }

    int64_t new_target = (int64_t)s_axis[axis].current_steps + (int64_t)delta;

    int32_t target;
    if (clamp_limits) {
        int32_t lo = config_angle_deg_to_steps(axis, a->min_angle_deg);
        int32_t hi = config_angle_deg_to_steps(axis, a->max_angle_deg);
        if (lo > hi) {
            int32_t t = lo;
            lo = hi;
            hi = t;
        }

        if (new_target < lo) {
            target = lo;
        } else if (new_target > hi) {
            target = hi;
        } else {
            target = (int32_t)new_target;
        }
    } else {
        target = (int32_t)new_target;
    }

    if (target == s_axis[axis].current_steps) {
        return true;
    }

    int32_t cur = s_axis[axis].current_steps;
    if (!start_axis_move(axis, target, rpm)) {
        return false;
    }

    ESP_LOGI(TAG, "move start j%u %c %.2f deg -> %ld steps (from %ld) @ %.2f rpm, %lu Hz",
             (unsigned)axis, d, degrees, (long)target, (long)cur, rpm, (unsigned long)s_axis[axis].step_hz);

    if (s_motion_task_handle != NULL) {
        xTaskNotifyGive(s_motion_task_handle);
    }
    return true;
}

bool motion_rotate_degrees(uint8_t axis, char dir, float degrees, float rpm)
{
    // Target clamping to configured soft limits is enforced in start_axis_move().
    return move_relative_degrees_rpm(axis, dir, degrees, rpm, false);
}

bool motion_jog_start(uint8_t axis, char dir, float rpm)
{
    if (!axis_valid(axis) || !(rpm > 0.0f) || !config_is_valid()) {
        return false;
    }

    int sign = dir_to_sign(dir);
    if (sign == 0) {
        return false;
    }

    int32_t cur = s_axis[axis].current_steps;
    // Large finite target so jog behaves continuously for practical UI usage.
    int32_t target = (sign > 0) ? (INT32_MAX - 1024) : (INT32_MIN + 1024);
    if (!start_axis_move(axis, target, rpm)) {
        return false;
    }

    ESP_LOGI(TAG, "jog start axis %u %c @ %.2f rpm (from %ld)",
             (unsigned)axis, (sign > 0) ? 'R' : 'L', rpm, (long)cur);

    if (s_motion_task_handle != NULL) {
        xTaskNotifyGive(s_motion_task_handle);
    }
    return true;
}

bool motion_movej_relative_degrees(const float *deg_by_axis, float rpm)
{
    if (!deg_by_axis || !(rpm > 0.0f) || !config_is_valid()) {
        return false;
    }

    bool any = false;
    for (uint8_t i = 0; i < CONFIG_AXES; i++) {
        float d = deg_by_axis[i];
        if (d > 0.0f) {
            if (!move_relative_degrees_rpm(i, 'R', d, rpm, false)) {
                return false;
            }
            any = true;
        } else if (d < 0.0f) {
            if (!move_relative_degrees_rpm(i, 'L', -d, rpm, false)) {
                return false;
            }
            any = true;
        }
    }

    return any;
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

bool motion_set_position_degrees(uint8_t axis, float pos_deg)
{
    if (!axis_valid(axis) || !config_is_valid()) {
        return false;
    }
    int32_t pos_steps = config_angle_deg_to_steps(axis, pos_deg);
    return motion_set_position_steps(axis, pos_steps);
}

bool motion_home_axis(uint8_t axis)
{
    if (!axis_valid(axis)) {
        return false;
    }
    motion_stop_axis(axis);
    return motion_set_position_steps(axis, 0);
}
