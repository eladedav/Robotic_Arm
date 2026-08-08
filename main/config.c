// config.c
#include "config.h"
#include "board_pins.h"
#include <math.h>
#include "esp_log.h"

static const char *TAG = "config";

static bool s_config_valid = false;

// Read-only axis configuration table.
// GPIOs: see board_pins.h
const axis_config_t g_axis_cfg[CONFIG_AXES] = {
    // Axis 0
    {
        .step_pin = BOARD_AXIS0_STEP_GPIO,
        .dir_pin  = BOARD_AXIS0_DIR_GPIO,
        .en_pin   = BOARD_AXIS0_EN_GPIO,
        .en_polarity = EN_ACTIVE_LOW,

        .full_steps_per_rev = 200,
        .microsteps = 16,
        .gear_ratio = 2.0f,

        .min_angle_deg = -180.0f,
        .max_angle_deg =  180.0f,

        .max_speed_sps  = 30000.0f,
        .max_accel_sps2 = 20000.0f,

        // derived will be computed in config_init()
        .steps_per_output_rev = 0.0f,
        .steps_per_deg = 0.0f,
    },

    // Axis 1
    {
        .step_pin = BOARD_AXIS1_STEP_GPIO,
        .dir_pin  = BOARD_AXIS1_DIR_GPIO,
        .en_pin   = BOARD_AXIS1_EN_GPIO,
        .en_polarity = EN_ACTIVE_LOW,

        .full_steps_per_rev = 200,
        .microsteps = 16,
        .gear_ratio = 5.0f,

        .min_angle_deg = -180.0f,
        .max_angle_deg =  180.0f,

        .max_speed_sps  = 30000.0f,
        .max_accel_sps2 = 20000.0f,

        .steps_per_output_rev = 0.0f,
        .steps_per_deg = 0.0f,
    },

    // Axis 2
    {
        .step_pin = BOARD_AXIS2_STEP_GPIO,
        .dir_pin  = BOARD_AXIS2_DIR_GPIO,
        .en_pin   = BOARD_AXIS2_EN_GPIO,
        .en_polarity = EN_ACTIVE_LOW,

        .full_steps_per_rev = 200,
        .microsteps = 16,
        .gear_ratio = 1.0f,

        .min_angle_deg = -180.0f,
        .max_angle_deg =  180.0f,

        .max_speed_sps  = 30000.0f,
        .max_accel_sps2 = 20000.0f,

        .steps_per_output_rev = 0.0f,
        .steps_per_deg = 0.0f,
    },

    // Axis 3
    {
        .step_pin = BOARD_AXIS3_STEP_GPIO,
        .dir_pin  = BOARD_AXIS3_DIR_GPIO,
        .en_pin   = BOARD_AXIS3_EN_GPIO,
        .en_polarity = EN_ACTIVE_LOW,

        .full_steps_per_rev = 200,
        .microsteps = 16,
        .gear_ratio = 1.0f,

        .min_angle_deg = -160.0f,
        .max_angle_deg =  160.0f,

        .max_speed_sps  = 30000.0f,
        .max_accel_sps2 = 20000.0f,

        .steps_per_output_rev = 0.0f,
        .steps_per_deg = 0.0f,
    },
};

// We want g_axis_cfg[] to be read-only externally, but we also need to compute derived fields.
// Easiest approach: keep a mutable copy internally after init.
static axis_config_t s_axis_runtime[CONFIG_AXES];

static bool axis_cfg_basic_valid(const axis_config_t *a)
{
    if (!a) return false;

    // Pin sanity (GPIO_NUM_NC exists in newer IDF; if not, remove these checks)
#ifdef GPIO_NUM_NC
    if (a->step_pin == GPIO_NUM_NC || a->dir_pin == GPIO_NUM_NC || a->en_pin == GPIO_NUM_NC) return false;
#endif

    if (a->full_steps_per_rev == 0) return false;
    if (a->microsteps == 0) return false;
    if (!(a->gear_ratio > 0.0f)) return false;

    if (!(a->max_angle_deg > a->min_angle_deg)) return false;

    if (!(a->max_speed_sps > 0.0f)) return false;
    if (!(a->max_accel_sps2 > 0.0f)) return false;

    return true;
}

void config_init(void)
{
    s_config_valid = false;

    // Copy const table to runtime table so we can compute derived values.
    for (uint32_t i = 0; i < CONFIG_AXES; i++) {
        s_axis_runtime[i] = g_axis_cfg[i];

        if (!axis_cfg_basic_valid(&s_axis_runtime[i])) {
            ESP_LOGE(TAG, "Axis %lu basic config invalid", (unsigned long)i);
            return;
        }

        // Derived constants:
        // steps_per_output_rev = full_steps_per_rev * microsteps * gear_ratio
        // steps_per_deg = steps_per_output_rev / 360
        s_axis_runtime[i].steps_per_output_rev =
            (float)s_axis_runtime[i].full_steps_per_rev *
            (float)s_axis_runtime[i].microsteps *
            s_axis_runtime[i].gear_ratio;

        s_axis_runtime[i].steps_per_deg = s_axis_runtime[i].steps_per_output_rev / 360.0f;

        if (!(s_axis_runtime[i].steps_per_deg > 0.0f)) {
            ESP_LOGE(TAG, "Axis %lu derived steps_per_deg invalid", (unsigned long)i);
            return;
        }
    }

    s_config_valid = true;
    ESP_LOGI(TAG, "config_init OK (%d axes)", CONFIG_AXES);
}

bool config_is_valid(void)
{
    return s_config_valid;
}

const axis_config_t* config_axis(uint8_t axis)
{
    if (!s_config_valid) return NULL;
    if (axis >= CONFIG_AXES) return NULL;
    return &s_axis_runtime[axis];
}

int32_t config_angle_deg_to_steps(uint8_t axis, float angle_deg)
{
    const axis_config_t *a = config_axis(axis);
    if (!a) return 0;

    // Saturate to limits
    if (angle_deg < a->min_angle_deg) angle_deg = a->min_angle_deg;
    if (angle_deg > a->max_angle_deg) angle_deg = a->max_angle_deg;

    // steps = angle_deg * steps_per_deg
    // Use lrintf for symmetric rounding
    return (int32_t)lrintf(angle_deg * a->steps_per_deg);
}

float config_steps_to_angle_deg(uint8_t axis, int32_t steps)
{
    const axis_config_t *a = config_axis(axis);
    if (!a) return 0.0f;

    // angle_deg = steps / steps_per_deg
    return ((float)steps) / a->steps_per_deg;
}
