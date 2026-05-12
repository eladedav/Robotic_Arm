// config.h
// Project-wide configuration (hardware map + mechanical params + limits)
// This module should be read-only during runtime and initialized once at boot.

#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "driver/gpio.h"


#ifdef __cplusplus
extern "C" {
#endif

// -------------------- System constants --------------------

#define CONFIG_AXES 4

// Enable pin polarity (depends on your driver wiring)
typedef enum {
    EN_ACTIVE_LOW = 0,   // EN=0 enables driver, EN=1 disables
    EN_ACTIVE_HIGH = 1,  // EN=1 enables driver, EN=0 disables
} en_polarity_t;

// -------------------- Per-axis configuration --------------------

typedef struct {
    // Hardware pins
    gpio_num_t step_pin;
    gpio_num_t dir_pin;
    gpio_num_t en_pin;
    en_polarity_t en_polarity;

    // Motor/mechanics
    uint32_t full_steps_per_rev;     // e.g. 200 for 1.8° stepper
    uint32_t microsteps;             // e.g. 16, 32 ... must match driver DIP
    float    gear_ratio;             // output_rev = motor_rev / gear_ratio (use 1.0 if direct)

    // Limits (pick one representation for the UI; motion will typically use steps internally)
    float min_angle_deg;             // soft limit
    float max_angle_deg;             // soft limit

    // Motion constraints
    float max_speed_sps;             // steps per second (at motor driver STEP input)
    float max_accel_sps2;            // steps per second^2

    // Derived constants (computed in config_init)
    float steps_per_output_rev;      // full_steps_per_rev * microsteps * gear_ratio
    float steps_per_deg;             // steps_per_output_rev / 360
} axis_config_t;

// -------------------- API --------------------

// Initialize and validate configuration. Call once from app_main().
void config_init(void);

// Returns true if config_init() succeeded and configuration is valid.
bool config_is_valid(void);

// Get pointer to axis configuration (0..CONFIG_AXES-1). Returns NULL if axis index invalid.
const axis_config_t* config_axis(uint8_t axis);

// Convenience conversion helpers (use config_axis()->steps_per_deg internally)
int32_t config_angle_deg_to_steps(uint8_t axis, float angle_deg);
float   config_steps_to_angle_deg(uint8_t axis, int32_t steps);

// Optional: expose all axis configs as a read-only array (defined in config.c)
extern const axis_config_t g_axis_cfg[CONFIG_AXES];

#ifdef __cplusplus
}
#endif
