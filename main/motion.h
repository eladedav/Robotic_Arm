// motion.h
// Motion control core: maintains per-axis motion state and generates step commands
// Timing-critical step pulse generation will live here (later: timer/ISR engine).

#pragma once

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

// Initialize motion subsystem (allocate/init axis states, validate config assumptions)
void motion_init(void);

// Start motion runtime (creates motion task or starts timers)
void motion_start(void);

// Stop motion (controlled stop: targets set to current; no new steps)
void motion_stop_all(void);

// Stop a single axis (controlled stop)
void motion_stop_axis(uint8_t axis);

// Set absolute target in STEPS (internal units)
bool motion_set_target_steps(uint8_t axis, int32_t target_steps);
bool motion_set_target_steps_rpm(uint8_t axis, int32_t target_steps, float rpm);

// Relative move by angle (degrees) with requested speed in output-shaft RPM.
// dir: 'R' or 'r' = clockwise (+), 'L' or 'l' = counterclockwise (-) in joint step space.
// Resulting target is not clamped; supports multi-turn motion.
bool motion_rotate_degrees(uint8_t axis, char dir, float degrees, float rpm);

// Start continuous jog on one axis at requested RPM and direction.
// Jog continues until motion_stop_axis()/motion_stop_all() or jog_stop endpoint is called.
bool motion_jog_start(uint8_t axis, char dir, float rpm);

// Relative multi-axis move in degrees at one shared RPM.
// deg_by_axis must point to CONFIG_AXES elements; any axis with 0 deg is ignored.
bool motion_movej_relative_degrees(const float *deg_by_axis, float rpm);

// Get current position in STEPS
int32_t motion_get_position_steps(uint8_t axis);

// Set current position (e.g., after homing)
bool motion_set_position_steps(uint8_t axis, int32_t pos_steps);
bool motion_set_position_degrees(uint8_t axis, float pos_deg);

// Mark an axis as homed (current position set to 0 steps).
bool motion_home_axis(uint8_t axis);

// Query if any axis is moving (distance-to-go != 0)
bool motion_is_any_moving(void);

// Query if a specific axis is moving
bool motion_is_axis_moving(uint8_t axis);

#ifdef __cplusplus
}
#endif
