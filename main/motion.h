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

// Get current position in STEPS
int32_t motion_get_position_steps(uint8_t axis);

// Set current position (e.g., after homing)
bool motion_set_position_steps(uint8_t axis, int32_t pos_steps);

// Query if any axis is moving (distance-to-go != 0)
bool motion_is_any_moving(void);

// Query if a specific axis is moving
bool motion_is_axis_moving(uint8_t axis);

#ifdef __cplusplus
}
#endif
