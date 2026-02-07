// safety.h
// System safety supervisor: states, faults, motion permissions, and emergency stop behavior.

#pragma once

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    SYS_BOOTING = 0,
    SYS_READY,
    SYS_MOVING,
    SYS_FAULT,
    SYS_ESTOP,
    SYS_HOMING
} system_state_t;

// Fault flags (bitmask)
typedef enum {
    FAULT_NONE            = 0,
    FAULT_CONFIG_INVALID  = (1u << 0),
    FAULT_LIMIT_VIOLATION = (1u << 1),
    FAULT_COMM_TIMEOUT    = (1u << 2),
    FAULT_INTERNAL        = (1u << 3),
} fault_flags_t;

// Initialize safety subsystem (call once at boot)
void safety_init(void);

// Start safety runtime supervision task
void safety_start(void);

// Mark system as ready (allows motion if no faults and no ESTOP)
void safety_set_ready(void);

// Emergency stop: immediate disable + stop motion
void safety_estop_trigger(void);

// Clear faults and exit FAULT/ESTOP state if allowed (your policy decides what “allowed” means)
bool safety_clear_faults(void);

// Query state
system_state_t safety_get_state(void);
uint32_t safety_get_fault_flags(void);

// Motion permission gate
bool safety_motion_allowed(void);

// Inform safety that a valid command was received (used for comm watchdog)
void safety_note_command_rx(void);

#ifdef __cplusplus
}
#endif
