// hal_gpio.h
// Hardware Abstraction Layer for GPIO access (stepper drivers)

#pragma once

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

// -------------------- Initialization --------------------

// Initialize GPIOs for all axes using config data.
// Sets STEP/DIR/EN pins as outputs and puts drivers in a SAFE (disabled) state.
void hal_gpio_init(void);

// -------------------- Driver enable / disable --------------------

// Enable all motor drivers (according to configured polarity).
void hal_gpio_enable_all(void);

// Disable all motor drivers (safe state).
void hal_gpio_disable_all(void);

// Enable or disable a single axis driver.
void hal_gpio_enable_axis(uint8_t axis, bool enable);

// -------------------- Low-level pin control --------------------

// Set direction pin for an axis.
// dir = true  -> forward
// dir = false -> reverse
void hal_gpio_set_dir(uint8_t axis, bool dir);

// Generate a single STEP pulse for an axis.
// Pulse width and timing guarantees are handled by the caller (motion module).
void hal_gpio_step_pulse(uint8_t axis);

#ifdef __cplusplus
}
#endif
