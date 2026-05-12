#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

// Initialize pulse generators for all configured axes.
esp_err_t pulse_gen_init(void);

// Start continuous STEP pulses on an axis at requested frequency (Hz).
// Produces a 50% duty waveform on STEP pin until pulse_gen_stop() is called.
esp_err_t pulse_gen_start(uint8_t axis, uint32_t step_hz);

// Stop pulse generation on an axis and force STEP low.
esp_err_t pulse_gen_stop(uint8_t axis);

// Query whether pulse generation is active for an axis.
bool pulse_gen_is_running(uint8_t axis);

// Read generated STEP pulse count (rising edges) for an axis.
uint32_t pulse_gen_get_pulse_count(uint8_t axis);

#ifdef __cplusplus
}
#endif
