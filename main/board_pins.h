// board_pins.h
// Single place for GPIO (and related) pin assignments for this board.
// Edit here when you rewire hardware; config.c pulls these into axis_config_t.

#pragma once

#include "driver/gpio.h"

#ifdef __cplusplus
extern "C" {
#endif

// ---------------------------------------------------------------------------
// Stepper motor drivers (4 axes)
// Pins are ESP32 GPIO numbers (not physical D-pin labels on a specific PCB).
// ---------------------------------------------------------------------------

// Axis / joint 0`
#define BOARD_AXIS0_STEP_GPIO GPIO_NUM_25
#define BOARD_AXIS0_DIR_GPIO  GPIO_NUM_26
#define BOARD_AXIS0_EN_GPIO   GPIO_NUM_27

// Axis / joint 1
#define BOARD_AXIS1_STEP_GPIO GPIO_NUM_14
#define BOARD_AXIS1_DIR_GPIO  GPIO_NUM_21
#define BOARD_AXIS1_EN_GPIO   GPIO_NUM_13

// Axis / joint 2
#define BOARD_AXIS2_STEP_GPIO GPIO_NUM_33
#define BOARD_AXIS2_DIR_GPIO  GPIO_NUM_32
#define BOARD_AXIS2_EN_GPIO   GPIO_NUM_22

// Axis / joint 3
#define BOARD_AXIS3_STEP_GPIO GPIO_NUM_23
#define BOARD_AXIS3_DIR_GPIO  GPIO_NUM_18
#define BOARD_AXIS3_EN_GPIO   GPIO_NUM_19

// ---------------------------------------------------------------------------
// Not used by this firmware today (document future pins here)
// ---------------------------------------------------------------------------
// Examples: limit switches, estop input, SPI CS for a sensor, status LED, etc.
// #define BOARD_LIMIT_J0_MIN_GPIO  GPIO_NUM_NC
// #define BOARD_ESTOP_GPIO         GPIO_NUM_NC

#ifdef __cplusplus
}
#endif
