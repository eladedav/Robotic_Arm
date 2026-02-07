// hal_gpio.c
// Hardware Abstraction Layer for GPIO access (stepper drivers)

#include "hal_gpio.h"

#include "config.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_rom_sys.h"   // esp_rom_delay_us()

static const char *TAG = "hal_gpio";

// Minimum STEP pulse width depends on your driver.
// DM556-class drivers typically accept a few microseconds; use a conservative value.
#ifndef HAL_STEP_PULSE_US
#define HAL_STEP_PULSE_US 3
#endif

static inline bool axis_valid(uint8_t axis)
{
    return axis < CONFIG_AXES;
}

static inline int en_level_for(bool enable, en_polarity_t pol)
{
    // Returns the GPIO level to write on EN pin for requested enable/disable
    if (pol == EN_ACTIVE_LOW) {
        return enable ? 0 : 1;
    } else { // EN_ACTIVE_HIGH
        return enable ? 1 : 0;
    }
}

void hal_gpio_init(void)
{
    // Configure pins for all axes and force a SAFE state (drivers disabled, STEP low)
    for (uint8_t i = 0; i < CONFIG_AXES; i++) {
        const axis_config_t *cfg = config_axis(i);
        if (!cfg) {
            ESP_LOGE(TAG, "config_axis(%u) returned NULL", (unsigned)i);
            continue;
        }

        // Reset and set output mode
        gpio_reset_pin(cfg->step_pin);
        gpio_reset_pin(cfg->dir_pin);
        gpio_reset_pin(cfg->en_pin);

        gpio_set_direction(cfg->step_pin, GPIO_MODE_OUTPUT);
        gpio_set_direction(cfg->dir_pin,  GPIO_MODE_OUTPUT);
        gpio_set_direction(cfg->en_pin,   GPIO_MODE_OUTPUT);

        // Safe defaults
        gpio_set_level(cfg->step_pin, 0);
        gpio_set_level(cfg->dir_pin,  0);
        gpio_set_level(cfg->en_pin,   en_level_for(false, cfg->en_polarity)); // disabled
    }

    ESP_LOGI(TAG, "HAL GPIO initialized (drivers disabled)");
}

void hal_gpio_enable_all(void)
{
    for (uint8_t i = 0; i < CONFIG_AXES; i++) {
        hal_gpio_enable_axis(i, true);
    }
}

void hal_gpio_disable_all(void)
{
    for (uint8_t i = 0; i < CONFIG_AXES; i++) {
        hal_gpio_enable_axis(i, false);
    }
}

void hal_gpio_enable_axis(uint8_t axis, bool enable)
{
    if (!axis_valid(axis)) {
        ESP_LOGW(TAG, "enable_axis: invalid axis %u", (unsigned)axis);
        return;
    }

    const axis_config_t *cfg = config_axis(axis);
    if (!cfg) {
        ESP_LOGE(TAG, "enable_axis: config_axis(%u) NULL", (unsigned)axis);
        return;
    }

    gpio_set_level(cfg->en_pin, en_level_for(enable, cfg->en_polarity));
}

void hal_gpio_set_dir(uint8_t axis, bool dir)
{
    if (!axis_valid(axis)) {
        ESP_LOGW(TAG, "set_dir: invalid axis %u", (unsigned)axis);
        return;
    }

    const axis_config_t *cfg = config_axis(axis);
    if (!cfg) {
        ESP_LOGE(TAG, "set_dir: config_axis(%u) NULL", (unsigned)axis);
        return;
    }

    gpio_set_level(cfg->dir_pin, dir ? 1 : 0);
}

void hal_gpio_step_pulse(uint8_t axis)
{
    if (!axis_valid(axis)) {
        ESP_LOGW(TAG, "step_pulse: invalid axis %u", (unsigned)axis);
        return;
    }

    const axis_config_t *cfg = config_axis(axis);
    if (!cfg) {
        ESP_LOGE(TAG, "step_pulse: config_axis(%u) NULL", (unsigned)axis);
        return;
    }

    // Generate one STEP pulse: HIGH -> short hold -> LOW.
    // Timing/spacing between pulses is the responsibility of the motion module.
    gpio_set_level(cfg->step_pin, 1);
    esp_rom_delay_us(HAL_STEP_PULSE_US);
    gpio_set_level(cfg->step_pin, 0);
}
