#include "pulse_gen.h"

#include "config.h"

#include "driver/gpio.h"
#include "driver/gptimer.h"
#include "esp_log.h"

typedef struct {
    gptimer_handle_t timer;
    gpio_num_t step_pin;
    volatile bool running;
    volatile bool level_high;
    volatile uint32_t pulse_count;
    uint32_t step_hz;
} pulse_axis_t;

static const char *TAG = "pulse_gen";
static pulse_axis_t s_axis[CONFIG_AXES];
static bool s_init_done = false;

static inline bool axis_valid(uint8_t axis)
{
    return axis < CONFIG_AXES;
}

static bool IRAM_ATTR pulse_alarm_cb(gptimer_handle_t timer,
                                     const gptimer_alarm_event_data_t *edata,
                                     void *user_ctx)
{
    (void)timer;
    (void)edata;
    pulse_axis_t *a = (pulse_axis_t *)user_ctx;

    a->level_high = !a->level_high;
    gpio_set_level(a->step_pin, a->level_high ? 1 : 0);
    if (a->level_high) {
        // Count each generated STEP edge.
        a->pulse_count++;
    }
    return false;
}

esp_err_t pulse_gen_init(void)
{
    if (s_init_done) {
        return ESP_OK;
    }

    for (uint8_t i = 0; i < CONFIG_AXES; i++) {
        const axis_config_t *cfg = config_axis(i);
        if (!cfg) {
            ESP_LOGE(TAG, "config_axis(%u) is NULL", (unsigned)i);
            return ESP_ERR_INVALID_STATE;
        }

        s_axis[i].step_pin = cfg->step_pin;
        s_axis[i].running = false;
        s_axis[i].level_high = false;
        s_axis[i].pulse_count = 0;
        s_axis[i].step_hz = 0;

        gpio_set_direction(cfg->step_pin, GPIO_MODE_OUTPUT);
        gpio_set_level(cfg->step_pin, 0);

        gptimer_config_t tcfg = {
            .clk_src = GPTIMER_CLK_SRC_DEFAULT,
            .direction = GPTIMER_COUNT_UP,
            .resolution_hz = 1000000, // 1 tick = 1 us
        };

        esp_err_t err = gptimer_new_timer(&tcfg, &s_axis[i].timer);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "Axis %u gptimer_new_timer failed (%s)", (unsigned)i, esp_err_to_name(err));
            return err;
        }

        gptimer_event_callbacks_t cbs = {
            .on_alarm = pulse_alarm_cb,
        };
        err = gptimer_register_event_callbacks(s_axis[i].timer, &cbs, &s_axis[i]);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "Axis %u register callbacks failed (%s)", (unsigned)i, esp_err_to_name(err));
            return err;
        }

        err = gptimer_enable(s_axis[i].timer);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "Axis %u gptimer_enable failed (%s)", (unsigned)i, esp_err_to_name(err));
            return err;
        }
    }

    s_init_done = true;
    ESP_LOGI(TAG, "Initialized %u axis pulse generators", (unsigned)CONFIG_AXES);
    return ESP_OK;
}

esp_err_t pulse_gen_start(uint8_t axis, uint32_t step_hz)
{
    if (!axis_valid(axis)) {
        return ESP_ERR_INVALID_ARG;
    }
    if (!s_init_done) {
        return ESP_ERR_INVALID_STATE;
    }
    if (step_hz == 0U) {
        return ESP_ERR_INVALID_ARG;
    }

    pulse_axis_t *a = &s_axis[axis];

    if (a->running) {
        ESP_ERROR_CHECK_WITHOUT_ABORT(gptimer_stop(a->timer));
        a->running = false;
    }

    // full period = 1e6 / step_hz us; alarm toggles every half period
    uint64_t half_period_us = ((uint64_t)500000 + (step_hz / 2U)) / (uint64_t)step_hz;
    if (half_period_us == 0U) {
        half_period_us = 1U;
    }

    a->step_hz = step_hz;
    a->level_high = false;
    gpio_set_level(a->step_pin, 0);

    esp_err_t err = gptimer_set_raw_count(a->timer, 0);
    if (err != ESP_OK) {
        return err;
    }

    gptimer_alarm_config_t alarm = {
        .alarm_count = half_period_us,
        .reload_count = 0,
        .flags.auto_reload_on_alarm = true,
    };
    err = gptimer_set_alarm_action(a->timer, &alarm);
    if (err != ESP_OK) {
        return err;
    }

    err = gptimer_start(a->timer);
    if (err != ESP_OK) {
        return err;
    }

    a->running = true;
    ESP_LOGI(TAG, "Axis %u pulse start: %lu Hz", (unsigned)axis, (unsigned long)step_hz);
    return ESP_OK;
}

esp_err_t pulse_gen_stop(uint8_t axis)
{
    if (!axis_valid(axis)) {
        return ESP_ERR_INVALID_ARG;
    }
    if (!s_init_done) {
        return ESP_ERR_INVALID_STATE;
    }

    pulse_axis_t *a = &s_axis[axis];
    if (a->running) {
        esp_err_t err = gptimer_stop(a->timer);
        if (err != ESP_OK) {
            return err;
        }
    }

    a->running = false;
    a->level_high = false;
    a->step_hz = 0;
    gpio_set_level(a->step_pin, 0);
    return ESP_OK;
}

bool pulse_gen_is_running(uint8_t axis)
{
    if (!axis_valid(axis)) {
        return false;
    }
    return s_axis[axis].running;
}

uint32_t pulse_gen_get_pulse_count(uint8_t axis)
{
    if (!axis_valid(axis)) {
        return 0;
    }
    return s_axis[axis].pulse_count;
}
