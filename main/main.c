// main.c
// Top-level application entry point
// Responsible only for system initialization and task startup

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

// Project modules
#include "config.h"
#include "hal_gpio.h"
#include "motion.h"
#include "safety.h"
#include "comms.h"

static const char *TAG = "app";

void app_main(void)
{
    ESP_LOGI(TAG, "Booting robotic arm firmware");

    /* -------------------------------------------------
     * 1. Load configuration
     * ------------------------------------------------- */
    config_init();

    /* -------------------------------------------------
     * 2. Initialize hardware to SAFE state
     * ------------------------------------------------- */
    hal_gpio_init();          // configure STEP/DIR/EN pins
    hal_gpio_disable_all();   // drivers OFF at boot

    /* -------------------------------------------------
     * 3. Initialize core subsystems (no motion yet)
     * ------------------------------------------------- */
    safety_init();            // clear faults, motion not allowed
    motion_init();            // init axis states, planners, timers
    comms_init();             // Wi-Fi + web server (async)

    /* -------------------------------------------------
     * 4. Start runtime tasks
     * ------------------------------------------------- */
    motion_start();           // high-priority motion task / timer
    safety_start();           // safety monitoring task
    comms_start();            // optional (if comms uses a task)

    /* -------------------------------------------------
     * 5. Transition to READY state
     * ------------------------------------------------- */
    safety_set_ready();       // motion now permitted
    hal_gpio_enable_all();    // enable motor drivers (idle)

    ESP_LOGI(TAG, "System READY");

    /* -------------------------------------------------
     * app_main ends here
     * FreeRTOS tasks keep the system alive
     * ------------------------------------------------- */
}
