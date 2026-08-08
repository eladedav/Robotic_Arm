// comms.c
#include "comms.h"

#include <mdns.h>

#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <ctype.h>

#include "esp_log.h"
#include "esp_err.h"
#include "nvs_flash.h"

#include "esp_netif.h"
#include "esp_event.h"
#include "esp_wifi.h"

#include "esp_http_server.h"

#include "config.h"
#include "motion.h"
#include "safety.h"
#include "hal_gpio.h"

#include "esp_timer.h"

static const char *TAG = "comms";

//-------------------- Helper functions --------------------
static void mdns_start(void)
{
    static bool started = false;
    if (started) return;

    ESP_ERROR_CHECK(mdns_init());
    ESP_ERROR_CHECK(mdns_hostname_set("esp-arm"));  // -> esp-arm.local
    ESP_ERROR_CHECK(mdns_instance_name_set("Robotic Arm ESP"));

    // Advertise HTTP service on port 80
    ESP_ERROR_CHECK(mdns_service_add(NULL, "_http", "_tcp", 80, NULL, 0));

    started = true;
    ESP_LOGI(TAG, "mDNS started: http://esp-arm.local/");
}

//--------------------------------------------------------



static httpd_handle_t s_http = NULL;
static EventGroupHandle_t s_wifi_event_group = NULL;

#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT      BIT1

static int s_retry_num = 0;
#ifndef WIFI_MAX_RETRY
#define WIFI_MAX_RETRY 10
#endif

static void wifi_event_handler(void *arg,
                               esp_event_base_t event_base,
                               int32_t event_id,
                               void *event_data)
{
    (void)arg;
    (void)event_data;

    /* ---------------- WIFI EVENTS ---------------- */
    if (event_base == WIFI_EVENT) {

        switch (event_id) {

        case WIFI_EVENT_STA_START:
            ESP_LOGI(TAG, "Wi-Fi start, connecting...");
            esp_wifi_connect();
            break;

        case WIFI_EVENT_STA_DISCONNECTED:
            // On link loss, do a controlled motion stop but keep drivers enabled
            // so joints can hold position under load.
            motion_stop_all();
            if (safety_get_state() != SYS_ESTOP) {
                hal_gpio_enable_all();
            }
            if (s_retry_num < WIFI_MAX_RETRY) {
                s_retry_num++;
                ESP_LOGW(TAG,
                         "Wi-Fi disconnected, retrying (%d/%d)",
                         s_retry_num,
                         WIFI_MAX_RETRY);
                esp_wifi_connect();
            } else {
                ESP_LOGE(TAG, "Wi-Fi failed after %d retries", WIFI_MAX_RETRY);
                xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
            }
            break;

        default:
            break;
        }
    }

    /* ---------------- IP EVENTS ---------------- */
    else if (event_base == IP_EVENT) {

        if (event_id == IP_EVENT_STA_GOT_IP) {

            s_retry_num = 0;

            xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);

            ESP_LOGI(TAG, "Wi-Fi connected (IP acquired)");

            /* Start mDNS once network stack is ready */
            mdns_start();
        }
    }
}

static void wifi_init_sta(void)
{
    s_wifi_event_group = xEventGroupCreate();

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    ESP_ERROR_CHECK(esp_event_handler_instance_register(
        WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL, NULL));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(
        IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler, NULL, NULL));

wifi_config_t wifi_config = {0};

snprintf((char*)wifi_config.sta.ssid,
         sizeof(wifi_config.sta.ssid),
         "%s",
         CONFIG_ROBOT_WIFI_SSID);

snprintf((char*)wifi_config.sta.password,
         sizeof(wifi_config.sta.password),
         "%s",
         CONFIG_ROBOT_WIFI_PASSWORD);

    wifi_config.sta.threshold.authmode = WIFI_AUTH_WPA2_PSK;

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "Wi-Fi STA start: SSID=%s", CONFIG_ROBOT_WIFI_SSID);

    EventBits_t bits = xEventGroupWaitBits(
        s_wifi_event_group,
        WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
        pdFALSE,
        pdFALSE,
        pdMS_TO_TICKS(15000)  // wait up to 15s
    );

    if (bits & WIFI_CONNECTED_BIT) {
        ESP_LOGI(TAG, "Wi-Fi connected");
    } else {
        ESP_LOGW(TAG, "Wi-Fi not connected (continuing anyway)");
        // We continue running: user can still flash/debug; server might start later if IP arrives.
    }
}

// -------------------- HTTP helpers --------------------

static esp_err_t send_text(httpd_req_t *req, const char *text, int code)
{
    httpd_resp_set_status(req, code == 200 ? "200 OK" :
                               code == 400 ? "400 Bad Request" :
                               code == 403 ? "403 Forbidden" :
                               code == 404 ? "404 Not Found" : "500 Internal Server Error");
    httpd_resp_set_type(req, "text/plain");
    return httpd_resp_send(req, text, HTTPD_RESP_USE_STRLEN);
}

static esp_err_t send_json(httpd_req_t *req, const char *json)
{
    httpd_resp_set_type(req, "application/json");
    return httpd_resp_send(req, json, HTTPD_RESP_USE_STRLEN);
}

static bool query_get_i32(httpd_req_t *req, const char *key, int32_t *out)
{
    char qs[128];
    if (httpd_req_get_url_query_str(req, qs, sizeof(qs)) != ESP_OK) return false;

    char val[32];
    if (httpd_query_key_value(qs, key, val, sizeof(val)) != ESP_OK) return false;

    *out = (int32_t)strtol(val, NULL, 10);
    return true;
}

static bool query_get_f32(httpd_req_t *req, const char *key, float *out)
{
    char qs[256];
    if (httpd_req_get_url_query_str(req, qs, sizeof(qs)) != ESP_OK) return false;

    char val[32];
    if (httpd_query_key_value(qs, key, val, sizeof(val)) != ESP_OK) return false;

    *out = strtof(val, NULL);
    return true;
}

// -------------------- Handlers --------------------

// GET /rotate?joint=0&dir=R&deg=5&rpm=2.0
// Single-axis relative rotate with requested output-shaft RPM.
static esp_err_t rotate_handler(httpd_req_t *req)
{
    int32_t joint = -1;
    if (!query_get_i32(req, "joint", &joint)) {
        return send_text(req, "Missing joint", 400);
    }
    if (joint < 0 || joint >= CONFIG_AXES) {
        return send_text(req, "Invalid joint", 400);
    }

    char qs[200];
    if (httpd_req_get_url_query_str(req, qs, sizeof(qs)) != ESP_OK) {
        return send_text(req, "Missing query", 400);
    }

    char dirval[8];
    if (httpd_query_key_value(qs, "dir", dirval, sizeof(dirval)) != ESP_OK) {
        return send_text(req, "Missing dir (R or L)", 400);
    }

    char degstr[32];
    float deg = 0.f;
    if (httpd_query_key_value(qs, "deg", degstr, sizeof(degstr)) == ESP_OK) {
        deg = strtof(degstr, NULL);
    } else if (httpd_query_key_value(qs, "degree", degstr, sizeof(degstr)) == ESP_OK) {
        deg = strtof(degstr, NULL);
    } else {
        return send_text(req, "Missing deg or degree", 400);
    }

    char rpmstr[32];
    float rpm = 0.f;
    if (httpd_query_key_value(qs, "rpm", rpmstr, sizeof(rpmstr)) == ESP_OK) {
        rpm = strtof(rpmstr, NULL);
    } else {
        return send_text(req, "Missing rpm", 400);
    }

    char d = dirval[0];
    if (!isalpha((unsigned char)d)) {
        return send_text(req, "Invalid dir (use R or L)", 400);
    }

    safety_note_command_rx();

    if (!safety_motion_allowed()) {
        return send_text(req, "Motion not allowed (safety state)", 403);
    }

    if (!motion_rotate_degrees((uint8_t)joint, d, deg, rpm)) {
        return send_text(req, "Rotate rejected (need deg>0, rpm>0, dir R/L)", 400);
    }

    return send_text(req, "OK", 200);
}

// GET /home?joint=0
static esp_err_t home_handler(httpd_req_t *req)
{
    int32_t joint = -1;
    if (!query_get_i32(req, "joint", &joint)) {
        return send_text(req, "Missing joint", 400);
    }
    if (joint < 0 || joint >= CONFIG_AXES) {
        return send_text(req, "Invalid joint", 400);
    }

    safety_note_command_rx();
    if (!motion_home_axis((uint8_t)joint)) {
        return send_text(req, "Home failed", 400);
    }
    return send_text(req, "OK", 200);
}

// GET /setpos?joint=0&deg=0   or   /setpos?joint=0&steps=1234
static esp_err_t setpos_handler(httpd_req_t *req)
{
    int32_t joint = -1;
    if (!query_get_i32(req, "joint", &joint)) {
        return send_text(req, "Missing joint", 400);
    }
    if (joint < 0 || joint >= CONFIG_AXES) {
        return send_text(req, "Invalid joint", 400);
    }

    char qs[200];
    if (httpd_req_get_url_query_str(req, qs, sizeof(qs)) != ESP_OK) {
        return send_text(req, "Missing query", 400);
    }

    safety_note_command_rx();

    char degstr[32];
    if (httpd_query_key_value(qs, "deg", degstr, sizeof(degstr)) == ESP_OK) {
        float deg = strtof(degstr, NULL);
        if (!motion_set_position_degrees((uint8_t)joint, deg)) {
            return send_text(req, "setpos deg failed", 400);
        }
        return send_text(req, "OK", 200);
    }

    char stepstr[32];
    if (httpd_query_key_value(qs, "steps", stepstr, sizeof(stepstr)) == ESP_OK) {
        int32_t steps = (int32_t)strtol(stepstr, NULL, 10);
        if (!motion_set_position_steps((uint8_t)joint, steps)) {
            return send_text(req, "setpos steps failed", 400);
        }
        return send_text(req, "OK", 200);
    }

    return send_text(req, "Missing deg or steps", 400);
}

// GET /jog_start?joint=0&dir=R&rpm=15
static esp_err_t jog_start_handler(httpd_req_t *req)
{
    int32_t joint = -1;
    if (!query_get_i32(req, "joint", &joint)) {
        return send_text(req, "Missing joint", 400);
    }
    if (joint < 0 || joint >= CONFIG_AXES) {
        return send_text(req, "Invalid joint", 400);
    }

    char qs[200];
    if (httpd_req_get_url_query_str(req, qs, sizeof(qs)) != ESP_OK) {
        return send_text(req, "Missing query", 400);
    }

    char dirval[8];
    if (httpd_query_key_value(qs, "dir", dirval, sizeof(dirval)) != ESP_OK) {
        return send_text(req, "Missing dir (R or L)", 400);
    }

    float rpm = 0.f;
    if (!query_get_f32(req, "rpm", &rpm)) {
        return send_text(req, "Missing rpm", 400);
    }

    safety_note_command_rx();
    if (!safety_motion_allowed()) {
        return send_text(req, "Motion not allowed (safety state)", 403);
    }

    if (!motion_jog_start((uint8_t)joint, dirval[0], rpm)) {
        return send_text(req, "Jog start failed (need rpm>0, dir R/L)", 400);
    }
    return send_text(req, "OK", 200);
}

// GET /jog_stop?joint=0   (joint optional: without it stops all axes)
static esp_err_t jog_stop_handler(httpd_req_t *req)
{
    int32_t joint = -1;
    bool has_joint = query_get_i32(req, "joint", &joint);

    safety_note_command_rx();

    if (!has_joint) {
        motion_stop_all();
        return send_text(req, "OK", 200);
    }
    if (joint < 0 || joint >= CONFIG_AXES) {
        return send_text(req, "Invalid joint", 400);
    }

    motion_stop_axis((uint8_t)joint);
    return send_text(req, "OK", 200);
}

// GET /movej?rpm=10&j0=5&j1=-3&j2=0&j3=0
// j0..j3 are relative move degrees per axis.
static esp_err_t movej_handler(httpd_req_t *req)
{
    float rpm = 0.f;
    if (!query_get_f32(req, "rpm", &rpm)) {
        return send_text(req, "Missing rpm", 400);
    }

    char qs[256];
    if (httpd_req_get_url_query_str(req, qs, sizeof(qs)) != ESP_OK) {
        return send_text(req, "Missing query", 400);
    }

    float delta_deg[CONFIG_AXES] = {0};
    bool any = false;
    for (uint8_t i = 0; i < CONFIG_AXES; i++) {
        char key[8];
        snprintf(key, sizeof(key), "j%u", (unsigned)i);

        char val[32];
        if (httpd_query_key_value(qs, key, val, sizeof(val)) == ESP_OK) {
            delta_deg[i] = strtof(val, NULL);
            if (delta_deg[i] != 0.0f) {
                any = true;
            }
        }
    }
    if (!any) {
        return send_text(req, "Missing j0..j3 deltas", 400);
    }

    safety_note_command_rx();
    if (!safety_motion_allowed()) {
        return send_text(req, "Motion not allowed (safety state)", 403);
    }

    if (!motion_movej_relative_degrees(delta_deg, rpm)) {
        return send_text(req, "movej rejected (need rpm>0, non-zero deltas)", 400);
    }
    return send_text(req, "OK", 200);
}

// GET /cmd?axis=0&steps=1234
static esp_err_t cmd_handler(httpd_req_t *req)
{
    int32_t axis = -1, steps = 0;

    if (!query_get_i32(req, "axis", &axis) || !query_get_i32(req, "steps", &steps)) {
        return send_text(req, "Missing axis/steps", 400);
    }
    if (axis < 0 || axis >= CONFIG_AXES) {
        return send_text(req, "Invalid axis", 400);
    }

    // Note command reception for watchdog policy
    safety_note_command_rx();

    // Gate motion by safety state
    if (!safety_motion_allowed()) {
        return send_text(req, "Motion not allowed (safety state)", 403);
    }

    // Optional: enforce soft limits here using config if you want steps-limits
    // (Right now config exposes angle limits; we can add step limits later.)

    char qs[160];
    float rpm = 2.0f; // default command speed if rpm is omitted
    if (httpd_req_get_url_query_str(req, qs, sizeof(qs)) == ESP_OK) {
        char rpmstr[32];
        if (httpd_query_key_value(qs, "rpm", rpmstr, sizeof(rpmstr)) == ESP_OK) {
            rpm = strtof(rpmstr, NULL);
        }
    }

    if (!motion_set_target_steps_rpm((uint8_t)axis, steps, rpm)) {
        return send_text(req, "Failed to set target (check rpm>0)", 400);
    }

    return send_text(req, "OK", 200);
}

// GET /stop  (controlled stop)
static esp_err_t stop_handler(httpd_req_t *req)
{
    safety_note_command_rx();
    motion_stop_all();
    return send_text(req, "STOPPING", 200);
}

// GET /estop (hard stop)
static esp_err_t estop_handler(httpd_req_t *req)
{
    safety_note_command_rx();
    safety_estop_trigger(); // stops motion + disables drivers (per safety.c)
    return send_text(req, "ESTOP", 200);
}

// GET /state
static esp_err_t state_handler(httpd_req_t *req)
{
    // Build a small JSON without extra libs
    // Example:
    // {"state":1,"faults":0,"pos_deg":[0.0,0.0,0.0,0.0],"moving":false}
    char buf[320];

    system_state_t st = safety_get_state();
    uint32_t faults = safety_get_fault_flags();

    int32_t p0_steps = motion_get_position_steps(0);
    int32_t p1_steps = motion_get_position_steps(1);
    int32_t p2_steps = motion_get_position_steps(2);
    int32_t p3_steps = motion_get_position_steps(3);

    float p0_deg = config_steps_to_angle_deg(0, p0_steps);
    float p1_deg = config_steps_to_angle_deg(1, p1_steps);
    float p2_deg = config_steps_to_angle_deg(2, p2_steps);
    float p3_deg = config_steps_to_angle_deg(3, p3_steps);

    bool moving = motion_is_any_moving();

    snprintf(buf, sizeof(buf),
             "{\"state\":%d,\"faults\":%lu,\"pos_deg\":[%.3f,%.3f,%.3f,%.3f],\"moving\":%s}",
             (int)st,
             (unsigned long)faults,
             (double)p0_deg, (double)p1_deg, (double)p2_deg, (double)p3_deg,
             moving ? "true" : "false");

    return send_json(req, buf);
}

// GET /uptime
static esp_err_t uptime_handler(httpd_req_t *req)
{
    // microseconds since boot
    int64_t us = esp_timer_get_time();

    uint64_t ms  = (uint64_t)(us / 1000);
    uint64_t sec = ms / 1000;
    uint64_t min = sec / 60;
    uint64_t hr  = min / 60;
    uint64_t day = hr  / 24;

    char buf[128];

    snprintf(buf, sizeof(buf),
             "{\"uptime_ms\":%llu,\"uptime_s\":%llu,"
             "\"d\":%llu,\"h\":%llu,\"m\":%llu,\"s\":%llu}",
             (unsigned long long)ms,
             (unsigned long long)sec,
             (unsigned long long)day,
             (unsigned long long)(hr  % 24),
             (unsigned long long)(min % 60),
             (unsigned long long)(sec % 60));

    return send_json(req, buf);
}
// -------------------- HTTP server start --------------------

static void http_server_start(void)
{
    httpd_config_t cfg = HTTPD_DEFAULT_CONFIG();
    cfg.server_port = 80;
    // We register many endpoints; default handler slots are not enough.
    // Keep this above the number of registered URIs to avoid ESP_ERR_HTTPD_HANDLERS_FULL.
    cfg.max_uri_handlers = 16;

    if (httpd_start(&s_http, &cfg) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start HTTP server");
        s_http = NULL;
        return;
    }

    httpd_uri_t uri_rotate = {
        .uri      = "/rotate",
        .method   = HTTP_GET,
        .handler  = rotate_handler,
        .user_ctx = NULL
    };
    httpd_uri_t uri_cmd = {
        .uri      = "/cmd",
        .method   = HTTP_GET,
        .handler  = cmd_handler,
        .user_ctx = NULL
    };
    httpd_uri_t uri_stop = {
        .uri      = "/stop",
        .method   = HTTP_GET,
        .handler  = stop_handler,
        .user_ctx = NULL
    };
    httpd_uri_t uri_estop = {
        .uri      = "/estop",
        .method   = HTTP_GET,
        .handler  = estop_handler,
        .user_ctx = NULL
    };
    httpd_uri_t uri_state = {
        .uri      = "/state",
        .method   = HTTP_GET,
        .handler  = state_handler,
        .user_ctx = NULL
    };
    httpd_uri_t uri_uptime = {
        .uri      = "/uptime",
        .method   = HTTP_GET,
        .handler  = uptime_handler,
        .user_ctx = NULL
    };
    httpd_uri_t uri_home = {
        .uri      = "/home",
        .method   = HTTP_GET,
        .handler  = home_handler,
        .user_ctx = NULL
    };
    httpd_uri_t uri_setpos = {
        .uri      = "/setpos",
        .method   = HTTP_GET,
        .handler  = setpos_handler,
        .user_ctx = NULL
    };
    httpd_uri_t uri_jog_start = {
        .uri      = "/jog_start",
        .method   = HTTP_GET,
        .handler  = jog_start_handler,
        .user_ctx = NULL
    };
    httpd_uri_t uri_jog_stop = {
        .uri      = "/jog_stop",
        .method   = HTTP_GET,
        .handler  = jog_stop_handler,
        .user_ctx = NULL
    };
    httpd_uri_t uri_movej = {
        .uri      = "/movej",
        .method   = HTTP_GET,
        .handler  = movej_handler,
        .user_ctx = NULL
    };
    
    ESP_ERROR_CHECK(httpd_register_uri_handler(s_http, &uri_rotate));
    ESP_ERROR_CHECK(httpd_register_uri_handler(s_http, &uri_cmd));
    ESP_ERROR_CHECK(httpd_register_uri_handler(s_http, &uri_stop));
    ESP_ERROR_CHECK(httpd_register_uri_handler(s_http, &uri_estop));
    ESP_ERROR_CHECK(httpd_register_uri_handler(s_http, &uri_state));
    ESP_ERROR_CHECK(httpd_register_uri_handler(s_http, &uri_uptime));
    ESP_ERROR_CHECK(httpd_register_uri_handler(s_http, &uri_home));
    ESP_ERROR_CHECK(httpd_register_uri_handler(s_http, &uri_setpos));
    ESP_ERROR_CHECK(httpd_register_uri_handler(s_http, &uri_jog_start));
    ESP_ERROR_CHECK(httpd_register_uri_handler(s_http, &uri_jog_stop));
    ESP_ERROR_CHECK(httpd_register_uri_handler(s_http, &uri_movej));

    ESP_LOGI(TAG, "HTTP server started: /rotate /cmd /home /setpos /jog_start /jog_stop /movej /stop /estop /state /uptime");
}

// -------------------- Public API --------------------

void comms_init(void)
{
    // NVS is required for Wi-Fi
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ESP_ERROR_CHECK(nvs_flash_init());
    } else {
        ESP_ERROR_CHECK(ret);
    }

    wifi_init_sta();
    http_server_start();
}

void comms_start(void)
{
    // Not needed in this minimal version (init starts everything).
}
