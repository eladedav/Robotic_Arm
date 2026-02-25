// comms.c
#include "comms.h"

#include <string.h>
#include <stdlib.h>
#include <stdio.h>

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

static httpd_handle_t s_http = NULL;
static EventGroupHandle_t s_wifi_event_group = NULL;

#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT      BIT1

static int s_retry_num = 0;
#ifndef WIFI_MAX_RETRY
#define WIFI_MAX_RETRY 10
#endif

static void wifi_event_handler(void* arg,
                               esp_event_base_t event_base,
                               int32_t event_id,
                               void* event_data)
{
    (void)arg;
    (void)event_data;

    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    }
    else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        if (s_retry_num < WIFI_MAX_RETRY) {
            s_retry_num++;
            ESP_LOGW(TAG, "Wi-Fi disconnected, retrying (%d/%d)", s_retry_num, WIFI_MAX_RETRY);
            esp_wifi_connect();
        } else {
            xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
        }
    }
    else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        s_retry_num = 0;
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
        ESP_LOGI(TAG, "Wi-Fi connected (got IP)");
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

// -------------------- Handlers --------------------

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

    if (!motion_set_target_steps((uint8_t)axis, steps)) {
        return send_text(req, "Failed to set target", 500);
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
    // {"state":1,"faults":0,"pos_steps":[0,0,0,0],"moving":false}
    char buf[256];

    system_state_t st = safety_get_state();
    uint32_t faults = safety_get_fault_flags();

    int32_t p0 = motion_get_position_steps(0);
    int32_t p1 = motion_get_position_steps(1);
    int32_t p2 = motion_get_position_steps(2);
    int32_t p3 = motion_get_position_steps(3);

    bool moving = motion_is_any_moving();

    snprintf(buf, sizeof(buf),
             "{\"state\":%d,\"faults\":%lu,\"pos_steps\":[%ld,%ld,%ld,%ld],\"moving\":%s}",
             (int)st,
             (unsigned long)faults,
             (long)p0, (long)p1, (long)p2, (long)p3,
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

    if (httpd_start(&s_http, &cfg) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start HTTP server");
        s_http = NULL;
        return;
    }

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
    
    ESP_ERROR_CHECK(httpd_register_uri_handler(s_http, &uri_cmd));
    ESP_ERROR_CHECK(httpd_register_uri_handler(s_http, &uri_stop));
    ESP_ERROR_CHECK(httpd_register_uri_handler(s_http, &uri_estop));
    ESP_ERROR_CHECK(httpd_register_uri_handler(s_http, &uri_state));
    ESP_ERROR_CHECK(httpd_register_uri_handler(s_http, &uri_uptime));

    ESP_LOGI(TAG, "HTTP server started: /cmd /stop /estop /state /uptime");
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
