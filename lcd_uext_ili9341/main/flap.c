// flap.c
#include "flap.h"
#include "motor_hal.h"

#include "esp_err.h"
#include "esp_log.h"
#include "esp_timer.h"

#include <math.h>
#include <stdint.h>

static const char *TAG = "flap";

// Current logical state (0.0..1.0)
static flap_state_t s_flap = {.opening = 0.0f};

// Simple “quick” actuator control state
static bool     s_linact_running = false;
static uint16_t s_last_speed     = 0;
static uint32_t s_last_cmd_ms    = 0;

static float clamp01(float x) { return x < 0 ? 0 : (x > 1 ? 1 : x); }

static inline uint32_t now_ms(void) {
    return (uint32_t)(esp_timer_get_time() / 1000ULL);
}

// Tuning knobs for the quick version
#define FLAP_OPEN_EPS              0.03f   // treat <= this as “closed”
#define FLAP_CMD_MIN_INTERVAL_MS   150u    // rate-limit UART RPCs
#define FLAP_SPEED_EPS             80u     // avoid spam if speed change tiny (~2%)

#define FLAP_MON_LOW_TH            0u
#define FLAP_MON_HIGH_TH           4095u
#define FLAP_MON_INTERVAL_MS       50u     // current monitor polling interval on Pico

esp_err_t flap_init(void) {
    s_flap.opening = 0.0f;
    s_linact_running = false;
    s_last_speed = 0;
    s_last_cmd_ms = 0;

    // Try to stop actuator, but don’t hard-fail init if link isn’t ready yet.
    esp_err_t err = motor_linact_stop_monitor();
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "linact stop on init failed (%s) [ignored]", esp_err_to_name(err));
    }
    return ESP_OK;
}

esp_err_t flap_set_opening(float opening)
{
    opening = clamp01(opening);
    s_flap.opening = opening;

    const bool want_run = (opening > FLAP_OPEN_EPS);

    // Map 0..1 -> 0..4095 PWM “speed”
    uint16_t speed = (uint16_t)lroundf(opening * 4095.0f);
    if (speed > 4095) speed = 4095;

    // Rate-limit commands so dosing_tick() can call this often without spamming UART
    const uint32_t t = now_ms();
    if ((t - s_last_cmd_ms) < FLAP_CMD_MIN_INTERVAL_MS) {
        return ESP_OK;
    }

    // If closed: stop monitor + stop motor
    if (!want_run) {
        if (s_linact_running) {
            esp_err_t err = motor_linact_stop_monitor();
            if (err != ESP_OK) {
                ESP_LOGW(TAG, "linact stop failed (%s)", esp_err_to_name(err));
                // For safety, still consider ourselves “not running” locally.
            }
            s_linact_running = false;
            s_last_speed = 0;
            s_last_cmd_ms = t;
        }
        return ESP_OK;
    }

    // If open: start (or update) monitor/speed
    uint16_t diff = (speed > s_last_speed) ? (speed - s_last_speed) : (s_last_speed - speed);

    if (!s_linact_running || diff > FLAP_SPEED_EPS) {
        esp_err_t err = motor_linact_start_monitor(speed,
                                                   FLAP_MON_LOW_TH,
                                                   FLAP_MON_HIGH_TH,
                                                   FLAP_MON_INTERVAL_MS);
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "linact start_monitor failed (%s)", esp_err_to_name(err));
            return err;
        }
        s_linact_running = true;
        s_last_speed = speed;
        s_last_cmd_ms = t;
    }

    return ESP_OK;
}

flap_state_t flap_get_state(void) {
    return s_flap;
}
