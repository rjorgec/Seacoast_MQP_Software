/**
 * @file dosing.h
 * @brief LEGACY ESP32-side dosing state machine.
 *
 * @deprecated This module implements a dosing loop that runs on the ESP32 and
 * commands the Pico via high-frequency flap position messages.  It pre-dates
 * the preferred architecture in which the Pico owns the closed-loop dosing
 * path (MSG_DISPENSE_SPAWN / MSG_SPAWN_STATUS).
 *
 * For new development, use sys_sequence.c which sends MSG_DISPENSE_SPAWN to
 * the Pico and receives MSG_SPAWN_STATUS asynchronously.  This file is kept
 * for reference and backward compatibility with the existing UI dosing screen.
 *
 * Do NOT add new dosing logic here.
 */

#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C"
{
#endif

    typedef enum
    {
        DOSING_STATE_IDLE = 0,
        DOSING_STATE_PRIME, // nsure flaps are open / stable
        DOSING_STATE_FAST,  // full open
        DOSING_STATE_TAPER, // start closing as approach taret
        DOSING_STATE_FINAL, // very small opening
        DOSING_STATE_DONE,
        DOSING_STATE_ABORTED,
        DOSING_STATE_ERROR,
    } dosing_state_t;

    typedef struct
    {
        float target_g;      // target dose mass in grams
        float taper_start_g; // when to begin reducing flow (grams)
        float final_band_g;  // within this band, go to FINAL (grams)
        float overshoot_g;   // allow a little overshoot before forcing stop (grams)

        // flap commands
        float fast_open;
        float taper_open;
        float final_open;

        // timing
        uint32_t min_step_ms; // minimum ms between actuator updates
        float ema_alpha;

    } dosing_cfg_t;

    typedef struct
    {
        dosing_state_t state;
        dosing_cfg_t cfg;

        float filtered_g;
        float last_g;
        uint32_t last_step_ms;

        bool running;
        esp_err_t last_err;
    } dosing_ctx_t;

    esp_err_t dosing_init(dosing_ctx_t *ctx);

    // non blocking
    esp_err_t dosing_start(dosing_ctx_t *ctx, float target_g);

    // close flaps
    esp_err_t dosing_abort(dosing_ctx_t *ctx);

    esp_err_t dosing_tick(dosing_ctx_t *ctx);

    bool dosing_is_running(const dosing_ctx_t *ctx);
    dosing_state_t dosing_get_state(const dosing_ctx_t *ctx);

#ifdef __cplusplus
}
#endif
