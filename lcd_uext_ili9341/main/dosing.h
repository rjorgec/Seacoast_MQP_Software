#pragma once
#include <stdbool.h>
#include "esp_err.h"

typedef enum {
    DOSE_IDLE = 0,
    DOSE_OPENING,
    DOSE_FULL_FLOW,
    DOSE_THROTTLING,
    DOSE_CLOSING,
    DOSE_SETTLE,
    DOSE_DONE,
    DOSE_ABORTED,
    DOSE_FAULT
} dose_state_t;

typedef struct {
    float target_g;

    //behavior tuning
    float throttle_opening;     // .g. 0.30
    float settle_band_g;        //e.g. 0.5 (stop when within band)
    float start_close_margin_g; //e.g. 3.0 (start closing early)
    float latency_s;            //effective latency for prediction (e.g. 0.20s)

    //safety
    float max_time_s;  // e.g. 20s
} dose_cfg_t;

typedef struct {
    dose_state_t state;
    float mass_g;
    float flow_gps;
    float opening;
} dose_status_t;

esp_err_t dosing_init(void);
esp_err_t dosing_start(const dose_cfg_t *cfg);
esp_err_t dosing_abort(void);
dose_status_t dosing_get_status(void);
