#pragma once
#include <stdbool.h>
#include "esp_err.h"
#include "touch_ns2009.h"

typedef struct {
    uint16_t x_min, x_max, y_min, y_max;
    uint8_t  version;     // for future-proofing
} touch_cal_t;

esp_err_t touch_cal_load(touch_cal_t *out);       // reads from NVS
esp_err_t touch_cal_save(const touch_cal_t *cal); // writes to NVS
void touch_cal_apply(touch_ns2009_t *ts, const touch_cal_t *cal);
