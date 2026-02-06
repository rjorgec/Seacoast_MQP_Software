#pragma once
#include "esp_err.h"

esp_err_t loadcell_init(void);

//returns current mass in grams
esp_err_t loadcell_read_g(float *out_g);

//tare/zero
esp_err_t loadcell_tare(void);
