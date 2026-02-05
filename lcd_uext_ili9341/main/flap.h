#pragma once
#include "esp_err.h"

typedef struct {
    float opening; //0.0 closed, 1.0 fully open
} flap_state_t;

esp_err_t flap_init(void);

//set flap opening
esp_err_t flap_set_opening(float opening);

flap_state_t flap_get_state(void);
