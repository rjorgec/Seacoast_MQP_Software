#include "flap.h"
#include <math.h>
//doesn't really do anything yet
static flap_state_t s_flap = {.opening = 0.0f};

static float clamp01(float x){ return x < 0 ? 0 : (x > 1 ? 1 : x); }

esp_err_t flap_init(void) { s_flap.opening = 0.0f; return ESP_OK; }

esp_err_t flap_set_opening(float opening)
{
    s_flap.opening = clamp01(opening);
    return ESP_OK;
}

flap_state_t flap_get_state(void) { return s_flap; }
