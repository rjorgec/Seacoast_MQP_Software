#include "loadcell.h"
#include "flap.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static float s_mass_g = 0.0f;

esp_err_t loadcell_init(void) { s_mass_g = 0.0f; return ESP_OK; }
esp_err_t loadcell_tare(void) { s_mass_g = 0.0f; return ESP_OK; }

esp_err_t loadcell_read_g(float *out_g)
{
    if (!out_g) return ESP_ERR_INVALID_ARG;

    //Simulated flow: max 12 g/s at full open (TUNE)
    const float max_flow_gps = 12.0f;
    float opening = flap_get_state().opening;

    //advance mass based on time between reads (assuming ~50Hz for now)
    const float dt = 0.02f;
    s_mass_g += (opening * max_flow_gps) * dt;

    *out_g = s_mass_g;
    return ESP_OK;
}
