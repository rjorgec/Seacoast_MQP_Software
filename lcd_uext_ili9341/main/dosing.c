#include "dosing.h"
#include "flap.h"
#include "loadcell.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include <math.h>

static const char *TAG = "dosing";

static TaskHandle_t s_task;
static dose_cfg_t s_cfg;
static volatile bool s_running = false;
static volatile bool s_abort = false;
static dose_status_t s_status = {.state = DOSE_IDLE};

static float clamp01(float x){ return x < 0 ? 0 : (x > 1 ? 1 : x); }

// simple IIR lowpass for flow estimate
static float iir(float prev, float x, float alpha){ return prev + alpha * (x - prev); }

static void dosing_task(void *arg)
{
    (void)arg;
    const TickType_t period = pdMS_TO_TICKS(20); // 50 Hz loop
    TickType_t last_wake = xTaskGetTickCount();

    float m_prev = 0.0f;
    float flow_f = 0.0f;
    float t_elapsed = 0.0f;

    s_status.state = DOSE_OPENING;
    ESP_LOGI(TAG, "Dose start: target=%.2fg", (double)s_cfg.target_g);

    // Open fully to start
    flap_set_opening(1.0f);

    while (s_running) {
        vTaskDelayUntil(&last_wake, period);
        t_elapsed += 0.02f;

        if (s_abort) {
            flap_set_opening(0.0f);
            s_status.state = DOSE_ABORTED;
            break;
        }

        float m = 0.0f;
        if (loadcell_read_g(&m) != ESP_OK) {
            s_status.state = DOSE_FAULT;
            break;
        }

        // flow estimate
        float dm = m - m_prev;
        float flow = dm / 0.02f;            // g/s
        flow_f = iir(flow_f, flow, 0.15f);  // filtered flow
        m_prev = m;

        // prediction for overshoot
        float m_pred = m + flow_f * s_cfg.latency_s;

        // state machine
        switch (s_status.state) {
            case DOSE_OPENING:
                s_status.state = DOSE_FULL_FLOW;
                break;

            case DOSE_FULL_FLOW:
                // start throttling early
                if (m_pred >= (s_cfg.target_g - s_cfg.start_close_margin_g)) {
                    flap_set_opening(clamp01(s_cfg.throttle_opening));
                    s_status.state = DOSE_THROTTLING;
                }
                break;

            case DOSE_THROTTLING:
                // final close when predicted to reach target
                if (m_pred >= s_cfg.target_g) {
                    flap_set_opening(0.0f);
                    s_status.state = DOSE_CLOSING;
                }
                break;

            case DOSE_CLOSING:
                // wait for flow to settle near 0 (or mass near target band)
                if (fabsf(flow_f) < 0.2f) {
                    s_status.state = DOSE_SETTLE;
                }
                break;

            case DOSE_SETTLE:
                if (fabsf(s_cfg.target_g - m) <= s_cfg.settle_band_g) {
                    s_status.state = DOSE_DONE;
                } else {
                    // optional: tiny “trim” pulses later; for now, done when close enough
                    s_status.state = DOSE_DONE;
                }
                break;

            default:
                break;
        }

        // safety timeout
        if (t_elapsed > s_cfg.max_time_s) {
            ESP_LOGW(TAG, "Dose timeout");
            flap_set_opening(0.0f);
            s_status.state = DOSE_FAULT;
            break;
        }

        // publish status
        s_status.mass_g = m;
        s_status.flow_gps = flow_f;
        s_status.opening = flap_get_state().opening;
    }

    s_running = false;
    s_abort = false;
    ESP_LOGI(TAG, "Dose ended state=%d mass=%.2f flow=%.2f",
             (int)s_status.state, (double)s_status.mass_g, (double)s_status.flow_gps);

    vTaskDelete(NULL);
}

esp_err_t dosing_init(void)
{
    ESP_ERROR_CHECK(flap_init());
    ESP_ERROR_CHECK(loadcell_init());
    s_status.state = DOSE_IDLE;
    return ESP_OK;
}

esp_err_t dosing_start(const dose_cfg_t *cfg)
{
    if (!cfg) return ESP_ERR_INVALID_ARG;
    if (s_running) return ESP_ERR_INVALID_STATE;

    s_cfg = *cfg;
    s_abort = false;
    s_running = true;

    BaseType_t ok = xTaskCreate(dosing_task, "dosing_task", 4096, NULL, 9, &s_task);
    return ok == pdTRUE ? ESP_OK : ESP_ERR_NO_MEM;
}

esp_err_t dosing_abort(void)
{
    if (!s_running) return ESP_OK;
    s_abort = true;
    return ESP_OK;
}

dose_status_t dosing_get_status(void) { return s_status; }
