#include "dosing.h"
#include <esp_timer.h>
#include <math.h>
#include "esp_log.h"
#include "esp_timer.h"

#include "flap.h"
#include "loadcell.h"
#include "pico_link.h"
#include "proto/proto.h"

static const char *TAG = "dosing";

static inline uint32_t now_ms(void)
{
    return (uint32_t)(esp_timer_get_time() / 1000ULL);
}

static float clampf(float x, float lo, float hi)
{
    if (x < lo)
        return lo;
    if (x > hi)
        return hi;
    return x;
}

// dan can swap this out later
static esp_err_t set_flaps_opening(float opening_0_to_1)
{
    opening_0_to_1 = clampf(opening_0_to_1, 0.0f, 1.0f);

    return flap_set_opening(opening_0_to_1);
}

// adapt
static esp_err_t read_mass_g(float *out_g)
{
    return loadcell_read_g(out_g);
}

// ---------- Public API ----------

esp_err_t dosing_init(dosing_ctx_t *ctx)
{
    if (!ctx)
        return ESP_ERR_INVALID_ARG;

    *ctx = (dosing_ctx_t){0};

    ctx->state = DOSING_STATE_IDLE;
    ctx->running = false;
    ctx->last_err = ESP_OK;

    ctx->cfg.target_g = 0.0f;
    ctx->cfg.taper_start_g = 8.0f; // egin taper 8g before target (adjust later)
    ctx->cfg.final_band_g = 2.0f;  // within 2g of target, go FINAL
    ctx->cfg.overshoot_g = 0.7f;   // tolerate up to +0.7g then stop hard

    ctx->cfg.fast_open = 1.0f;
    ctx->cfg.taper_open = 0.40f;
    ctx->cfg.final_open = 0.15f;

    ctx->cfg.min_step_ms = 50;
    ctx->cfg.ema_alpha = 0.25f; // smoothing

    // making sure flaps are closed initially
    esp_err_t err = set_flaps_opening(0.0f);
    if (err != ESP_OK)
    {
        ctx->state = DOSING_STATE_ERROR;
        ctx->last_err = err;
        return err;
    }

    ESP_LOGI(TAG, "dosing_init ok");
    return ESP_OK;
}

esp_err_t dosing_start(dosing_ctx_t *ctx, float target_g)
{
    if (!ctx)
        return ESP_ERR_INVALID_ARG;
    if (target_g <= 0.0f)
        return ESP_ERR_INVALID_ARG;

    ctx->cfg.target_g = target_g;

    // Reset filters/state
    ctx->filtered_g = 0.0f;
    ctx->last_g = 0.0f;
    ctx->last_step_ms = 0;
    ctx->running = true;
    ctx->state = DOSING_STATE_PRIME;
    ctx->last_err = ESP_OK;

    ESP_LOGI(TAG, "dosing_start target=%.2fg", target_g);
    return ESP_OK;
}

esp_err_t dosing_abort(dosing_ctx_t *ctx)
{
    if (!ctx)
        return ESP_ERR_INVALID_ARG;

    ctx->running = false;
    ctx->state = DOSING_STATE_ABORTED;

    esp_err_t err = set_flaps_opening(0.0f);
    if (err != ESP_OK)
    {
        ctx->state = DOSING_STATE_ERROR;
        ctx->last_err = err;
        return err;
    }

    esp_err_t send_err = pico_link_send(MSG_CTRL_STOP, NULL, 0, NULL);
    if (send_err != ESP_OK)
    {
        ESP_LOGW(TAG, "dosing_abort: MSG_CTRL_STOP send failed: %s", esp_err_to_name(send_err));
    }

    ESP_LOGW(TAG, "dosing_abort");
    return ESP_OK;
}

esp_err_t dosing_tick(dosing_ctx_t *ctx)
{
    if (!ctx)
        return ESP_ERR_INVALID_ARG;

    if (!ctx->running)
    {
        // Keep flaps closed in non-running states
        if (ctx->state == DOSING_STATE_IDLE || ctx->state == DOSING_STATE_DONE)
        {
            return ESP_OK;
        }
        return ESP_OK;
    }

    float g = 0.0f;
    esp_err_t err = read_mass_g(&g);
    if (err != ESP_OK)
    {
        ctx->state = DOSING_STATE_ERROR;
        ctx->last_err = err;
        ctx->running = false;
        set_flaps_opening(0.0f);
        ESP_LOGE(TAG, "loadcell read failed: %s", esp_err_to_name(err));
        return err;
    }

    // EMA smoothing to reduce jitter near threshold
    if (ctx->filtered_g == 0.0f && ctx->last_g == 0.0f)
    {
        ctx->filtered_g = g; // first sample
    }
    else
    {
        float a = clampf(ctx->cfg.ema_alpha, 0.01f, 1.0f);
        ctx->filtered_g = a * g + (1.0f - a) * ctx->filtered_g;
    }
    ctx->last_g = g;

    const float target = ctx->cfg.target_g;
    const float remain = target - ctx->filtered_g;

    // Overshoot protection
    if (ctx->filtered_g > (target + ctx->cfg.overshoot_g))
    {
        ESP_LOGW(TAG, "Overshoot: %.2f > %.2f. Closing flaps.", ctx->filtered_g, target);
        ctx->running = false;
        ctx->state = DOSING_STATE_DONE;
        set_flaps_opening(0.0f);
        return ESP_OK;
    }

    // Throttle actuator updates
    const uint32_t t = now_ms();
    if (ctx->last_step_ms && (t - ctx->last_step_ms) < ctx->cfg.min_step_ms)
    {
        return ESP_OK;
    }

    // State machine
    switch (ctx->state)
    {
    case DOSING_STATE_PRIME:

        err = set_flaps_opening(ctx->cfg.fast_open);
        if (err != ESP_OK)
            goto actuator_fail;
        ctx->state = DOSING_STATE_FAST;
        ctx->last_step_ms = t;
        ESP_LOGI(TAG, "PRIME->FAST (g=%.2f)", ctx->filtered_g);
        break;

    case DOSING_STATE_FAST:
        // If close enough, start taper
        if (remain <= ctx->cfg.taper_start_g)
        {
            err = set_flaps_opening(ctx->cfg.taper_open);
            if (err != ESP_OK)
                goto actuator_fail;
            ctx->state = DOSING_STATE_TAPER;
            ctx->last_step_ms = t;
            ESP_LOGI(TAG, "FAST->TAPER remain=%.2f (g=%.2f)", remain, ctx->filtered_g);
        }
        else
        {
            // Keep fast open
            err = set_flaps_opening(ctx->cfg.fast_open);
            if (err != ESP_OK)
                goto actuator_fail;
            ctx->last_step_ms = t;
        }
        break;

    case DOSING_STATE_TAPER:
        // go to final small opening
        if (remain <= ctx->cfg.final_band_g)
        {
            err = set_flaps_opening(ctx->cfg.final_open);
            if (err != ESP_OK)
                goto actuator_fail;
            ctx->state = DOSING_STATE_FINAL;
            ctx->last_step_ms = t;
            ESP_LOGI(TAG, "TAPER->FINAL remain=%.2f (g=%.2f)", remain, ctx->filtered_g);
        }
        else
        {
            // keep taper opening
            err = set_flaps_opening(ctx->cfg.taper_open);
            if (err != ESP_OK)
                goto actuator_fail;
            ctx->last_step_ms = t;
        }
        break;

    case DOSING_STATE_FINAL:
        // stop when reach target
        if (ctx->filtered_g >= target)
        {
            err = set_flaps_opening(0.0f);
            if (err != ESP_OK)
                goto actuator_fail;
            ctx->running = false;
            ctx->state = DOSING_STATE_DONE;
            ctx->last_step_ms = t;
            ESP_LOGI(TAG, "FINAL->DONE g=%.2f target=%.2f", ctx->filtered_g, target);
        }
        else
        {
            err = set_flaps_opening(ctx->cfg.final_open);
            if (err != ESP_OK)
                goto actuator_fail;
            ctx->last_step_ms = t;
        }
        break;

    default:
        // failsafe
        ESP_LOGW(TAG, "Unexpected state %d while running, aborting", (int)ctx->state);
        return dosing_abort(ctx);
    }

    return ESP_OK;

actuator_fail:
    ctx->state = DOSING_STATE_ERROR;
    ctx->last_err = err;
    ctx->running = false;
    set_flaps_opening(0.0f);
    ESP_LOGE(TAG, "Actuator control failed: %s", esp_err_to_name(err));
    return err;
}

bool dosing_is_running(const dosing_ctx_t *ctx)
{
    return ctx && ctx->running;
}

dosing_state_t dosing_get_state(const dosing_ctx_t *ctx)
{
    return ctx ? ctx->state : DOSING_STATE_ERROR;
}
