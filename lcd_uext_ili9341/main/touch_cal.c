#include "touch_cal.h"
#include <nvs.h>
#include <nvs_flash.h> 
#include "esp_log.h"

static const char *TAG = "touch_cal";
static const char *NS  = "touch";
static const char *KEY = "cal_v1";

esp_err_t touch_cal_load(touch_cal_t *out)
{
    if (!out) return ESP_ERR_INVALID_ARG;

    nvs_handle_t h;
    esp_err_t err = nvs_open(NS, NVS_READONLY, &h);
    if (err != ESP_OK) return err;

    size_t len = sizeof(*out);
    err = nvs_get_blob(h, KEY, out, &len);
    nvs_close(h);

    if (err == ESP_OK && len == sizeof(*out) && out->version == 1) {
        ESP_LOGI(TAG, "Loaded cal: X[%u..%u] Y[%u..%u]",
                 out->x_min, out->x_max, out->y_min, out->y_max);
        return ESP_OK;
    }
    return err;
}

esp_err_t touch_cal_save(const touch_cal_t *cal)
{
    if (!cal) return ESP_ERR_INVALID_ARG;

    nvs_handle_t h;
    esp_err_t err = nvs_open(NS, NVS_READWRITE, &h);
    if (err != ESP_OK) return err;

    err = nvs_set_blob(h, KEY, cal, sizeof(*cal));
    if (err == ESP_OK) err = nvs_commit(h);
    nvs_close(h);

    ESP_LOGI(TAG, "Saved cal: X[%u..%u] Y[%u..%u]",
             cal->x_min, cal->x_max, cal->y_min, cal->y_max);
    return err;
}

void touch_cal_apply(touch_ns2009_t *ts, const touch_cal_t *cal)
{
    if (!ts || !cal) return;
    ts->x_min = cal->x_min;
    ts->x_max = cal->x_max;
    ts->y_min = cal->y_min;
    ts->y_max = cal->y_max;
}
