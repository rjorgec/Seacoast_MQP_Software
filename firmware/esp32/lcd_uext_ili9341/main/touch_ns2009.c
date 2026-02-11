#include "touch_ns2009.h"
#include "esp_log.h"
#include "esp_check.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <math.h>

static const char *TAG = "touch_ns2009";


#define NS2009_CMD_X   0xC0
#define NS2009_CMD_Y   0xD0
#define NS2009_CMD_Z1  0xE0
#define NS2009_CMD_Z2  0xF0


#define NS2009_PEN_DOWN_Z1   200
#define NS2009_PEN_UP_Z1      80


static esp_err_t ns2009_read12(touch_ns2009_t *ts, uint8_t cmd, uint16_t *out)
{
    uint8_t rx[2] = {0};
    esp_err_t err = i2c_master_transmit_receive(
        ts->dev,
        &cmd, 1,
        rx, 2,
        50 /*timeout ms*/
    );
    if (err != ESP_OK) return err;

    //low nibble should be 0
    if (rx[1] & 0x0F) {
        return ESP_ERR_INVALID_RESPONSE;
    }

    *out = ((uint16_t)rx[0] << 4) | ((uint16_t)rx[1] >> 4); // 12-bit
    return ESP_OK;
}

static uint16_t map_u16(uint16_t v, uint16_t in_min, uint16_t in_max, uint16_t out_max)
{
    if (in_max <= in_min) return 0;
    if (v < in_min) v = in_min;
    if (v > in_max) v = in_max;

    uint32_t num = (uint32_t)(v - in_min) * (uint32_t)out_max;
    uint32_t den = (uint32_t)(in_max - in_min);
    return (uint16_t)(num / den);
}

static void apply_transform(touch_ns2009_t *ts, uint16_t *x, uint16_t *y)
{
    uint16_t xx = *x;
    uint16_t yy = *y;

    if (ts->swap_xy) {
        uint16_t t = xx; xx = yy; yy = t;
    }
    if (ts->mirror_x) {
        xx = (ts->screen_w > 0) ? (uint16_t)((ts->screen_w - 1) - xx) : xx;
    }
    if (ts->mirror_y) {
        yy = (ts->screen_h > 0) ? (uint16_t)((ts->screen_h - 1) - yy) : yy;
    }

    *x = xx;
    *y = yy;
}

esp_err_t touch_ns2009_init(
    touch_ns2009_t *ts,
    int sda_gpio,
    int scl_gpio,
    uint32_t i2c_hz,
    uint8_t addr_7bit,
    uint16_t screen_w,
    uint16_t screen_h
)
{
    if (!ts) return ESP_ERR_INVALID_ARG;

    ts->addr = addr_7bit;
    ts->screen_w = screen_w;
    ts->screen_h = screen_h;

    
    ts->x_min = 0; ts->x_max = 4095;
    ts->y_min = 0; ts->y_max = 4095;

    ts->swap_xy = false;
    ts->mirror_x = false;
    ts->mirror_y = false;

    ts->last.x = 0;
    ts->last.y = 0;
    ts->last_pressed = false;

    // Create I2C bus
    i2c_master_bus_config_t bus_cfg = {
        .i2c_port = I2C_NUM_0,
        .sda_io_num = sda_gpio,
        .scl_io_num = scl_gpio,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .intr_priority = 0,
        .flags.enable_internal_pullup = true,
    };

    ESP_RETURN_ON_ERROR(i2c_new_master_bus(&bus_cfg, &ts->bus), TAG, "i2c_new_master_bus failed");

    // Add NS2009 as a device on the bus
    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = addr_7bit,
        .scl_speed_hz = i2c_hz,
    };

    ESP_RETURN_ON_ERROR(i2c_master_bus_add_device(ts->bus, &dev_cfg, &ts->dev), TAG, "add dev failed");

    // Quick sanity read: try Z1 once
    uint16_t z1 = 0;
    esp_err_t err = ns2009_read12(ts, NS2009_CMD_Z1, &z1);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "NS2009 probe read failed (%s). Address/pullups?", esp_err_to_name(err));
        return err;
    }

    ESP_LOGI(TAG, "NS2009 ready at 0x%02X, Z1=%u", addr_7bit, (unsigned)z1);
    return ESP_OK;
}

//helper
static inline float clamp01(float v)
{
    if (v < 0.0f) return 0.0f;
    if (v > 1.0f) return 1.0f;
    return v;
}

static void ns2009_map_to_screen(touch_ns2009_t *ts,
                                 uint16_t rx, uint16_t ry,
                                 uint16_t *x, uint16_t *y)
{
    //normalize using RAW calibration ranges
    float xn = (float)((int)rx - (int)ts->x_min) / (float)((int)ts->x_max - (int)ts->x_min);
    float yn = (float)((int)ry - (int)ts->y_min) / (float)((int)ts->y_max - (int)ts->y_min);
    xn = clamp01(xn);
    yn = clamp01(yn);

    //apply orientation transforms (do swap first, then mirrors)
    if (ts->swap_xy) {
        float tmp = xn; xn = yn; yn = tmp;
    }
    if (ts->mirror_x) xn = 1.0f - xn;
    if (ts->mirror_y) yn = 1.0f - yn;

    //scale to screen pixels
    int xi = (int)lroundf(xn * (ts->screen_w - 1));
    int yi = (int)lroundf(yn * (ts->screen_h - 1));

    if (xi < 0) xi = 0;
    if (yi < 0) yi = 0;
    if (xi >= ts->screen_w) xi = ts->screen_w - 1;
    if (yi >= ts->screen_h) yi = ts->screen_h - 1;

    *x = (uint16_t)xi;
    *y = (uint16_t)yi;
}

esp_err_t touch_ns2009_read(touch_ns2009_t *ts, bool *pressed, uint16_t *x, uint16_t *y)
{
    uint16_t z1 = 0;
    ESP_RETURN_ON_ERROR(ns2009_read12(ts, NS2009_CMD_Z1, &z1), TAG, "read Z1 failed");

    bool now_pressed;
    if (ts->last_pressed) {
        now_pressed = (z1 >= NS2009_PEN_UP_Z1);
    } else {
        now_pressed = (z1 >= NS2009_PEN_DOWN_Z1);
    }

    if (!now_pressed) {
        *pressed = false;
        *x = (uint16_t)ts->last.x;
        *y = (uint16_t)ts->last.y;
        ts->last_pressed = false;
        return ESP_OK;
    }

    uint16_t raw_x = 0, raw_y = 0;
    ESP_RETURN_ON_ERROR(ns2009_read12(ts, NS2009_CMD_X, &raw_x), TAG, "read X failed");
    ESP_RETURN_ON_ERROR(ns2009_read12(ts, NS2009_CMD_Y, &raw_y), TAG, "read Y failed");

    //new mapping attempt
    uint16_t sx = 0, sy = 0;
    ns2009_map_to_screen(ts, raw_x, raw_y, &sx, &sy);

    *pressed = true;
    *x = sx;
    *y = sy;

    ts->last.x = sx;
    ts->last.y = sy;
    ts->last_pressed = true;
    return ESP_OK;
}

// ---------- LVGL integration ----------

#if LVGL_VERSION_MAJOR >= 9
static void lvgl_read_cb(lv_indev_t *indev, lv_indev_data_t *data)
{
    touch_ns2009_t *ts = (touch_ns2009_t *)lv_indev_get_user_data(indev);
    bool pressed;
    uint16_t x, y;

    if (touch_ns2009_read(ts, &pressed, &x, &y) == ESP_OK) {
        data->state = pressed ? LV_INDEV_STATE_PRESSED : LV_INDEV_STATE_RELEASED;
        data->point.x = (lv_coord_t)x;
        data->point.y = (lv_coord_t)y;
    } else {
        data->state = LV_INDEV_STATE_RELEASED;
    }
}

void touch_ns2009_register_lvgl(touch_ns2009_t *ts)
{
    lv_indev_t *indev = lv_indev_create();
    lv_indev_set_type(indev, LV_INDEV_TYPE_POINTER);
    lv_indev_set_read_cb(indev, lvgl_read_cb);
    lv_indev_set_user_data(indev, ts);
}

#else  // LVGL 8.x
static void lvgl_read_cb(lv_indev_drv_t *drv, lv_indev_data_t *data)
{
    touch_ns2009_t *ts = (touch_ns2009_t *)drv->user_data;
    bool pressed;
    uint16_t x, y;

    if (touch_ns2009_read(ts, &pressed, &x, &y) == ESP_OK) {
        data->state = pressed ? LV_INDEV_STATE_PRESSED : LV_INDEV_STATE_RELEASED;
        data->point.x = (lv_coord_t)x;
        data->point.y = (lv_coord_t)y;
    } else {
        data->state = LV_INDEV_STATE_RELEASED;
    }
}

void touch_ns2009_register_lvgl(touch_ns2009_t *ts)
{
    lv_indev_drv_t indev_drv;
    lv_indev_drv_init(&indev_drv);
    indev_drv.type = LV_INDEV_TYPE_POINTER;
    indev_drv.read_cb = lvgl_read_cb;
    indev_drv.user_data = ts;
    lv_indev_drv_register(&indev_drv);
}
#endif

// void touch_ns2009_debug_dump(touch_ns2009_t *ts) //temp
// {
//     while (1) {
//         uint16_t z1=0, x=0, y=0;
//         esp_err_t e1 = ns2009_read12(ts, NS2009_CMD_Z1, &z1);
//         esp_err_t e2 = ns2009_read12(ts, NS2009_CMD_X,  &x);
//         esp_err_t e3 = ns2009_read12(ts, NS2009_CMD_Y,  &y);

//         if (e1 == ESP_OK && e2 == ESP_OK && e3 == ESP_OK) {
//             ESP_LOGI(TAG, "Z1=%u X=%u Y=%u", (unsigned)z1, (unsigned)x, (unsigned)y);
//         } else {
//             ESP_LOGW(TAG, "read failed: Z1=%s X=%s Y=%s",
//                      esp_err_to_name(e1), esp_err_to_name(e2), esp_err_to_name(e3));
//         }

//         vTaskDelay(pdMS_TO_TICKS(200));
//     }
// }
void touch_ns2009_capture_minmax(touch_ns2009_t *ts, int seconds)
{
    uint16_t xmin = 4095, xmax = 0, ymin = 4095, ymax = 0;
    int samples = 0;

    const TickType_t deadline = xTaskGetTickCount() + pdMS_TO_TICKS(seconds * 1000);

    ESP_LOGI(TAG, "Capture min/max for %d s: drag along ALL edges, especially corners.", seconds);

    while (xTaskGetTickCount() < deadline) {
        uint16_t z1 = 0, rx = 0, ry = 0;

        if (ns2009_read12(ts, NS2009_CMD_Z1, &z1) == ESP_OK && z1 > 200) { // “pressed”
            if (ns2009_read12(ts, NS2009_CMD_X, &rx) == ESP_OK &&
                ns2009_read12(ts, NS2009_CMD_Y, &ry) == ESP_OK) {

                if (rx < xmin) xmin = rx;
                if (rx > xmax) xmax = rx;
                if (ry < ymin) ymin = ry;
                if (ry > ymax) ymax = ry;
                samples++;
            }
        }

        vTaskDelay(pdMS_TO_TICKS(20));
    }

    ESP_LOGI(TAG, "Raw min/max from %d samples:", samples);
    ESP_LOGI(TAG, "  X: min=%u max=%u", (unsigned)xmin, (unsigned)xmax);
    ESP_LOGI(TAG, "  Y: min=%u max=%u", (unsigned)ymin, (unsigned)ymax);
}
