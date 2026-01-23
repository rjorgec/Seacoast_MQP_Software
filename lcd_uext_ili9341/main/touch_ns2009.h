#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"
#include "driver/i2c_master.h"
#include "lvgl.h"

typedef struct {
    i2c_master_bus_handle_t bus;
    i2c_master_dev_handle_t dev;
    uint8_t addr;

    // Screen size in LVGL coordinates (i.e., after your chosen rotation)
    uint16_t screen_w;
    uint16_t screen_h;

    // Simple 12-bit calibration range (raw 0..4095 by default)
    uint16_t x_min, x_max;
    uint16_t y_min, y_max;

    // Apply same “feel” as your display rotation/fixes
    bool swap_xy;
    bool mirror_x;
    bool mirror_y;

    // For LVGL “continue last point” behavior
    lv_point_t last;
    bool last_pressed;
} touch_ns2009_t;

esp_err_t touch_ns2009_init(
    touch_ns2009_t *ts,
    int sda_gpio,
    int scl_gpio,
    uint32_t i2c_hz,
    uint8_t addr_7bit,
    uint16_t screen_w,
    uint16_t screen_h
);

esp_err_t touch_ns2009_read(
    touch_ns2009_t *ts,
    bool *pressed,
    uint16_t *x,
    uint16_t *y
);


void touch_ns2009_register_lvgl(touch_ns2009_t *ts);
// void touch_ns2009_debug_dump(touch_ns2009_t *ts);
void touch_ns2009_capture_minmax(touch_ns2009_t *ts, int seconds);

