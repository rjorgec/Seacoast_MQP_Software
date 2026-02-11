#pragma once
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_ops.h"

typedef struct {
    esp_lcd_panel_io_handle_t io;
    esp_lcd_panel_handle_t panel;
    int hres;
    int vres;
} display_handles_t;

display_handles_t display_init(void);
