#include <assert.h>
#include "ui.h"

#include "lvgl.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_lvgl_port.h"

#include "touch_ns2009.h"
#include "ui_screens.h"  

static const char *TAG = "ui";

#if defined(LVGL_VERSION_MAJOR) && (LVGL_VERSION_MAJOR >= 9)
#define LVGL_ACTIVE_SCREEN() lv_screen_active()
#else
#define LVGL_ACTIVE_SCREEN() lv_scr_act()
#endif

static touch_ns2009_t g_touch;

void ui_init(const display_handles_t *disp)
{
    ESP_LOGI(TAG, "LVGL init");
    const lvgl_port_cfg_t lvgl_cfg = ESP_LVGL_PORT_INIT_CONFIG();
    ESP_ERROR_CHECK(lvgl_port_init(&lvgl_cfg));

    ESP_LOGI(TAG, "Register display");
    lvgl_port_display_cfg_t cfg = {0};
    cfg.io_handle = disp->io;
    cfg.panel_handle = disp->panel;

    cfg.hres = disp->hres;
    cfg.vres = disp->vres;

    cfg.buffer_size = cfg.hres * 40; // pixels
    cfg.double_buffer = true;
    cfg.monochrome = false;
    cfg.color_format = LV_COLOR_FORMAT_RGB565;


    cfg.rotation.swap_xy = true;
    cfg.rotation.mirror_x = true;
    cfg.rotation.mirror_y = true;

    cfg.flags.buff_dma = true;
    cfg.flags.swap_bytes = false;

    lv_disp_t *d = lvgl_port_add_disp(&cfg);
    assert(d);

    //home screen
    lvgl_port_lock(0);
    ui_show_home();
    lvgl_port_unlock();
}

void ui_init_after_display_ready(void)
{
    //I2c touch
    const int SDA = 6;
    const int SCL = 7;

    const uint16_t W = 320;
    const uint16_t H = 240;

    ESP_LOGI(TAG, "Touch init");
    ESP_ERROR_CHECK(touch_ns2009_init(&g_touch, SDA, SCL, 100000, 0x48, W, H));

 
    g_touch.swap_xy  = true;
    g_touch.mirror_x = true;
    g_touch.mirror_y = true;

    lvgl_port_lock(0);
    touch_ns2009_register_lvgl(&g_touch);
    lvgl_port_unlock();
}
