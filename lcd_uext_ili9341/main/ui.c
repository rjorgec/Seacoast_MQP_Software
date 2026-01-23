#include <assert.h>
#include "ui.h"

#include "lvgl.h"
#include "esp_lvgl_port.h"

#include "touch_ns2009.h"
#include "esp_lvgl_port.h"

#if defined(LVGL_VERSION_MAJOR) && (LVGL_VERSION_MAJOR >= 9)
#define LVGL_ACTIVE_SCREEN() lv_screen_active()
#else
#define LVGL_ACTIVE_SCREEN() lv_scr_act()
#endif

static touch_ns2009_t g_touch;

// static void ui_build(void)
// {
//     lv_obj_t *label = lv_label_create(LVGL_ACTIVE_SCREEN());
//     lv_label_set_text(label, "Seacoast Mushrooms");
//     lv_obj_center(label);
// }
static lv_obj_t *g_touch_label;

static void on_btn(lv_event_t *e)
{
    lv_event_code_t code = lv_event_get_code(e);
    if (code == LV_EVENT_PRESSED) {
        lv_label_set_text(g_touch_label, "PRESSED");
    } else if (code == LV_EVENT_RELEASED) {
        lv_label_set_text(g_touch_label, "RELEASED");
    }
}

void ui_build_touch_test(void)
{
    lv_obj_t *scr = LVGL_ACTIVE_SCREEN();

    lv_obj_clean(scr);

    //Big button covering most of screen
    lv_obj_t *btn = lv_btn_create(scr);
    lv_obj_set_size(btn, 300, 180);
    lv_obj_center(btn);

    lv_obj_add_event_cb(btn, on_btn, LV_EVENT_ALL, NULL);

    g_touch_label = lv_label_create(btn);
    lv_label_set_text(g_touch_label, "Tap the screen");
    lv_obj_center(g_touch_label);
}

void ui_init(const display_handles_t *disp)
{
    const lvgl_port_cfg_t lvgl_cfg = ESP_LVGL_PORT_INIT_CONFIG();
    ESP_ERROR_CHECK(lvgl_port_init(&lvgl_cfg));

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

    lvgl_port_lock(0);
    ui_build_touch_test();
    lvgl_port_unlock();
}


void ui_init_after_display_ready(void)
{
    
    const int SDA = 6;
    const int SCL = 7;

    //display res
    const uint16_t W = 320;
    const uint16_t H = 240;

    ESP_ERROR_CHECK(touch_ns2009_init(&g_touch, SDA, SCL, 100000, 0x48, W, H));

    g_touch.swap_xy  = true;
    g_touch.mirror_x = true;
    g_touch.mirror_y = true;


    lvgl_port_lock(0);
    touch_ns2009_register_lvgl(&g_touch);
    lvgl_port_unlock();
}



