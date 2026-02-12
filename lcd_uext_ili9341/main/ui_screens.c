#include <assert.h>
#include <stdio.h>

#include "ui_screens.h"
#include "control.h"

#include "lvgl.h"
#include "esp_log.h"
#include "esp_lvgl_port.h"
#include "motor_hal.h"

static const char *TAG = "ui_screens";

#if defined(LVGL_VERSION_MAJOR) && (LVGL_VERSION_MAJOR >= 9)
#define LVGL_ACTIVE_SCREEN() lv_screen_active()
#else
#define LVGL_ACTIVE_SCREEN() lv_scr_act()
#endif


#define JOG_SPEED          2500u
#define JOG_LOW_TH         0u
#define JOG_HIGH_TH        4095u
#define JOG_INTERVAL_MS    50u

static lv_obj_t *lbl_status = NULL;

void ui_status_set(const char *s)
{
    if (!s) s = "";
    lvgl_port_lock(0);
    if (lbl_status) lv_label_set_text(lbl_status, s);
    lvgl_port_unlock();
}

static void set_status(const char *s)
{
    ui_status_set(s);
}


// ---- Button callbacks ----

static void on_start(lv_event_t *e)
{
    (void)e;
    ctrl_cmd_t cmd = {
        .type = CTRL_CMD_START,
        .target_g = 50.0f,   //TODO make it editable
        .recipe_id = 0,
    };

    bool ok = control_send(&cmd);
    set_status(ok ? "START sent" : "START failed");
    ESP_LOGI(TAG, "Start pressed (send=%d)", (int)ok);
}

static void on_fwd(lv_event_t *e)
{
    (void)e;
    esp_err_t err = motor_linact_start_monitor_dir(MOTOR_DIR_FWD,
                                                   JOG_SPEED,
                                                   JOG_LOW_TH,
                                                   JOG_HIGH_TH,
                                                   JOG_INTERVAL_MS);
    set_status(err == ESP_OK ? "LINACT: FWD" : "LINACT: FWD FAILED");
    ESP_LOGI(TAG, "FWD pressed (%s)", esp_err_to_name(err));
}

static void on_rev(lv_event_t *e)
{
    (void)e;
    esp_err_t err = motor_linact_start_monitor_dir(MOTOR_DIR_REV,
                                                   JOG_SPEED,
                                                   JOG_LOW_TH,
                                                   JOG_HIGH_TH,
                                                   JOG_INTERVAL_MS);
    set_status(err == ESP_OK ? "LINACT: REV" : "LINACT: REV FAILED");
    ESP_LOGI(TAG, "REV pressed (%s)", esp_err_to_name(err));
}

static void on_stop_linact(lv_event_t *e)
{
    (void)e;
    esp_err_t err = motor_linact_stop_monitor();
    set_status(err == ESP_OK ? "LINACT: STOP" : "LINACT: STOP FAILED");
    ESP_LOGI(TAG, "STOP pressed (%s)", esp_err_to_name(err));
}


// static void on_clean(lv_event_t *e) //dont need a clean lol
// {
//     (void)e;
//     ctrl_cmd_t cmd = { .type = CTRL_CMD_CLEAN };

//     bool ok = control_send(&cmd);
//     set_status(ok ? "CLEAN sent" : "CLEAN failed");
//     ESP_LOGI(TAG, "Clean pressed (send=%d)", (int)ok);
// }

static void on_tare(lv_event_t *e)
{
    (void)e;
    ctrl_cmd_t cmd = { .type = CTRL_CMD_TARE };

    bool ok = control_send(&cmd);
    set_status(ok ? "TARE sent" : "TARE failed");
    ESP_LOGI(TAG, "Tare pressed (send=%d)", (int)ok);
}

static void on_recipes(lv_event_t *e)
{
    (void)e;
    //TODO
    set_status("RECIPES (TODO)");
    ESP_LOGI(TAG, "Recipes pressed");
}

static void on_pause(lv_event_t *e)
{
    (void)e;
    ctrl_cmd_t cmd = { .type = CTRL_CMD_PAUSE };

    bool ok = control_send(&cmd);
    set_status(ok ? "PAUSE sent" : "PAUSE failed");
    ESP_LOGI(TAG, "Pause pressed (send=%d)", (int)ok);
}

static void on_stop(lv_event_t *e)
{
    (void)e;
    ctrl_cmd_t cmd = { .type = CTRL_CMD_STOP };

    bool ok = control_send(&cmd);
    set_status(ok ? "STOP sent" : "STOP failed");
    ESP_LOGI(TAG, "Stop pressed (send=%d)", (int)ok);
}

// ---- UI helpers ----

static lv_obj_t *make_big_btn(lv_obj_t *parent, const char *txt, lv_event_cb_t cb)
{
    lv_obj_t *btn = lv_btn_create(parent);
    lv_obj_add_event_cb(btn, cb, LV_EVENT_CLICKED, NULL);

    lv_obj_t *label = lv_label_create(btn);
    lv_label_set_text(label, txt);
    lv_obj_center(label);

    return btn;
}

void ui_show_home(void)
{
    lv_obj_t *scr = LVGL_ACTIVE_SCREEN();
    lv_obj_clean(scr);

    //fix scrolling
    lv_obj_clear_flag(scr, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_style_pad_all(scr, 10, 0);

    //status strip
    lbl_status = lv_label_create(scr);
    lv_label_set_text(lbl_status, "IDLE • Seacoast Inoculator");
    lv_obj_align(lbl_status, LV_ALIGN_TOP_LEFT, 0, 0);

    //container for 2x2 big butt
    lv_obj_t *cont = lv_obj_create(scr);
    lv_obj_clear_flag(cont, LV_OBJ_FLAG_SCROLLABLE);

    lv_obj_set_size(cont, 300, 200);
    lv_obj_align(cont, LV_ALIGN_BOTTOM_MID, 0, 0);

    lv_obj_set_style_pad_all(cont, 6, 0);
    lv_obj_set_style_pad_row(cont, 6, 0);
    lv_obj_set_style_pad_column(cont, 6, 0);

    lv_obj_set_layout(cont, LV_LAYOUT_GRID);

    static lv_coord_t col_dsc[] = { LV_GRID_FR(1), LV_GRID_FR(1), LV_GRID_TEMPLATE_LAST };
    static lv_coord_t row_dsc[] = { LV_GRID_FR(1), LV_GRID_FR(1), LV_GRID_TEMPLATE_LAST };
    lv_obj_set_grid_dsc_array(cont, col_dsc, row_dsc);

    lv_obj_t *b_fwd    = make_big_btn(cont, "Forward",  on_fwd);
    lv_obj_t *b_rev    = make_big_btn(cont, "Backward", on_rev);
    lv_obj_t *b_stop   = make_big_btn(cont, "Stop",     on_stop_linact);
    lv_obj_t *b_tare   = make_big_btn(cont, "Tare",     on_tare);  

    lv_obj_set_grid_cell(b_fwd,  LV_GRID_ALIGN_STRETCH, 0, 1, LV_GRID_ALIGN_STRETCH, 0, 1);
    lv_obj_set_grid_cell(b_rev,  LV_GRID_ALIGN_STRETCH, 1, 1, LV_GRID_ALIGN_STRETCH, 0, 1);
    lv_obj_set_grid_cell(b_stop, LV_GRID_ALIGN_STRETCH, 0, 1, LV_GRID_ALIGN_STRETCH, 1, 1);
    lv_obj_set_grid_cell(b_tare, LV_GRID_ALIGN_STRETCH, 1, 1, LV_GRID_ALIGN_STRETCH, 1, 1);


    }

//dosing screens tub
void ui_show_dosing(void)
{
    lv_obj_t *scr = LVGL_ACTIVE_SCREEN();
    lv_obj_clean(scr);
    lv_obj_clear_flag(scr, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_style_pad_all(scr, 10, 0);

    lbl_status = lv_label_create(scr);
    lv_label_set_text(lbl_status, "DOSING…");
    lv_obj_align(lbl_status, LV_ALIGN_TOP_LEFT, 0, 0);

    lv_obj_t *btn_pause = lv_btn_create(scr);
    lv_obj_set_size(btn_pause, 140, 70);
    lv_obj_align(btn_pause, LV_ALIGN_BOTTOM_LEFT, 0, 0);
    lv_obj_add_event_cb(btn_pause, on_pause, LV_EVENT_CLICKED, NULL);
    lv_obj_t *lp = lv_label_create(btn_pause);
    lv_label_set_text(lp, "Pause");
    lv_obj_center(lp);

    lv_obj_t *btn_stop = lv_btn_create(scr);
    lv_obj_set_size(btn_stop, 140, 70);
    lv_obj_align(btn_stop, LV_ALIGN_BOTTOM_RIGHT, 0, 0);
    lv_obj_add_event_cb(btn_stop, on_stop, LV_EVENT_CLICKED, NULL);
    lv_obj_t *ls = lv_label_create(btn_stop);
    lv_label_set_text(ls, "Stop");
    lv_obj_center(ls);
}
