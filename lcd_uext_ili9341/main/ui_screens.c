#include "ui_screens.h"
#include "lvgl.h"
#include "control.h"

#if defined(LVGL_VERSION_MAJOR) && (LVGL_VERSION_MAJOR >= 9)
#define LVGL_ACTIVE_SCREEN() lv_screen_active()
#else
#define LVGL_ACTIVE_SCREEN() lv_scr_act()
#endif

static lv_obj_t *lbl_status;
static lv_obj_t *lbl_target;

static void set_status(const char *s)
{
    if (lbl_status) lv_label_set_text(lbl_status, s);
}

static void on_start(lv_event_t *e)
{
    (void)e;
    ctrl_cmd_t cmd = {.type = CTRL_CMD_START, .target_g = 50.0f, .recipe_id = 0};
    control_send(&cmd);
    ui_show_run();
}

static void on_clean(lv_event_t *e)
{
    (void)e;
    ctrl_cmd_t cmd = {.type = CTRL_CMD_CLEAN};
    control_send(&cmd);
    set_status("Cleaning requested");
}

static void on_tare(lv_event_t *e)
{
    (void)e;
    ctrl_cmd_t cmd = {.type = CTRL_CMD_TARE};
    control_send(&cmd);
    set_status("Tare requested");
}

static void on_pause(lv_event_t *e)
{
    (void)e;
    ctrl_cmd_t cmd = {.type = CTRL_CMD_PAUSE};
    control_send(&cmd);
}

static void on_stop(lv_event_t *e)
{
    (void)e;
    ctrl_cmd_t cmd = {.type = CTRL_CMD_STOP};
    control_send(&cmd);
    ui_show_home();
}

void ui_show_home(void)
{
    lv_obj_t *scr = LVGL_ACTIVE_SCREEN();
    lv_obj_clean(scr);
    lv_obj_set_style_pad_all(scr, 10, 0);

    lv_obj_clear_flag(scr, LV_OBJ_FLAG_SCROLLABLE); //stop scrolling smh
    // Status strip
    lbl_status = lv_label_create(scr);
    lv_label_set_text(lbl_status, "IDLE • Seacoast Inoculator");
    lv_obj_align(lbl_status, LV_ALIGN_TOP_LEFT, 0, 0);

    // 2x2 big buttons
    lv_obj_t *cont = lv_obj_create(scr);

    lv_obj_clear_flag(cont, LV_OBJ_FLAG_SCROLLABLE);  //stop scrolling smh

    lv_obj_set_size(cont, 300, 200);
    lv_obj_align(cont, LV_ALIGN_BOTTOM_MID, 0, 0);
    lv_obj_set_style_pad_all(cont, 6, 0);
    lv_obj_set_style_pad_row(cont, 6, 0);
    lv_obj_set_style_pad_column(cont, 6, 0);
    lv_obj_set_layout(cont, LV_LAYOUT_GRID);

    static lv_coord_t col[] = {145, 145, LV_GRID_TEMPLATE_LAST};
    static lv_coord_t row[] = {95, 95, LV_GRID_TEMPLATE_LAST};
    lv_obj_set_grid_dsc_array(cont, col, row);

    lv_obj_t *b_start = lv_btn_create(cont);
    lv_obj_set_grid_cell(b_start, LV_GRID_ALIGN_STRETCH, 0, 1, LV_GRID_ALIGN_STRETCH, 0, 1);
    lv_obj_add_event_cb(b_start, on_start, LV_EVENT_CLICKED, NULL);
    lv_obj_t *t1 = lv_label_create(b_start);
    lv_label_set_text(t1, "START");
    lv_obj_center(t1);

    lv_obj_t *b_clean = lv_btn_create(cont);
    lv_obj_set_grid_cell(b_clean, LV_GRID_ALIGN_STRETCH, 1, 1, LV_GRID_ALIGN_STRETCH, 0, 1);
    lv_obj_add_event_cb(b_clean, on_clean, LV_EVENT_CLICKED, NULL);
    lv_obj_t *t2 = lv_label_create(b_clean);
    lv_label_set_text(t2, "CLEAN");
    lv_obj_center(t2);

    lv_obj_t *b_tare = lv_btn_create(cont);
    lv_obj_set_grid_cell(b_tare, LV_GRID_ALIGN_STRETCH, 0, 1, LV_GRID_ALIGN_STRETCH, 1, 1);
    lv_obj_add_event_cb(b_tare, on_tare, LV_EVENT_CLICKED, NULL);
    lv_obj_t *t3 = lv_label_create(b_tare);
    lv_label_set_text(t3, "TARE");
    lv_obj_center(t3);

    lv_obj_t *b_run = lv_btn_create(cont);
    lv_obj_set_grid_cell(b_run, LV_GRID_ALIGN_STRETCH, 1, 1, LV_GRID_ALIGN_STRETCH, 1, 1);
    lv_obj_add_event_cb(b_run, on_start, LV_EVENT_CLICKED, NULL);
    lv_obj_t *t4 = lv_label_create(b_run);
    lv_label_set_text(t4, "RECIPES");
    lv_obj_center(t4);
}

void ui_show_run(void)
{
    lv_obj_t *scr = LVGL_ACTIVE_SCREEN();
    lv_obj_clean(scr);

    lv_obj_set_style_pad_all(scr, 10, 0);

    lbl_status = lv_label_create(scr);
    lv_label_set_text(lbl_status, "RUNNING");
    lv_obj_align(lbl_status, LV_ALIGN_TOP_LEFT, 0, 0);

    lbl_target = lv_label_create(scr);
    lv_label_set_text(lbl_target, "Target: 50.0 g");
    lv_obj_align(lbl_target, LV_ALIGN_TOP_LEFT, 0, 25);

    // Big pause/stop
    lv_obj_t *b_pause = lv_btn_create(scr);
    lv_obj_set_size(b_pause, 150, 70);
    lv_obj_align(b_pause, LV_ALIGN_BOTTOM_LEFT, 0, 0);
    lv_obj_add_event_cb(b_pause, on_pause, LV_EVENT_CLICKED, NULL);
    lv_obj_t *tp = lv_label_create(b_pause);
    lv_label_set_text(tp, "PAUSE");
    lv_obj_center(tp);

    lv_obj_t *b_stop = lv_btn_create(scr);
    lv_obj_set_size(b_stop, 150, 70);
    lv_obj_align(b_stop, LV_ALIGN_BOTTOM_RIGHT, 0, 0);
    lv_obj_add_event_cb(b_stop, on_stop, LV_EVENT_CLICKED, NULL);
    lv_obj_t *ts = lv_label_create(b_stop);
    lv_label_set_text(ts, "STOP");
    lv_obj_center(ts);
}
