#include <assert.h>
#include <stdio.h>

#include "ui.h"
#include "control.h"
#include "dosing.h"

#include "lvgl.h"
#include "esp_lvgl_port.h"

#include "touch_ns2009.h"


#if defined(LVGL_VERSION_MAJOR) && (LVGL_VERSION_MAJOR >= 9)
#define LVGL_ACTIVE_SCREEN() lv_screen_active()
#else
#define LVGL_ACTIVE_SCREEN() lv_scr_act()
#endif

static touch_ns2009_t g_touch;

/* ---------------------------
 * Home/Run UI globals
 * --------------------------- */
static lv_obj_t *lbl_status;

static lv_obj_t *lbl_state;
static lv_obj_t *lbl_mass;
static lv_obj_t *lbl_flow;
static lv_obj_t *bar_opening;
static lv_timer_t *run_timer;

/* ---------------------------
 * Forward declarations
 * --------------------------- */
static void ui_apply_noscroll(lv_obj_t *obj);
static void ui_run_tick(lv_timer_t *t);

static void on_btn_start(lv_event_t *e);
static void on_btn_clean(lv_event_t *e);
static void on_btn_tare(lv_event_t *e);
static void on_btn_recipes(lv_event_t *e);

static void on_btn_abort(lv_event_t *e);
static void on_btn_home(lv_event_t *e);

static const char *dose_state_str(dose_state_t s);

/* ---------------------------
 * Helpers
 * --------------------------- */
static void ui_apply_noscroll(lv_obj_t *obj)
{
    lv_obj_clear_flag(obj, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_scroll_dir(obj, LV_DIR_NONE);
    lv_obj_set_scrollbar_mode(obj, LV_SCROLLBAR_MODE_OFF);
}

/* ---------------------------
 * Screens
 * --------------------------- */
void ui_show_home(void)
{
    lv_obj_t *scr = LVGL_ACTIVE_SCREEN();
    lv_obj_clean(scr);
    lv_obj_set_style_pad_all(scr, 10, 0);
    ui_apply_noscroll(scr);

    //Stop run timer if we came back from Run
    if (run_timer) {
        lv_timer_del(run_timer);
        run_timer = NULL;
    }

    // Status strip
    lbl_status = lv_label_create(scr);
    lv_label_set_text(lbl_status, "IDLE • Seacoast Inoculator");
    lv_obj_align(lbl_status, LV_ALIGN_TOP_LEFT, 0, 0);

    // 2x2 big buttons container
    lv_obj_t *cont = lv_obj_create(scr);
    ui_apply_noscroll(cont);

    lv_obj_set_size(cont, 300, 200);
    lv_obj_align(cont, LV_ALIGN_BOTTOM_MID, 0, 0);
    lv_obj_set_style_pad_all(cont, 6, 0);
    lv_obj_set_style_pad_row(cont, 6, 0);
    lv_obj_set_style_pad_column(cont, 6, 0);
    lv_obj_set_layout(cont, LV_LAYOUT_GRID);

    static int32_t col_dsc[] = { 1, 1, LV_GRID_TEMPLATE_LAST };
    static int32_t row_dsc[] = { 1, 1, LV_GRID_TEMPLATE_LAST };
    lv_obj_set_grid_dsc_array(cont, col_dsc, row_dsc);

    // Create 4 buttons
    lv_obj_t *btn_start = lv_btn_create(cont);
    lv_obj_set_grid_cell(btn_start, LV_GRID_ALIGN_STRETCH, 0, 1,
                                   LV_GRID_ALIGN_STRETCH, 0, 1);
    lv_obj_add_event_cb(btn_start, on_btn_start, LV_EVENT_CLICKED, NULL);
    lv_obj_t *lbl = lv_label_create(btn_start);
    lv_label_set_text(lbl, "Start");
    lv_obj_center(lbl);

    lv_obj_t *btn_clean = lv_btn_create(cont);
    lv_obj_set_grid_cell(btn_clean, LV_GRID_ALIGN_STRETCH, 1, 1,
                                   LV_GRID_ALIGN_STRETCH, 0, 1);
    lv_obj_add_event_cb(btn_clean, on_btn_clean, LV_EVENT_CLICKED, NULL);
    lbl = lv_label_create(btn_clean);
    lv_label_set_text(lbl, "Clean");
    lv_obj_center(lbl);

    lv_obj_t *btn_tare = lv_btn_create(cont);
    lv_obj_set_grid_cell(btn_tare, LV_GRID_ALIGN_STRETCH, 0, 1,
                                  LV_GRID_ALIGN_STRETCH, 1, 1);
    lv_obj_add_event_cb(btn_tare, on_btn_tare, LV_EVENT_CLICKED, NULL);
    lbl = lv_label_create(btn_tare);
    lv_label_set_text(lbl, "Tare");
    lv_obj_center(lbl);

    lv_obj_t *btn_recipes = lv_btn_create(cont);
    lv_obj_set_grid_cell(btn_recipes, LV_GRID_ALIGN_STRETCH, 1, 1,
                                     LV_GRID_ALIGN_STRETCH, 1, 1);
    lv_obj_add_event_cb(btn_recipes, on_btn_recipes, LV_EVENT_CLICKED, NULL);
    lbl = lv_label_create(btn_recipes);
    lv_label_set_text(lbl, "Recipes");
    lv_obj_center(lbl);
}

void ui_show_run(void)
{
    lv_obj_t *scr = LVGL_ACTIVE_SCREEN();
    lv_obj_clean(scr);
    lv_obj_set_style_pad_all(scr, 10, 0);
    ui_apply_noscroll(scr);

    lbl_state = lv_label_create(scr);
    lv_label_set_text(lbl_state, "State: ---");
    lv_obj_align(lbl_state, LV_ALIGN_TOP_LEFT, 0, 0);

    lbl_mass = lv_label_create(scr);
    lv_label_set_text(lbl_mass, "Mass: --.- g");
    lv_obj_align(lbl_mass, LV_ALIGN_TOP_LEFT, 0, 30);

    lbl_flow = lv_label_create(scr);
    lv_label_set_text(lbl_flow, "Flow: --.- g/s");
    lv_obj_align(lbl_flow, LV_ALIGN_TOP_LEFT, 0, 60);

    bar_opening = lv_bar_create(scr);
    lv_obj_set_size(bar_opening, 280, 24);
    lv_obj_align(bar_opening, LV_ALIGN_TOP_LEFT, 0, 95);
    lv_bar_set_range(bar_opening, 0, 100);
    lv_bar_set_value(bar_opening, 0, LV_ANIM_OFF);

    // STOP button
    lv_obj_t *btn_abort = lv_btn_create(scr);
    lv_obj_set_size(btn_abort, 140, 60);
    lv_obj_align(btn_abort, LV_ALIGN_BOTTOM_LEFT, 0, 0);
    lv_obj_add_event_cb(btn_abort, on_btn_abort, LV_EVENT_CLICKED, NULL);

    lv_obj_t *lbl = lv_label_create(btn_abort);
    lv_label_set_text(lbl, "STOP");
    lv_obj_center(lbl);

    // HOME button
    lv_obj_t *btn_home = lv_btn_create(scr);
    lv_obj_set_size(btn_home, 140, 60);
    lv_obj_align(btn_home, LV_ALIGN_BOTTOM_RIGHT, 0, 0);
    lv_obj_add_event_cb(btn_home, on_btn_home, LV_EVENT_CLICKED, NULL);

    lbl = lv_label_create(btn_home);
    lv_label_set_text(lbl, "HOME");
    lv_obj_center(lbl);

    //Telemetry update timer (10 Hz)
    if (run_timer) {
        lv_timer_del(run_timer);
        run_timer = NULL;
    }
    run_timer = lv_timer_create(ui_run_tick, 100, NULL);
}

/* ---------------------------
 * Button callbacks
 * --------------------------- */
static void on_btn_start(lv_event_t *e)
{
    (void)e;

    // Show run screen immediately 
    ui_show_run();

    //TEMP arbitrary target
    ctrl_cmd_t cmd = {
        .id = CTRL_CMD_START_DOSE,
        .target_g = 50.0f
    };
    control_send(&cmd);
}

static void on_btn_clean(lv_event_t *e)
{
    (void)e;
    ui_show_run();

    ctrl_cmd_t cmd = { .id = CTRL_CMD_CLEAN };
    control_send(&cmd);
}

static void on_btn_tare(lv_event_t *e)
{
    (void)e;
    ctrl_cmd_t cmd = { .id = CTRL_CMD_TARE };
    control_send(&cmd);

    //ui feedback
    if (lbl_status) lv_label_set_text(lbl_status, "TARE requested…");
}

static void on_btn_recipes(lv_event_t *e)
{
    (void)e;
    //PLACEHOLDER for recipe screen
    if (lbl_status) lv_label_set_text(lbl_status, "Recipes (TODO)");
}

static void on_btn_abort(lv_event_t *e)
{
    (void)e;
    ctrl_cmd_t cmd = { .id = CTRL_CMD_ABORT };
    control_send(&cmd);
}

static void on_btn_home(lv_event_t *e)
{
    (void)e;
    ui_show_home();
}

/* ---------------------------
 * Run telemetry update
 * --------------------------- */
static const char *dose_state_str(dose_state_t s)
{
    switch (s) {
        case DOSE_IDLE:       return "IDLE";
        case DOSE_OPENING:    return "OPENING";
        case DOSE_FULL_FLOW:  return "FULL";
        case DOSE_THROTTLING: return "THROTTLE";
        case DOSE_CLOSING:    return "CLOSING";
        case DOSE_SETTLE:     return "SETTLE";
        case DOSE_DONE:       return "DONE";
        case DOSE_ABORTED:    return "ABORTED";
        case DOSE_FAULT:      return "FAULT";
        default:              return "?";
    }
}

static void ui_run_tick(lv_timer_t *t)
{
    (void)t;

    dose_status_t st = dosing_get_status();

    if (lbl_state) {
        char buf[64];
        snprintf(buf, sizeof(buf), "State: %s", dose_state_str(st.state));
        lv_label_set_text(lbl_state, buf);
    }

    if (lbl_mass) {
        char buf[64];
        snprintf(buf, sizeof(buf), "Mass: %.1f g", (double)st.mass_g);
        lv_label_set_text(lbl_mass, buf);
    }

    if (lbl_flow) {
        char buf[64];
        snprintf(buf, sizeof(buf), "Flow: %.1f g/s", (double)st.flow_gps);
        lv_label_set_text(lbl_flow, buf);
    }

    if (bar_opening) {
        int pct = (int)(st.opening * 100.0f + 0.5f);
        if (pct < 0) pct = 0;
        if (pct > 100) pct = 100;
        lv_bar_set_value(bar_opening, pct, LV_ANIM_OFF);
    }
}

/* ---------------------------
 * Init entry points
 * --------------------------- */
void ui_init(const display_handles_t *disp)
{
    const lvgl_port_cfg_t lvgl_cfg = ESP_LVGL_PORT_INIT_CONFIG();
    ESP_ERROR_CHECK(lvgl_port_init(&lvgl_cfg));

    lvgl_port_display_cfg_t cfg = {0};
    cfg.io_handle = disp->io;
    cfg.panel_handle = disp->panel;
    cfg.hres = disp->hres;
    cfg.vres = disp->vres;

    cfg.buffer_size = cfg.hres * 40;
    cfg.double_buffer = true;
    cfg.monochrome = false;
    cfg.color_format = LV_COLOR_FORMAT_RGB565;

    //display orientation
    cfg.rotation.swap_xy = true;
    cfg.rotation.mirror_x = true;
    cfg.rotation.mirror_y = true;

    cfg.flags.buff_dma = true;
    cfg.flags.swap_bytes = false;

    lv_disp_t *d = lvgl_port_add_disp(&cfg);
    assert(d);

    // Build initial screen (home)
    lvgl_port_lock(0);
    ui_show_home();
    lvgl_port_unlock();
}

void ui_init_after_display_ready(void)
{
    const int SDA = 6;
    const int SCL = 7;

    const uint16_t W = 320;
    const uint16_t H = 240;

    ESP_ERROR_CHECK(touch_ns2009_init(&g_touch, SDA, SCL, 100000, 0x48, W, H));

    //double check touch mapping
    g_touch.swap_xy  = false;  
    g_touch.mirror_x = true;
    g_touch.mirror_y = true;

    lvgl_port_lock(0);
    touch_ns2009_register_lvgl(&g_touch);
    lvgl_port_unlock();
}
