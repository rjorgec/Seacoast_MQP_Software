#include <assert.h>
#include <stdio.h>
#include <string.h>

#include "ui_screens.h"
#include "control.h"

#include "lvgl.h"
#include "esp_log.h"
#include "esp_lvgl_port.h"
#include "motor_hal.h"
#include "pico_link.h"
#include "proto/proto.h"

static const char *TAG = "ui_screens";

#if defined(LVGL_VERSION_MAJOR) && (LVGL_VERSION_MAJOR >= 9)
#define LVGL_ACTIVE_SCREEN() lv_screen_active()
#else
#define LVGL_ACTIVE_SCREEN() lv_scr_act()
#endif

#define JOG_SPEED 2500u
#define JOG_LOW_TH 30u
#define JOG_HIGH_TH 200u
#define JOG_INTERVAL_MS 5u

/* Shared file-scope labels — re-assigned by every ui_show_* call */
static lv_obj_t *lbl_status = NULL; /* updated by ui_status_set() / set_status() */
static lv_obj_t *lbl_weight = NULL; /* weight readout; NULL on screens without it */
static float s_last_weight_g = 0.0f;

/* Operations screen async-update labels — NULL when not on that screen */
static lv_obj_t *s_ops_lbl_status = NULL; /* last motion-done result */
static lv_obj_t *s_ops_lbl_vacuum = NULL; /* vacuum pump status + RPM */

/* Dosing screen async-update labels — NULL when not on that screen */
static lv_obj_t *s_dose_lbl_status = NULL;   /* spawn status string */
static lv_obj_t *s_dose_lbl_progress = NULL; /* "X.XX g / Y.YY g" */
static lv_obj_t *s_dose_lbl_retries = NULL;  /* "Retries: N  Bag: N" */
static lv_obj_t *s_dose_bar = NULL;          /* progress bar 0..1000 */
static lv_obj_t *s_dose_lbl_innoc = NULL;    /* inoculation % display */
static lv_obj_t *s_dose_lbl_style = NULL;    /* dose style display */
static uint32_t s_dose_target_ug = 0;
static uint16_t s_dose_innoc_pct = 200; /* x10 → default 20.0 % */
static uint8_t s_dose_style = DOSE_STYLE_A;
static uint8_t s_dose_bag_number = 0;

/* ── Status helpers ─────────────────────────────────────────────────────── */

void ui_status_set(const char *s)
{
    if (!s)
        s = "";
    lvgl_port_lock(0);
    if (lbl_status)
        lv_label_set_text(lbl_status, s);
    lvgl_port_unlock();
}

static void set_status(const char *s)
{
    ui_status_set(s);
}

/* ── Home-screen button callbacks ────────────────────────────────────────── */

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

static void on_dose(lv_event_t *e)
{
    (void)e;
    ui_show_dosing();
    ESP_LOGI(TAG, "DOSE pressed");
}

static void on_home(lv_event_t *e)
{
    (void)e;
    ui_show_home();
    ESP_LOGI(TAG, "HOME pressed");
}

static void on_ops_page(lv_event_t *e)
{
    (void)e;
    ui_show_operations();
    ESP_LOGI(TAG, "Operations pressed");
}

static void on_tare(lv_event_t *e)
{
    (void)e;
    uint8_t nack_code = 0;
    esp_err_t err = pico_link_send_rpc(MSG_HX711_TARE,
                                       NULL, 0,
                                       1000, /* 1 s timeout */
                                       &nack_code);
    if (err == ESP_OK)
    {
        s_last_weight_g = 0.0f;
        set_status("TARE successful");
        ESP_LOGI(TAG, "HX711 Tare successful");
    }
    else
    {
        char buf[64];
        snprintf(buf, sizeof(buf), "TARE failed: %s (nack=%u)",
                 esp_err_to_name(err), nack_code);
        set_status(buf);
        ESP_LOGW(TAG, "HX711 Tare failed (%s), nack=%u",
                 esp_err_to_name(err), nack_code);
    }
}

static void on_read_weight(lv_event_t *e)
{
    (void)e;
    pl_hx711_measure_t measure_payload = {
        .interval_us = 1000000 /* 1 s interval */
    };
    uint16_t seq = 0;
    esp_err_t err = pico_link_send(MSG_HX711_MEASURE,
                                   &measure_payload,
                                   sizeof(measure_payload),
                                   &seq);
    if (err == ESP_OK)
    {
        set_status("Weight request sent...");
        ESP_LOGI(TAG, "Weight request sent (seq=%u)", seq);
    }
    else
    {
        char buf[64];
        snprintf(buf, sizeof(buf), "Weight req failed: %s",
                 esp_err_to_name(err));
        set_status(buf);
        ESP_LOGW(TAG, "Weight request failed (%s)", esp_err_to_name(err));
    }
}

static void on_start(lv_event_t *e)
{
    (void)e;
    ctrl_cmd_t cmd = {
        .type = CTRL_CMD_START,
        .target_g = 50.0f,
        .recipe_id = 0,
    };
    bool ok = control_send(&cmd);
    set_status(ok ? "START sent" : "START failed");
    ESP_LOGI(TAG, "Start pressed (send=%d)", (int)ok);
}

static void on_pause(lv_event_t *e)
{
    (void)e;
    ctrl_cmd_t cmd = {.type = CTRL_CMD_PAUSE};
    bool ok = control_send(&cmd);
    set_status(ok ? "PAUSE sent" : "PAUSE failed");
    ESP_LOGI(TAG, "Pause pressed (send=%d)", (int)ok);
}

static void on_stop(lv_event_t *e)
{
    (void)e;
    ctrl_cmd_t cmd = {.type = CTRL_CMD_STOP};
    bool ok = control_send(&cmd);
    set_status(ok ? "STOP sent" : "STOP failed");
    ESP_LOGI(TAG, "Stop pressed (send=%d)", (int)ok);
}

static void on_stop_linact(lv_event_t *e)
{
    (void)e;
    esp_err_t err = motor_linact_stop_monitor();
    set_status(err == ESP_OK ? "LINACT: STOP" : "LINACT: STOP FAILED");
    ESP_LOGI(TAG, "STOP pressed (%s)", esp_err_to_name(err));
}

/* ── Operations-screen button callbacks ──────────────────────────────────── */

static void on_flap_open(lv_event_t *e)
{
    (void)e;
    esp_err_t err = motor_flap_open();
    set_status(err == ESP_OK ? "Flaps: opening..." : "Flap open: FAILED");
    ESP_LOGI(TAG, "Flap open (%s)", esp_err_to_name(err));
}

static void on_flap_close(lv_event_t *e)
{
    (void)e;
    esp_err_t err = motor_flap_close();
    set_status(err == ESP_OK ? "Flaps: closing..." : "Flap close: FAILED");
    ESP_LOGI(TAG, "Flap close (%s)", esp_err_to_name(err));
}

static void on_arm_press(lv_event_t *e)
{
    (void)e;
    esp_err_t err = motor_arm_move((uint8_t)ARM_POS_PRESS);
    set_status(err == ESP_OK ? "Arm: to PRESS..." : "Arm press: FAILED");
    ESP_LOGI(TAG, "Arm press (%s)", esp_err_to_name(err));
}

static void on_arm_pos1(lv_event_t *e)
{
    (void)e;
    esp_err_t err = motor_arm_move((uint8_t)ARM_POS_1);
    set_status(err == ESP_OK ? "Arm: to POS1..." : "Arm pos1: FAILED");
    ESP_LOGI(TAG, "Arm pos1 (%s)", esp_err_to_name(err));
}

static void on_arm_pos2(lv_event_t *e)
{
    (void)e;
    esp_err_t err = motor_arm_move((uint8_t)ARM_POS_2);
    set_status(err == ESP_OK ? "Arm: to POS2..." : "Arm pos2: FAILED");
    ESP_LOGI(TAG, "Arm pos2 (%s)", esp_err_to_name(err));
}

static void on_rack_home(lv_event_t *e)
{
    (void)e;
    esp_err_t err = motor_rack_move((uint8_t)RACK_POS_HOME);
    set_status(err == ESP_OK ? "Rack: homing..." : "Rack home: FAILED");
    ESP_LOGI(TAG, "Rack home (%s)", esp_err_to_name(err));
}

static void on_rack_extend(lv_event_t *e)
{
    (void)e;
    esp_err_t err = motor_rack_move((uint8_t)RACK_POS_EXTEND);
    set_status(err == ESP_OK ? "Rack: extending..." : "Rack extend: FAILED");
    ESP_LOGI(TAG, "Rack extend (%s)", esp_err_to_name(err));
}

static void on_rack_press(lv_event_t *e)
{
    (void)e;
    esp_err_t err = motor_rack_move((uint8_t)RACK_POS_PRESS);
    set_status(err == ESP_OK ? "Rack: to PRESS..." : "Rack press: FAILED");
    ESP_LOGI(TAG, "Rack press (%s)", esp_err_to_name(err));
}

static void on_turntable_home(lv_event_t *e)
{
    (void)e;
    esp_err_t err = motor_turntable_home();
    set_status(err == ESP_OK ? "Tbl: homing..." : "Tbl home: FAILED");
    ESP_LOGI(TAG, "Turntable home (%s)", esp_err_to_name(err));
}

static void on_turntable_a(lv_event_t *e)
{
    (void)e;
    esp_err_t err = motor_turntable_goto((uint8_t)TURNTABLE_POS_A);
    set_status(err == ESP_OK ? "Tbl: to A..." : "Tbl A: FAILED");
    ESP_LOGI(TAG, "Turntable A (%s)", esp_err_to_name(err));
}

static void on_turntable_b(lv_event_t *e)
{
    (void)e;
    esp_err_t err = motor_turntable_goto((uint8_t)TURNTABLE_POS_B);
    set_status(err == ESP_OK ? "Tbl: to B..." : "Tbl B: FAILED");
    ESP_LOGI(TAG, "Turntable B (%s)", esp_err_to_name(err));
}

static void on_turntable_c(lv_event_t *e)
{
    (void)e;
    esp_err_t err = motor_turntable_goto((uint8_t)TURNTABLE_POS_C);
    set_status(err == ESP_OK ? "Tbl: to C..." : "Tbl C: FAILED");
    ESP_LOGI(TAG, "Turntable C (%s)", esp_err_to_name(err));
}

static void on_turntable_d(lv_event_t *e)
{
    (void)e;
    esp_err_t err = motor_turntable_goto((uint8_t)TURNTABLE_POS_D);
    set_status(err == ESP_OK ? "Tbl: to D..." : "Tbl D: FAILED");
    ESP_LOGI(TAG, "Turntable D (%s)", esp_err_to_name(err));
}

static void on_hotwire_on(lv_event_t *e)
{
    (void)e;
    esp_err_t err = motor_hotwire_set(true);
    set_status(err == ESP_OK ? "Wire: ON" : "Wire ON: FAILED");
    ESP_LOGI(TAG, "Hotwire on (%s)", esp_err_to_name(err));
}

static void on_hotwire_off(lv_event_t *e)
{
    (void)e;
    esp_err_t err = motor_hotwire_set(false);
    set_status(err == ESP_OK ? "Wire: OFF" : "Wire OFF: FAILED");
    ESP_LOGI(TAG, "Hotwire off (%s)", esp_err_to_name(err));
}

static void on_vacuum_on(lv_event_t *e)
{
    (void)e;
    esp_err_t err = motor_vacuum_set(true);
    set_status(err == ESP_OK ? "Vac: ON" : "Vac ON: FAILED");
    ESP_LOGI(TAG, "Vacuum on (%s)", esp_err_to_name(err));
}

static void on_vacuum_off(lv_event_t *e)
{
    (void)e;
    esp_err_t err = motor_vacuum_set(false);
    set_status(err == ESP_OK ? "Vac: OFF" : "Vac OFF: FAILED");
    ESP_LOGI(TAG, "Vacuum off (%s)", esp_err_to_name(err));
}

static void on_vacuum2_on(lv_event_t *e)
{
    (void)e;
    ESP_LOGI(TAG, "Vacuum2 ON");
    esp_err_t err = motor_vacuum2_set(true);
    set_status(err == ESP_OK ? "Vac2: ON" : "Vac2 ON failed");
}

static void on_vacuum2_off(lv_event_t *e)
{
    (void)e;
    ESP_LOGI(TAG, "Vacuum2 OFF");
    esp_err_t err = motor_vacuum2_set(false);
    set_status(err == ESP_OK ? "Vac2: OFF" : "Vac2 OFF failed");
}

/* ── Pico RX handler (HX711 weight display) ──────────────────────────────── */

void ui_screens_pico_rx_handler(uint8_t msg_type, uint16_t seq,
                                const uint8_t *payload, uint16_t len)
{
    (void)seq;

    if (msg_type == MSG_HX711_MEASURE && len == sizeof(pl_hx711_mass_t))
    {
        pl_hx711_mass_t mass_data;
        memcpy(&mass_data, payload, sizeof(mass_data));

        float weight_g = (float)mass_data.mass_ug / 1000000.0f;
        s_last_weight_g = weight_g;
        ESP_LOGI(TAG, "Received weight: %.3f g (unit=%u)", weight_g,
                 mass_data.unit);

        if (lbl_weight)
        {
            char weight_str[32];
            snprintf(weight_str, sizeof(weight_str), "Weight: %.2f g",
                     weight_g);
            lvgl_port_lock(0);
            lv_label_set_text(lbl_weight, weight_str);
            lvgl_port_unlock();
        }

        char status_buf[64];
        snprintf(status_buf, sizeof(status_buf), "Weight: %.2f g", weight_g);
        set_status(status_buf);
    }
}

/* ── Operations screen — async motion / vacuum handlers ──────────────────── */

void ui_ops_on_motion_done(const pl_motion_done_t *pl)
{
    if (!pl || !s_ops_lbl_status)
        return;

    const char *sub_name;
    switch ((subsystem_id_t)pl->subsystem)
    {
    case SUBSYS_FLAPS:
        sub_name = "Flaps";
        break;
    case SUBSYS_ARM:
        sub_name = "Arm";
        break;
    case SUBSYS_RACK:
        sub_name = "Rack";
        break;
    case SUBSYS_TURNTABLE:
        sub_name = "Turntable";
        break;
    case SUBSYS_HOTWIRE:
        sub_name = "HotWire";
        break;
    case SUBSYS_VACUUM:
        sub_name = "Vacuum";
        break;
    default:
        sub_name = "Unknown";
        break;
    }

    const char *res_name;
    switch ((motion_result_t)pl->result)
    {
    case MOTION_OK:
        res_name = "OK";
        break;
    case MOTION_STALLED:
        res_name = "STALLED";
        break;
    case MOTION_TIMEOUT:
        res_name = "TIMEOUT";
        break;
    case MOTION_FAULT:
        res_name = "FAULT";
        break;
    default:
        res_name = "?";
        break;
    }

    char buf[48];
    snprintf(buf, sizeof(buf), "%s: %s", sub_name, res_name);

    lvgl_port_lock(0);
    lv_label_set_text(s_ops_lbl_status, buf);
    lvgl_port_unlock();

    ESP_LOGI(TAG, "Motion done: %s (steps=%ld)", buf, (long)pl->steps_done);
}

void ui_ops_on_vacuum_status(const pl_vacuum_status_t *pl)
{
    if (!pl || !s_ops_lbl_vacuum)
        return;

    char buf[40];
    lv_color_t col;

    switch ((vacuum_status_code_t)pl->status)
    {
    case VACUUM_OK:
        snprintf(buf, sizeof(buf), "Vac: OK %u RPM", (unsigned)pl->rpm);
        col = lv_color_hex(0x00AA44); /* green */
        break;
    case VACUUM_BLOCKED:
        snprintf(buf, sizeof(buf), "Vac: BLOCKED! %u RPM",
                 (unsigned)pl->rpm);
        col = lv_color_hex(0xCC2200); /* red */
        break;
    case VACUUM_OFF:
    default:
        snprintf(buf, sizeof(buf), "Vac: OFF");
        col = lv_color_hex(0xFFFFFF); /* default white */
        break;
    }

    lvgl_port_lock(0);
    lv_label_set_text(s_ops_lbl_vacuum, buf);
    lv_obj_set_style_text_color(s_ops_lbl_vacuum, col, 0);
    lvgl_port_unlock();

    ESP_LOGI(TAG, "Vacuum status: %s", buf);
}

/* ── UI helpers ──────────────────────────────────────────────────────────── */

/** Create a full-featured grid button (for Home screen grid layout). */
static lv_obj_t *make_big_btn(lv_obj_t *parent, const char *txt,
                              lv_event_cb_t cb)
{
    lv_obj_t *btn = lv_btn_create(parent);
    lv_obj_add_event_cb(btn, cb, LV_EVENT_CLICKED, NULL);

    lv_obj_t *label = lv_label_create(btn);
    lv_label_set_text(label, txt);
    lv_obj_center(label);

    return btn;
}

/**
 * Create a button at an explicit absolute position within the screen's
 * padded content area. Uses LV_ALIGN_TOP_LEFT + (x, y) offsets.
 */
static lv_obj_t *make_btn(lv_obj_t *scr, const char *txt,
                          lv_coord_t x, lv_coord_t y,
                          lv_coord_t w, lv_coord_t h,
                          lv_event_cb_t cb)
{
    lv_obj_t *btn = lv_btn_create(scr);
    lv_obj_set_size(btn, w, h);
    lv_obj_align(btn, LV_ALIGN_TOP_LEFT, x, y);
    lv_obj_add_event_cb(btn, cb, LV_EVENT_CLICKED, NULL);

    lv_obj_t *lbl = lv_label_create(btn);
    lv_label_set_text(lbl, txt);
    lv_obj_center(lbl);

    return btn;
}

/* ── Screen builders ─────────────────────────────────────────────────────── */

void ui_show_home(void)
{
    lv_obj_t *scr = LVGL_ACTIVE_SCREEN();
    lv_obj_clean(scr);

    /* Nullify ops-screen labels so async handlers don't touch freed widgets */
    s_ops_lbl_status = NULL;
    s_ops_lbl_vacuum = NULL;

    lv_obj_clear_flag(scr, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_style_pad_all(scr, 10, 0);

    /* Status strip */
    lbl_status = lv_label_create(scr);
    lv_label_set_text(lbl_status, "IDLE \xe2\x80\xa2 Seacoast Inoculator");
    lv_obj_align(lbl_status, LV_ALIGN_TOP_LEFT, 0, 0);

    /* 2-column × 3-row button grid */
    lv_obj_t *cont = lv_obj_create(scr);
    lv_obj_clear_flag(cont, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_size(cont, 300, 210);
    lv_obj_align(cont, LV_ALIGN_BOTTOM_MID, 0, 0);

    lv_obj_set_style_pad_all(cont, 6, 0);
    lv_obj_set_style_pad_row(cont, 6, 0);
    lv_obj_set_style_pad_column(cont, 6, 0);
    lv_obj_set_layout(cont, LV_LAYOUT_GRID);

    static lv_coord_t col_dsc[] = {
        LV_GRID_FR(1), LV_GRID_FR(1), LV_GRID_TEMPLATE_LAST};
    static lv_coord_t row_dsc[] = {
        LV_GRID_FR(1), LV_GRID_FR(1), LV_GRID_FR(1), LV_GRID_TEMPLATE_LAST};
    lv_obj_set_grid_dsc_array(cont, col_dsc, row_dsc);

    lv_obj_t *b_fwd = make_big_btn(cont, "Forward", on_fwd);
    lv_obj_t *b_rev = make_big_btn(cont, "Backward", on_rev);
    lv_obj_t *b_dose = make_big_btn(cont, "Dose", on_dose);
    lv_obj_t *b_tare = make_big_btn(cont, "Tare", on_tare);
    lv_obj_t *b_ops = make_big_btn(cont, "Operations", on_ops_page);

    lv_obj_set_grid_cell(b_fwd, LV_GRID_ALIGN_STRETCH, 0, 1,
                         LV_GRID_ALIGN_STRETCH, 0, 1);
    lv_obj_set_grid_cell(b_rev, LV_GRID_ALIGN_STRETCH, 1, 1,
                         LV_GRID_ALIGN_STRETCH, 0, 1);
    lv_obj_set_grid_cell(b_dose, LV_GRID_ALIGN_STRETCH, 0, 1,
                         LV_GRID_ALIGN_STRETCH, 1, 1);
    lv_obj_set_grid_cell(b_tare, LV_GRID_ALIGN_STRETCH, 1, 1,
                         LV_GRID_ALIGN_STRETCH, 1, 1);
    /* Operations spans both columns in row 2 for a wide, easy touch target */
    lv_obj_set_grid_cell(b_ops, LV_GRID_ALIGN_STRETCH, 0, 2,
                         LV_GRID_ALIGN_STRETCH, 2, 1);
}

/*
 * Operations Screen Layout  (320 × 240, landscape, 10 px pad → 300 × 220 content)
 *
 *  y=  0  "Operations"  title label (TOP_LEFT)    [Home 60×20, TOP_RIGHT]
 *  y= 22  [Flap Open  148×24]  4  [Flap Close  148×24]
 *  y= 50  [Arm Press  96×24]  4  [Arm Pos 1  96×24]  4  [Arm Pos 2  96×24]
 *  y= 78  [Rack Home  96×24]  4  [Rack Ext   96×24]  4  [Rack Press 96×24]
 *  y=106  [TblHome 56×24] 4 [TblA 56×24] 4 [TblB 56×24] 4 [TblC 56×24] 4 [TblD 56×24]
 *  y=134  [Wire ON  148×24]  4  [Wire OFF  148×24]
 *  y=162  [Vac ON   148×24]  4  [Vac OFF   148×24]
 *  y=188  s_ops_lbl_status  (motion-done text)
 *  y=204  s_ops_lbl_vacuum  (vacuum status text)
 *
 *  Button-row widths:
 *    2-btn: 148+4+148 = 300 px
 *    3-btn: 96+4+96+4+96 = 296 px
 *    5-btn: 56×5 + 4×4 = 296 px   (positions: x = 0, 60, 120, 180, 240)
 */
void ui_show_operations(void)
{
    lv_obj_t *scr = LVGL_ACTIVE_SCREEN();
    lv_obj_clean(scr);
    lv_obj_clear_flag(scr, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_style_pad_all(scr, 10, 0);

    /* Weight label not present on this screen */
    lbl_weight = NULL;

    /* ── Title label ──────────────────────────────────────────────────────── */
    lv_obj_t *lbl_title = lv_label_create(scr);
    lv_label_set_text(lbl_title, "Operations");
    lv_obj_align(lbl_title, LV_ALIGN_TOP_LEFT, 0, 2);

    /* ── Home nav button (top-right) ──────────────────────────────────────── */
    lv_obj_t *btn_nav_home = lv_btn_create(scr);
    lv_obj_set_size(btn_nav_home, 60, 20);
    lv_obj_align(btn_nav_home, LV_ALIGN_TOP_RIGHT, 0, 0);
    lv_obj_add_event_cb(btn_nav_home, on_home, LV_EVENT_CLICKED, NULL);
    lv_obj_t *l_nav_home = lv_label_create(btn_nav_home);
    lv_label_set_text(l_nav_home, "Home");
    lv_obj_center(l_nav_home);

    /* ── Row 1: Flaps  (y=22, h=24) ──────────────────────────────────────── */
    make_btn(scr, "Flaps Open", 0, 22, 148, 24, on_flap_open);
    make_btn(scr, "Flaps Close", 152, 22, 148, 24, on_flap_close);

    /* ── Row 2: Arm  (y=50, h=24) ────────────────────────────────────────── */
    make_btn(scr, "Arm Press", 0, 50, 96, 24, on_arm_press);
    make_btn(scr, "Arm Pos 1", 100, 50, 96, 24, on_arm_pos1);
    make_btn(scr, "Arm Pos 2", 200, 50, 96, 24, on_arm_pos2);

    /* ── Row 3: Rack  (y=78, h=24) ───────────────────────────────────────── */
    make_btn(scr, "Rack Home", 0, 78, 96, 24, on_rack_home);
    make_btn(scr, "Rack Extend", 100, 78, 96, 24, on_rack_extend);
    make_btn(scr, "Rack Press", 200, 78, 96, 24, on_rack_press);

    /* ── Row 4: Turntable  (y=106, h=24)  5 × 56 px + 4 × 4 px = 296 px ── */
    make_btn(scr, "Tbl Home", 0, 106, 56, 24, on_turntable_home);
    make_btn(scr, "Tbl A", 60, 106, 56, 24, on_turntable_a);
    make_btn(scr, "Tbl B", 120, 106, 56, 24, on_turntable_b);
    make_btn(scr, "Tbl C", 180, 106, 56, 24, on_turntable_c);
    make_btn(scr, "Tbl D", 240, 106, 56, 24, on_turntable_d);

    /* ── Row 5: Hot Wire  (y=134, h=24) ──────────────────────────────────── */
    make_btn(scr, "Wire ON", 0, 134, 148, 24, on_hotwire_on);
    make_btn(scr, "Wire OFF", 152, 134, 148, 24, on_hotwire_off);

    /* ── Row 6: Vacuum  (y=162, h=24)  4 × 72 px + 3 × 4 px = 300 px ────── */
    make_btn(scr, "Vac ON", 0, 162, 72, 24, on_vacuum_on);
    make_btn(scr, "Vac OFF", 76, 162, 72, 24, on_vacuum_off);
    make_btn(scr, "Vac2 ON", 152, 162, 72, 24, on_vacuum2_on);
    make_btn(scr, "Vac2 OFF", 228, 162, 72, 24, on_vacuum2_off);

    /* ── Status labels  (y=188, y=204) ───────────────────────────────────── */
    s_ops_lbl_status = lv_label_create(scr);
    lv_label_set_text(s_ops_lbl_status, "Status: --");
    lv_obj_set_style_text_font(s_ops_lbl_status, &lv_font_montserrat_14, 0);
    lv_obj_align(s_ops_lbl_status, LV_ALIGN_TOP_LEFT, 0, 188);

    s_ops_lbl_vacuum = lv_label_create(scr);
    lv_label_set_text(s_ops_lbl_vacuum, "Vac: --");
    lv_obj_set_style_text_font(s_ops_lbl_vacuum, &lv_font_montserrat_14, 0);
    lv_obj_align(s_ops_lbl_vacuum, LV_ALIGN_TOP_LEFT, 0, 204);

    /*
     * Point lbl_status at s_ops_lbl_status so that set_status() (called by
     * button callbacks) updates the same label that ui_ops_on_motion_done()
     * uses for async notifications.
     */
    lbl_status = s_ops_lbl_status;
}

/* ── Dosing screen callbacks ─────────────────────────────────────────────── */

static void dose_update_innoc_label(void)
{
    if (!s_dose_lbl_innoc)
        return;
    char buf[24];
    snprintf(buf, sizeof(buf), "Innoc: %u.%u%%",
             s_dose_innoc_pct / 10u, s_dose_innoc_pct % 10u);
    lv_label_set_text(s_dose_lbl_innoc, buf);
}

static void dose_update_style_label(void)
{
    if (!s_dose_lbl_style)
        return;
    lv_label_set_text(s_dose_lbl_style,
                      (s_dose_style == DOSE_STYLE_B) ? "Dose Style: B" : "Dose Style: A");
}

static void on_innoc_minus(lv_event_t *e)
{
    (void)e;
    if (s_dose_innoc_pct > 10u)
        s_dose_innoc_pct -= 10u; /* − 1.0 % */
    dose_update_innoc_label();
}

static void on_innoc_plus(lv_event_t *e)
{
    (void)e;
    if (s_dose_innoc_pct < 500u)
        s_dose_innoc_pct += 10u; /* + 1.0 % */
    dose_update_innoc_label();
}

static void on_dose_style_a(lv_event_t *e)
{
    (void)e;
    s_dose_style = DOSE_STYLE_A;
    dose_update_style_label();
}

static void on_dose_style_b(lv_event_t *e)
{
    (void)e;
    s_dose_style = DOSE_STYLE_B;
    dose_update_style_label();
}

static void on_dose_start(lv_event_t *e)
{
    (void)e;

    pl_innoculate_bag_t pl = {
        .bag_mass = 0,      /* unused by Pico — scale read internally */
        .spawn_mass = 1000, /* 1 kg remaining (informational) */
        .innoc_percent = s_dose_innoc_pct,
        .dose_style = s_dose_style,
        .bag_number = s_dose_bag_number,
    };

    if (s_dose_lbl_status)
    {
        lvgl_port_lock(0);
        lv_label_set_text(s_dose_lbl_status, "Starting\xe2\x80\xa6");
        lv_obj_set_style_text_color(s_dose_lbl_status,
                                    lv_color_hex(0xFFFFFF), 0);
        lvgl_port_unlock();
    }

    uint8_t nack = 0;
    esp_err_t err = pico_link_send_rpc(MSG_DISPENSE_SPAWN,
                                       &pl, sizeof(pl),
                                       2000, &nack);
    if (err == ESP_OK)
    {
        s_dose_bag_number++;
        set_status("Dose running\xe2\x80\xa6");
        ESP_LOGI(TAG, "MSG_DISPENSE_SPAWN sent (innoc=%u, style=%u, bag=%u)",
                 s_dose_innoc_pct, s_dose_style, pl.bag_number);
    }
    else
    {
        set_status("Start FAILED");
        ESP_LOGW(TAG, "MSG_DISPENSE_SPAWN failed: %s nack=%u",
                 esp_err_to_name(err), nack);
    }
}

static void on_dose_abort(lv_event_t *e)
{
    (void)e;
    /* Close flaps to stop material flow immediately; the Pico's spawn timer
     * will exhaust its agitation retries and send SPAWN_STATUS_BAG_EMPTY. */
    esp_err_t err = motor_flap_close();
    set_status(err == ESP_OK ? "Aborting\xe2\x80\xa6" : "Abort FAILED");
    ESP_LOGI(TAG, "Dose abort (%s)", esp_err_to_name(err));
}

/* ── Dosing screen — spawn-status async update ───────────────────────────── */

void ui_dosing_on_spawn_status(const pl_spawn_status_t *pl)
{
    if (!pl)
        return;

    /* Map status code to a human-readable string and display colour. */
    const char *status_str;
    lv_color_t col;

    switch ((spawn_status_code_t)pl->status)
    {
    case SPAWN_STATUS_RUNNING:
        status_str = "Running\xe2\x80\xa6";
        col = lv_color_hex(0xFFFFFF);
        break;
    case SPAWN_STATUS_AGITATING:
        status_str = "Agitating\xe2\x80\xa6";
        col = lv_color_hex(0xFFAA00); /* amber */
        break;
    case SPAWN_STATUS_DONE:
        status_str = "Done \xe2\x9c\x93";
        col = lv_color_hex(0x00CC44); /* green */
        break;
    case SPAWN_STATUS_BAG_EMPTY:
        status_str = "Bag empty";
        col = lv_color_hex(0xFF8800); /* orange */
        break;
    case SPAWN_STATUS_FLOW_FAILURE:
        status_str = "Flow failure!";
        col = lv_color_hex(0xFF2200); /* red */
        break;
    case SPAWN_STATUS_STALLED:
        status_str = "Stalled";
        col = lv_color_hex(0xFF8800);
        break;
    case SPAWN_STATUS_ERROR:
    default:
        status_str = "Error!";
        col = lv_color_hex(0xFF0000); /* bright red */
        break;
    }

    /* Latch target mass for progress bar scaling. */
    if (pl->target_ug > 0u)
        s_dose_target_ug = pl->target_ug;

    lvgl_port_lock(0);

    if (s_dose_lbl_status)
    {
        lv_label_set_text(s_dose_lbl_status, status_str);
        lv_obj_set_style_text_color(s_dose_lbl_status, col, 0);
    }

    if (s_dose_lbl_progress)
    {
        char buf[48];
        float disp_g = (float)pl->disp_ug / 1000000.0f;
        float target_g = (float)pl->target_ug / 1000000.0f;
        snprintf(buf, sizeof(buf), "%.2f g  /  %.2f g", disp_g, target_g);
        lv_label_set_text(s_dose_lbl_progress, buf);
    }

    if (s_dose_lbl_retries)
    {
        char buf[40];
        snprintf(buf, sizeof(buf), "Retries: %u   Bag: %u",
                 (unsigned)pl->retries, (unsigned)pl->bag_number);
        lv_label_set_text(s_dose_lbl_retries, buf);
    }

    if (s_dose_bar && s_dose_target_ug > 0u)
    {
        int32_t pct = (int32_t)(((uint64_t)pl->disp_ug * 1000ULL) /
                                s_dose_target_ug);
        if (pct > 1000)
            pct = 1000;
        lv_bar_set_value(s_dose_bar, pct, LV_ANIM_OFF);
    }

    lvgl_port_unlock();

    ESP_LOGI(TAG, "Spawn status: %s  disp=%lu ug  target=%lu ug",
             status_str, (unsigned long)pl->disp_ug,
             (unsigned long)pl->target_ug);
}

/* ── Dosing screen builder ────────────────────────────────────────────────── *
 *                                                                              *
 *  Layout  (320 × 240, landscape, pad=10 → 300 × 220 content area):           *
 *    y=  0  "Spawn Dosing"  title                [Home  60×20  TOP_RIGHT]      *
 *    y= 25  status label  (colour-coded per spawn_status_code_t)               *
 *    y= 48  progress label  "X.XX g / Y.YY g"                                 *
 *    y= 68  retries label   "Retries: N   Bag: N"                              *
 *    y= 88  progress bar    300 × 18                                           *
 *    y=115  innoc label 90×24  [−  55×24]  [+  55×24]                         *
 *    y=142  dose style label + [A 40×24] [B 40×24]                             *
 *    y=170  [Start Dose 145×47]  4  [Abort 145×47]  (to bottom)                *
 * ─────────────────────────────────────────────────────────────────────────── */
void ui_show_dosing(void)
{
    lv_obj_t *scr = LVGL_ACTIVE_SCREEN();
    lv_obj_clean(scr);

    /* Nullify all async-update handles so stale pointer writes can't crash. */
    s_ops_lbl_status = NULL;
    s_ops_lbl_vacuum = NULL;
    s_dose_lbl_status = NULL;
    s_dose_lbl_progress = NULL;
    s_dose_lbl_retries = NULL;
    s_dose_bar = NULL;
    s_dose_lbl_innoc = NULL;
    s_dose_lbl_style = NULL;
    s_dose_target_ug = 0;
    lbl_weight = NULL;

    lv_obj_clear_flag(scr, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_style_pad_all(scr, 10, 0);

    /* ── Title ───────────────────────────────────────────────────────────── */
    lv_obj_t *lbl_title = lv_label_create(scr);
    lv_label_set_text(lbl_title, "Spawn Dosing");
    lv_obj_align(lbl_title, LV_ALIGN_TOP_LEFT, 0, 2);

    /* ── Home nav (top-right) ────────────────────────────────────────────── */
    lv_obj_t *btn_nav = lv_btn_create(scr);
    lv_obj_set_size(btn_nav, 60, 20);
    lv_obj_align(btn_nav, LV_ALIGN_TOP_RIGHT, 0, 0);
    lv_obj_add_event_cb(btn_nav, on_home, LV_EVENT_CLICKED, NULL);
    lv_obj_t *l_nav = lv_label_create(btn_nav);
    lv_label_set_text(l_nav, "Home");
    lv_obj_center(l_nav);

    /* ── Status label (y=25) ─────────────────────────────────────────────── */
    s_dose_lbl_status = lv_label_create(scr);
    lv_label_set_text(s_dose_lbl_status, "Ready");
    lv_obj_set_style_text_font(s_dose_lbl_status, &lv_font_montserrat_14, 0);
    lv_obj_set_style_text_color(s_dose_lbl_status,
                                lv_color_hex(0xFFFFFF), 0);
    lv_obj_align(s_dose_lbl_status, LV_ALIGN_TOP_LEFT, 0, 25);
    /* lbl_status also points here so set_status() updates it from callbacks */
    lbl_status = s_dose_lbl_status;

    /* ── Progress label (y=48) ───────────────────────────────────────────── */
    s_dose_lbl_progress = lv_label_create(scr);
    lv_label_set_text(s_dose_lbl_progress, "0.00 g  /  -- g");
    lv_obj_set_style_text_font(s_dose_lbl_progress,
                               &lv_font_montserrat_14, 0);
    lv_obj_align(s_dose_lbl_progress, LV_ALIGN_TOP_LEFT, 0, 48);

    /* ── Retries label (y=68) ────────────────────────────────────────────── */
    s_dose_lbl_retries = lv_label_create(scr);
    lv_label_set_text(s_dose_lbl_retries, "Retries: 0   Bag: 0");
    lv_obj_set_style_text_font(s_dose_lbl_retries,
                               &lv_font_montserrat_14, 0);
    lv_obj_align(s_dose_lbl_retries, LV_ALIGN_TOP_LEFT, 0, 68);

    /* ── Progress bar (y=88) ─────────────────────────────────────────────── */
    s_dose_bar = lv_bar_create(scr);
    lv_obj_set_size(s_dose_bar, 300, 18);
    lv_obj_align(s_dose_bar, LV_ALIGN_TOP_LEFT, 0, 88);
    lv_bar_set_range(s_dose_bar, 0, 1000);
    lv_bar_set_value(s_dose_bar, 0, LV_ANIM_OFF);

    /* ── Inoculation % row (y=115): label + − + buttons ─────────────────── */
    s_dose_lbl_innoc = lv_label_create(scr);
    lv_obj_set_style_text_font(s_dose_lbl_innoc,
                               &lv_font_montserrat_14, 0);
    lv_obj_align(s_dose_lbl_innoc, LV_ALIGN_TOP_LEFT, 0, 115);
    dose_update_innoc_label();

    make_btn(scr, " - ", 100, 113, 55, 24, on_innoc_minus);
    make_btn(scr, " + ", 159, 113, 55, 24, on_innoc_plus);

    /* ── Dose style row (y=142): label + A/B buttons ─────────────────────── */
    s_dose_lbl_style = lv_label_create(scr);
    lv_obj_set_style_text_font(s_dose_lbl_style,
                               &lv_font_montserrat_14, 0);
    lv_obj_align(s_dose_lbl_style, LV_ALIGN_TOP_LEFT, 0, 142);
    dose_update_style_label();

    make_btn(scr, "A", 150, 140, 40, 24, on_dose_style_a);
    make_btn(scr, "B", 194, 140, 40, 24, on_dose_style_b);

    /* ── Start / Abort row (y=170) ───────────────────────────────────────── */
    make_btn(scr, "Start Dose", 0, 170, 145, 47, on_dose_start);
    make_btn(scr, "Abort", 155, 170, 145, 47, on_dose_abort);
}
