#include <assert.h>
#include <stdio.h>
#include <string.h>

#include "ui_screens.h"
#include "sys_sequence.h"

#include "lvgl.h"
#include "esp_log.h"
#include "esp_lvgl_port.h"
#include "motor_hal.h"
#include "pico_link.h"
#include "proto.h"

static const char *TAG = "ui_screens";

#if defined(LVGL_VERSION_MAJOR) && (LVGL_VERSION_MAJOR >= 9)
#define LVGL_ACTIVE_SCREEN() lv_screen_active()
#else
#define LVGL_ACTIVE_SCREEN() lv_scr_act()
#endif

/* Shared file-scope labels — re-assigned by every ui_show_* call */
static lv_obj_t *lbl_status = NULL; /* updated by ui_status_set() / set_status() */
static lv_obj_t *lbl_weight = NULL; /* weight readout; NULL on screens without it */
static float s_last_weight_g = -1.0f; /* -1 = no reading yet */
static uint16_t s_weight_req_seq = 0u; /* 0 = no in-flight read request */
static uint32_t s_weight_req_interval_us = 0u;

/* Scale screen auto-refresh timer — non-NULL only while the Scale screen is active */
static lv_timer_t *s_scale_timer = NULL;

/* Operations screen async-update labels — NULL when not on that screen */
static lv_obj_t *s_ops_lbl_status = NULL; /* last motion-done result */
static lv_obj_t *s_ops_lbl_vacuum = NULL; /* vacuum pump status + RPM */

/* Dosing screen async-update labels — NULL when not on that screen */
static lv_obj_t *s_dose_lbl_status = NULL;   /* spawn status string */
static lv_obj_t *s_dose_lbl_progress = NULL; /* "X.XX g / Y.YY g" */
static lv_obj_t *s_dose_lbl_retries = NULL;  /* "Retries: N  Bag: N" */
static lv_obj_t *s_dose_bar = NULL;          /* progress bar 0..1000 */
static lv_obj_t *s_dose_lbl_innoc = NULL;    /* inoculation % display */
static uint32_t s_dose_target_ug = 0;
static uint16_t s_dose_innoc_pct = 200; /* x10 → default 20.0 % */
static uint8_t s_dose_bag_number = 0;
#define UI_STATUS_MSG_MAX_LEN 72u

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

static bool sequence_manual_actions_blocked(void)
{
    switch (sys_sequence_get_state())
    {
    case SYS_IDLE:
    case SYS_SPAWN_EMPTY:
    case SYS_ERROR:
    case SYS_ESTOP:
        return false;
    default:
        return true;
    }
}

/* ── Home-screen button callbacks ────────────────────────────────────────── */

static void on_dose(lv_event_t *e)
{
    (void)e;
    if (sequence_manual_actions_blocked())
    {
        set_status("SEQ active: manual disabled");
        return;
    }
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
    if (sequence_manual_actions_blocked())
    {
        set_status("SEQ active: manual disabled");
        return;
    }
    ui_show_operations();
    ESP_LOGI(TAG, "Operations pressed");
}

static void on_auto_page(lv_event_t *e)
{
    (void)e;
    ui_show_auto();
    ESP_LOGI(TAG, "Automated Functions pressed");
}

static void scale_auto_read_cb(lv_timer_t *t)
{
    (void)t;
    static const pl_hx711_measure_t req = {.interval_us = 500000}; /* 500 ms measurement window */
    (void)pico_link_send(MSG_HX711_MEASURE, &req, sizeof(req), NULL);
}

static void scale_timer_stop(void)
{
    if (s_scale_timer)
    {
#if defined(LVGL_VERSION_MAJOR) && (LVGL_VERSION_MAJOR >= 9)
        lv_timer_delete(s_scale_timer);
#else
        lv_timer_del(s_scale_timer);
#endif
        s_scale_timer = NULL;
    }
}

static void on_scale_page(lv_event_t *e)
{
    (void)e;
    ui_show_scale();
    ESP_LOGI(TAG, "Scale pressed");
}

static void on_tare(lv_event_t *e)
{
    (void)e;
    if (sequence_manual_actions_blocked())
    {
        set_status("SEQ active: manual disabled");
        return;
    }
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
        s_weight_req_seq = seq;
        s_weight_req_interval_us = measure_payload.interval_us;
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

static void on_setup_load(lv_event_t *e)
{
    (void)e;
    esp_err_t err = sys_sequence_send_cmd(SYS_CMD_SETUP_LOAD);
    set_status(err == ESP_OK ? "Setup/Load started" : "Setup/Load FAILED");
    ESP_LOGI(TAG, "Setup/Load (%s)", esp_err_to_name(err));
}

static void on_start(lv_event_t *e)
{
    (void)e;
    esp_err_t err = sys_sequence_send_cmd(SYS_CMD_START);
    set_status(err == ESP_OK ? "Starting..." : "Start FAILED");
    ESP_LOGI(TAG, "Start pressed (%s)", esp_err_to_name(err));
}

static void on_seq_start(lv_event_t *e)
{
    (void)e;
    esp_err_t err = sys_sequence_send_cmd(SYS_CMD_START);
    set_status(err == ESP_OK ? "SEQ: START sent" : "SEQ: START failed");
    ESP_LOGI(TAG, "Sequence start (%s)", esp_err_to_name(err));
}

static void on_seq_abort(lv_event_t *e)
{
    (void)e;
    esp_err_t err = sys_sequence_send_cmd(SYS_CMD_ABORT);
    set_status(err == ESP_OK ? "Abort sent" : "Abort failed");
    ESP_LOGI(TAG, "Sequence abort (%s)", esp_err_to_name(err));
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

static void on_arm_home(lv_event_t *e)
{
    (void)e;
    esp_err_t err = motor_arm_home();
    set_status(err == ESP_OK ? "Arm: homing..." : "Arm home: FAILED");
    ESP_LOGI(TAG, "Arm home (%s)", esp_err_to_name(err));
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

static void on_agitate_home(lv_event_t *e)
{
    (void)e;
    /* Sensorlessly home the agitator (AGITATE_FLAG_DO_HOME, default n_cycles) */
    pl_agitate_t pl = { .flags = AGITATE_FLAG_DO_HOME, .n_cycles = 0u };
    uint8_t nack_code = 0u;
    esp_err_t err = pico_link_send_rpc(MSG_AGITATE,
                                       &pl, (uint16_t)sizeof(pl),
                                       300u, &nack_code);
    set_status(err == ESP_OK ? "Agit: homing..." : "Agit home: FAILED");
    ESP_LOGI(TAG, "Agitate home (%s nack=%u)", esp_err_to_name(err), nack_code);
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

static void on_turntable_intake(lv_event_t *e)
{
    (void)e;
    esp_err_t err = motor_turntable_goto((uint8_t)TURNTABLE_POS_INTAKE);
    set_status(err == ESP_OK ? "Tbl: to INTAKE..." : "Tbl INTAKE: FAILED");
    ESP_LOGI(TAG, "Turntable INTAKE (%s)", esp_err_to_name(err));
}

static void on_turntable_trash(lv_event_t *e)
{
    (void)e;
    esp_err_t err = motor_turntable_goto((uint8_t)TURNTABLE_POS_TRASH);
    set_status(err == ESP_OK ? "Tbl: to TRASH..." : "Tbl TRASH: FAILED");
    ESP_LOGI(TAG, "Turntable TRASH (%s)", esp_err_to_name(err));
}

static void on_turntable_eject(lv_event_t *e)
{
    (void)e;
    esp_err_t err = motor_turntable_goto((uint8_t)TURNTABLE_POS_EJECT);
    set_status(err == ESP_OK ? "Tbl: to EJECT..." : "Tbl EJECT: FAILED");
    ESP_LOGI(TAG, "Turntable EJECT (%s)", esp_err_to_name(err));
}

static void on_agitate(lv_event_t *e)
{
    (void)e;
    esp_err_t err = motor_agitate();
    set_status(err == ESP_OK ? "Agitating..." : "Agitate: FAILED");
    ESP_LOGI(TAG, "Agitate (%s)", esp_err_to_name(err));
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

static void on_hotwire_cut(lv_event_t *e)
{
    (void)e;
    esp_err_t err = motor_hotwire_traverse(true);
    set_status(err == ESP_OK ? "Wire: CUT..." : "Wire CUT: FAILED");
    ESP_LOGI(TAG, "Hotwire cut traverse (%s)", esp_err_to_name(err));
}

static void on_hotwire_return(lv_event_t *e)
{
    (void)e;
    esp_err_t err = motor_hotwire_traverse(false);
    set_status(err == ESP_OK ? "Wire: RETURN (zeroing)..." : "Wire RETURN: FAILED");
    ESP_LOGI(TAG, "Hotwire return traverse (%s); Pico zeros position on success",
             esp_err_to_name(err));
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
    if (msg_type == MSG_HX711_MEASURE && payload != NULL)
    {
        if (len == sizeof(pl_hx711_measure_t) && s_weight_req_seq != 0u && seq == s_weight_req_seq)
        {
            uint32_t echoed_interval_us = 0u;
            memcpy(&echoed_interval_us, payload, sizeof(echoed_interval_us));
            if (echoed_interval_us == s_weight_req_interval_us)
            {
                /* UART loopback/echo case: request came back as RX frame.
                 * This is not a weight response. */
                ESP_LOGW(TAG, "Ignored echoed HX711 request (seq=%u interval_us=%lu)",
                         (unsigned)seq, (unsigned long)echoed_interval_us);
                set_status("HX711 echo detected (check Pico->ESP RX path)");
                s_weight_req_seq = 0u;
                return;
            }
        }

        int64_t mass_ug = 0;
        uint8_t unit = 0u;

        if (len >= sizeof(pl_hx711_mass_t))
        {
            pl_hx711_mass_t mass_data;
            memcpy(&mass_data, payload, sizeof(mass_data));
            mass_ug = mass_data.mass_ug;
            unit = mass_data.unit;
        }
        else if (len >= sizeof(int64_t))
        {
            /* Legacy Pico payload: mass only (int64_t micrograms). */
            memcpy(&mass_ug, payload, sizeof(mass_ug));
            ESP_LOGW(TAG, "Received legacy HX711 payload len=%u (mass only)",
                     (unsigned)len);
        }
        else
        {
            ESP_LOGW(TAG, "Invalid HX711 payload len=%u", (unsigned)len);
            return;
        }

        float weight_g = (float)mass_ug / 1000000.0f;
        s_last_weight_g = weight_g;
        s_weight_req_seq = 0u;
        ESP_LOGI(TAG, "Received weight: %.3f g (unit=%u)", weight_g,
                 unit);

        char weight_str[32];
        snprintf(weight_str, sizeof(weight_str), "Weight: %.2f g", weight_g);

        lvgl_port_lock(0);
        if (lbl_weight)
        {
            lv_label_set_text(lbl_weight, weight_str);
        }
        lvgl_port_unlock();

        char status_buf[64];
        snprintf(status_buf, sizeof(status_buf), "Weight: %.2f g", weight_g);
        set_status(status_buf);
        return;
    }

    /* If the Pico rejects a read request, surface it on the Scale screen. */
    if (msg_type == MSG_NACK &&
        payload != NULL &&
        len >= sizeof(pl_nack_t) &&
        s_weight_req_seq != 0u &&
        seq == s_weight_req_seq)
    {
        const pl_nack_t *nack = (const pl_nack_t *)payload;
        char status_buf[64];
        snprintf(status_buf, sizeof(status_buf), "Weight read NACK (code=%u)",
                 (unsigned)nack->code);
        set_status(status_buf);
        ESP_LOGW(TAG, "Weight read NACK for seq=%u code=%u",
                 (unsigned)seq, (unsigned)nack->code);
        s_weight_req_seq = 0u;
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
    case MOTION_SPI_FAULT:
        res_name = "SPI FAULT";
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

void ui_ops_on_arm_seal_event(const pl_arm_seal_event_t *pl)
{
    if (!pl)
        return;

    const char *event_str = ((arm_seal_event_t)pl->event == ARM_SEAL_EVENT_RESTORED) ? "restored" : "lost";
    const char *reason_str = "unknown";
    switch ((arm_seal_reason_t)pl->reason)
    {
    case ARM_SEAL_REASON_TRANSIENT:
        reason_str = "transient";
        break;
    case ARM_SEAL_REASON_STEADY:
        reason_str = "steady";
        break;
    case ARM_SEAL_REASON_STALE_TACH:
        reason_str = "stale tach";
        break;
    default:
        break;
    }

    char msg[UI_STATUS_MSG_MAX_LEN];
    snprintf(msg, sizeof(msg), "Arm seal %s (%s)", event_str, reason_str);
    ui_status_set(msg);
    ESP_LOGW(TAG, "%s", msg);
}

/* ── UI helpers ──────────────────────────────────────────────────────────── */

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

/*
 * Automated Functions Screen Layout  (320 × 240, landscape, 10 px pad → 300 × 220 content)
 *
 *  y=  2  "Automated Functions"  title label (TOP_LEFT)    [Home 60×20, TOP_RIGHT]
 *  y= 20  status label (colour = white)
 *  y= 42  [Setup / Load  300×56]
 *  y=104  [Start         145×56]  10  [Abort  145×56]
 */
void ui_show_auto(void)
{
    lv_obj_t *scr = LVGL_ACTIVE_SCREEN();
    lv_obj_clean(scr);

    /* Stop Scale screen auto-refresh timer if active */
    scale_timer_stop();

    /* Nullify async-update handles on unrelated screens */
    s_ops_lbl_status = NULL;
    s_ops_lbl_vacuum = NULL;
    lbl_weight = NULL;

    lv_obj_clear_flag(scr, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_style_pad_all(scr, 10, 0);

    /* ── Title ───────────────────────────────────────────────────────────── */
    lv_obj_t *lbl_title = lv_label_create(scr);
    lv_label_set_text(lbl_title, "Automated Functions");
    lv_obj_align(lbl_title, LV_ALIGN_TOP_LEFT, 0, 2);

    /* ── Home nav (top-right) ────────────────────────────────────────────── */
    lv_obj_t *btn_nav = lv_btn_create(scr);
    lv_obj_set_size(btn_nav, 60, 20);
    lv_obj_align(btn_nav, LV_ALIGN_TOP_RIGHT, 0, 0);
    lv_obj_add_event_cb(btn_nav, on_home, LV_EVENT_CLICKED, NULL);
    lv_obj_t *l_nav = lv_label_create(btn_nav);
    lv_label_set_text(l_nav, "Home");
    lv_obj_center(l_nav);

    /* ── Status label (y=20) ─────────────────────────────────────────────── */
    lbl_status = lv_label_create(scr);
    lv_label_set_text(lbl_status, "Ready");
    lv_obj_set_style_text_font(lbl_status, &lv_font_montserrat_14, 0);
    lv_obj_set_style_text_color(lbl_status, lv_color_hex(0xFFFFFF), 0);
    lv_obj_align(lbl_status, LV_ALIGN_TOP_LEFT, 0, 20);

    /* ── Action buttons ─────────────────────────────────────────────────── */
    make_btn(scr, "Setup / Load", 0,   42, 300, 56, on_setup_load);
    make_btn(scr, "Start",        0,  104, 145, 56, on_seq_start);
    make_btn(scr, "Abort",      155,  104, 145, 56, on_seq_abort);
}

/*
 * Scale Screen Layout  (320 × 240, landscape, 10 px pad → 300 × 220 content)
 *
 *  y=  2  "Scale"  title label (TOP_LEFT)    [Home 60×20, TOP_RIGHT]
 *  y= 28  weight readout label (large, live-updated)
 *  y= 68  [Tare  145×80]   4   [Read  145×80]
 *  y=156  status label (result of last Tare / Read)
 */
void ui_show_scale(void)
{
    lv_obj_t *scr = LVGL_ACTIVE_SCREEN();
    lv_obj_clean(scr);

    /* Delete any stale timer before creating a new one (re-entry guard) */
    scale_timer_stop();

    /* Nullify unrelated async handles */
    s_ops_lbl_status = NULL;
    s_ops_lbl_vacuum = NULL;
    s_dose_lbl_status = NULL;
    s_dose_lbl_progress = NULL;
    s_dose_lbl_retries = NULL;
    s_dose_bar = NULL;
    s_dose_lbl_innoc = NULL;
    s_dose_target_ug = 0;

    lv_obj_clear_flag(scr, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_style_pad_all(scr, 10, 0);

    /* ── Title ───────────────────────────────────────────────────────────── */
    lv_obj_t *lbl_title = lv_label_create(scr);
    lv_label_set_text(lbl_title, "Scale");
    lv_obj_align(lbl_title, LV_ALIGN_TOP_LEFT, 0, 2);

    /* ── Home nav (top-right) ────────────────────────────────────────────── */
    lv_obj_t *btn_nav = lv_btn_create(scr);
    lv_obj_set_size(btn_nav, 60, 20);
    lv_obj_align(btn_nav, LV_ALIGN_TOP_RIGHT, 0, 0);
    lv_obj_add_event_cb(btn_nav, on_home, LV_EVENT_CLICKED, NULL);
    lv_obj_t *l_nav = lv_label_create(btn_nav);
    lv_label_set_text(l_nav, "Home");
    lv_obj_center(l_nav);

    /* ── Weight readout (y=28) — updated live via ui_screens_pico_rx_handler */
    lbl_weight = lv_label_create(scr);
    lv_obj_set_style_text_font(lbl_weight, &lv_font_montserrat_14, 0);
    /* Show last-known value immediately so the screen is not blank */
    {
        char w_buf[32];
        if (s_last_weight_g >= 0.0f)
            snprintf(w_buf, sizeof(w_buf), "Weight: %.2f g", (double)s_last_weight_g);
        else
            snprintf(w_buf, sizeof(w_buf), "Weight: -- g");
        lv_label_set_text(lbl_weight, w_buf);
    }
    lv_obj_align(lbl_weight, LV_ALIGN_TOP_LEFT, 0, 28);

    /* ── Tare / Read buttons (y=68, h=80, 4 px gap) ─────────────────────── */
    make_btn(scr, "Tare", 0,   68, 148, 80, on_tare);
    make_btn(scr, "Read", 152, 68, 148, 80, on_read_weight);

    /* ── Status label (y=156) ────────────────────────────────────────────── */
    lbl_status = lv_label_create(scr);
    lv_label_set_text(lbl_status, "");
    lv_obj_set_style_text_font(lbl_status, &lv_font_montserrat_14, 0);
    lv_obj_align(lbl_status, LV_ALIGN_TOP_LEFT, 0, 156);

    /* Manual read only: do not auto-poll the Pico from this screen. */
}

/*
 * Home Screen Layout  (320 × 240, landscape, 10 px pad → 300 × 220 content)
 *
 *  y=  0  status label "IDLE • Seacoast Inoculator"
 *  y= 20  [Automated Functions  300×46]
 *  y= 70  [Scale                300×46]
 *  y=120  [Operations           300×46]
 *  y=170  [Dosing               300×46]
 */
void ui_show_home(void)
{
    lv_obj_t *scr = LVGL_ACTIVE_SCREEN();
    lv_obj_clean(scr);

    /* Stop Scale screen auto-refresh timer if active */
    scale_timer_stop();

    /* Nullify all async-update handles so stale pointer writes can't crash */
    s_ops_lbl_status = NULL;
    s_ops_lbl_vacuum = NULL;
    s_dose_lbl_status = NULL;
    s_dose_lbl_progress = NULL;
    s_dose_lbl_retries = NULL;
    s_dose_bar = NULL;
    s_dose_lbl_innoc = NULL;
    s_dose_target_ug = 0;
    lbl_weight = NULL;

    lv_obj_clear_flag(scr, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_style_pad_all(scr, 10, 0);

    /* Status strip */
    lbl_status = lv_label_create(scr);
    lv_label_set_text(lbl_status, "IDLE \xe2\x80\xa2 Seacoast Inoculator");
    lv_obj_align(lbl_status, LV_ALIGN_TOP_LEFT, 0, 0);

    /* Four navigation buttons */
    make_btn(scr, "Automated Functions", 0,   20, 300, 46, on_auto_page);
    make_btn(scr, "Scale",               0,   70, 300, 46, on_scale_page);
    make_btn(scr, "Operations",          0,  120, 300, 46, on_ops_page);
    make_btn(scr, "Dosing",              0,  170, 300, 46, on_dose);
}

/*
 * Operations Screen Layout  (320 × 240, landscape, 10 px pad → 300 × 220 content)
 *
 *  y=  0  "Operations"  title label (TOP_LEFT)    [Home 60×20, TOP_RIGHT]
 *  y= 22  [Flap Open  148×24]  4  [Flap Close  148×24]
 *  y= 50  4-btn 72px: [Arm Home][Arm Press][Arm Pos1][Arm Pos2]
 *  y= 78  [Rack Home  96×24]  4  [Rack Ext 96×24]  4  [Rack Press 96×24]
 *  y=106  4-btn 72px: [Tbl Home][Intake][Trash][Eject]
 *  y=134  4-btn 72px: [Wire ON][Wire OFF][Cut][Rtn+Zero]
 *  y=162  6-btn 46px: [Vac ON][Vac OFF][Vac2 ON][Vac2 OFF][Agitate][Agit Home]
 *  y=188  s_ops_lbl_status  (motion-done text)
 *  y=204  s_ops_lbl_vacuum  (vacuum status text)
 *
 *  Button-row widths:
 *    2-btn 148px: 148+4+148 = 300 px
 *    3-btn 96px:  96+4+96+4+96 = 296 px
 *    4-btn 72px:  72+4+72+4+72+4+72 = 300 px  (x = 0, 76, 152, 228)
 *    5-btn 56px:  56+4+56+4+56+4+56+4+56 = 296 px  (x = 0, 60, 120, 180, 240)
 */
void ui_show_operations(void)
{
    lv_obj_t *scr = LVGL_ACTIVE_SCREEN();
    lv_obj_clean(scr);
    lv_obj_clear_flag(scr, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_style_pad_all(scr, 10, 0);

    /* Stop Scale screen auto-refresh timer if active */
    scale_timer_stop();

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

    /* ── Row 2: Arm  (y=50, h=24)  4 × 72 px + 3 × 4 px = 300 px ───────── */
    make_btn(scr, "Arm Home",  0,   50, 72, 24, on_arm_home);
    make_btn(scr, "Arm Press", 76,  50, 72, 24, on_arm_press);
    make_btn(scr, "Arm Pos 1", 152, 50, 72, 24, on_arm_pos1);
    make_btn(scr, "Arm Pos 2", 228, 50, 72, 24, on_arm_pos2);

    /* ── Row 3: Rack  (y=78, h=24)  3 × 96 px + 2 × 4 px = 296 px ──────── */
    make_btn(scr, "Rack Home",   0,   78, 96, 24, on_rack_home);
    make_btn(scr, "Rack Extend", 100, 78, 96, 24, on_rack_extend);
    make_btn(scr, "Rack Press",  200, 78, 96, 24, on_rack_press);

    /* ── Row 4: Turntable  (y=106, h=24)  4 × 72 px + 3 × 4 px = 300 px ── */
    make_btn(scr, "Tbl Home", 0,   106, 72, 24, on_turntable_home);
    make_btn(scr, "Intake",   76,  106, 72, 24, on_turntable_intake);
    make_btn(scr, "Trash",    152, 106, 72, 24, on_turntable_trash);
    make_btn(scr, "Eject",    228, 106, 72, 24, on_turntable_eject);

    /* ── Row 5: Hot Wire  (y=134, h=24)  4 × 72 px + 3 × 4 px = 300 px ───── */
    make_btn(scr, "Wire ON",  0,   134, 72, 24, on_hotwire_on);
    make_btn(scr, "Wire OFF", 76,  134, 72, 24, on_hotwire_off);
    make_btn(scr, "Cut",      152, 134, 72, 24, on_hotwire_cut);
    make_btn(scr, "Rtn+Zero", 228, 134, 72, 24, on_hotwire_return);

    /* ── Row 6: Vacuum + Agitator  (y=162, h=24)  6 × 46 px + 5 × 4 px = 296 px ── */
    make_btn(scr, "Vac ON",    0,   162, 46, 24, on_vacuum_on);
    make_btn(scr, "Vac OFF",   50,  162, 46, 24, on_vacuum_off);
    make_btn(scr, "Vac2 ON",   100, 162, 46, 24, on_vacuum2_on);
    make_btn(scr, "Vac2 OFF",  150, 162, 46, 24, on_vacuum2_off);
    make_btn(scr, "Agitate",   200, 162, 46, 24, on_agitate);
    make_btn(scr, "Agit Home", 250, 162, 46, 24, on_agitate_home);

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

static void on_dose_start(lv_event_t *e)
{
    (void)e;

    pl_innoculate_bag_t pl = {
        .bag_mass = 0,      /* unused by Pico — scale read internally */
        .spawn_mass = 1000, /* 1 kg remaining (informational) */
        .innoc_percent = s_dose_innoc_pct,
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
        ESP_LOGI(TAG, "MSG_DISPENSE_SPAWN sent (innoc=%u, bag=%u)",
                 s_dose_innoc_pct, pl.bag_number);
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
    /* Send MSG_CTRL_STOP so the Pico aborts the spawn state machine,
     * stops the timer, stops flaps, and fast-closes — see uart_server.c
     * MSG_CTRL_STOP handler. */
    uint8_t nack = 0;
    esp_err_t err = pico_link_send_rpc(MSG_CTRL_STOP, NULL, 0,
                                       2000, /* 2 s timeout */
                                       &nack);
    set_status(err == ESP_OK ? "Aborted" : "Abort FAILED");
    ESP_LOGI(TAG, "Dose abort via MSG_CTRL_STOP (%s nack=%u)",
             esp_err_to_name(err), nack);
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
 *    y=152  [Start Dose 145×65]  4  [Abort 145×65]  (to bottom)                *
 * ─────────────────────────────────────────────────────────────────────────── */
void ui_show_dosing(void)
{
    lv_obj_t *scr = LVGL_ACTIVE_SCREEN();
    lv_obj_clean(scr);

    /* Stop Scale screen auto-refresh timer if active */
    scale_timer_stop();

    /* Nullify all async-update handles so stale pointer writes can't crash. */
    s_ops_lbl_status = NULL;
    s_ops_lbl_vacuum = NULL;
    s_dose_lbl_status = NULL;
    s_dose_lbl_progress = NULL;
    s_dose_lbl_retries = NULL;
    s_dose_bar = NULL;
    s_dose_lbl_innoc = NULL;
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

    /* ── Start / Abort row (y=152) ───────────────────────────────────────── */
    make_btn(scr, "Start Dose", 0, 152, 145, 65, on_dose_start);
    make_btn(scr, "Abort", 155, 152, 145, 65, on_dose_abort);
}
