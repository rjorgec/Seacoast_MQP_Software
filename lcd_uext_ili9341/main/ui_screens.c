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

static lv_obj_t *lbl_status = NULL;
static lv_obj_t *lbl_weight = NULL;
static float s_last_weight_g = 0.0f;

// ── Stepper test screen state ────────────────────────────────────────────────
#define STEPPER_N_DEVICES 3u
#define STEPPER_DELAY_US 20000u // 10 ms inter-step delay
#define STEPPER_STEPS_MIN 100u
#define STEPPER_STEPS_MAX 2000u
#define STEPPER_STEPS_INC 100u

static lv_obj_t *s_step_dev_ind[STEPPER_N_DEVICES]; // per-device indicator boxes
static lv_obj_t *s_step_lbl_steps = NULL;           // "Steps: NNN" label
static bool s_step_enabled = false;                 // tracks last enable state
static uint32_t s_step_count = 100u;                // step count for next job

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

// ---- Button callbacks ----

static void on_start(lv_event_t *e)
{
    (void)e;
    ctrl_cmd_t cmd = {
        .type = CTRL_CMD_START,
        .target_g = 50.0f, // TODO make it editable
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

static void on_dose(lv_event_t *e)
{
    (void)e;
    ui_show_dosing();
    ESP_LOGI(TAG, "DOSE presses");
}

static void on_home(lv_event_t *e)
{
    (void)e;
    ui_show_home();
    ESP_LOGI(TAG, "HOME pressed");
}

static void on_stepper_page(lv_event_t *e)
{
    (void)e;
    ui_show_stepper();
    ESP_LOGI(TAG, "Stepper Test pressed");
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

    // Send MSG_HX711_TARE request to Pico via UART
    uint8_t nack_code = 0;
    esp_err_t err = pico_link_send_rpc(MSG_HX711_TARE,
                                       NULL,
                                       0,
                                       1000, // 1 second timeout
                                       &nack_code);

    if (err == ESP_OK)
    {
        s_last_weight_g = 0.0f; // Reset displayed weight
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

static void on_recipes(lv_event_t *e)
{
    (void)e;
    // TODO
    set_status("RECIPES (TODO)");
    ESP_LOGI(TAG, "Recipes pressed");
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

static void on_read_weight(lv_event_t *e)
{
    (void)e;

    // Send MSG_HX711_MEASURE request to Pico via UART
    pl_hx711_measure_t measure_payload = {
        .interval_us = 1000000 // 1 second interval for measurement
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
        snprintf(buf, sizeof(buf), "Weight req failed: %s", esp_err_to_name(err));
        set_status(buf);
        ESP_LOGW(TAG, "Weight request failed (%s)", esp_err_to_name(err));
    }
}

// Callback for receiving weight data from Pico (exposed for app_main)
void ui_screens_pico_rx_handler(uint8_t msg_type, uint16_t seq, const uint8_t *payload, uint16_t len)
{
    (void)seq;

    if (msg_type == MSG_HX711_MEASURE && len == sizeof(pl_hx711_mass_t))
    {
        pl_hx711_mass_t mass_data;
        memcpy(&mass_data, payload, sizeof(mass_data));

        // Convert micrograms to grams
        float weight_g = (float)mass_data.mass_ug / 1000000.0f;
        s_last_weight_g = weight_g;

        ESP_LOGI(TAG, "Received weight: %.3f g (unit=%u)", weight_g, mass_data.unit);

        // Update weight label if it exists
        if (lbl_weight)
        {
            char weight_str[32];
            snprintf(weight_str, sizeof(weight_str), "Weight: %.2f g", weight_g);

            lvgl_port_lock(0);
            lv_label_set_text(lbl_weight, weight_str);
            lvgl_port_unlock();
        }

        // Update status
        char status_buf[64];
        snprintf(status_buf, sizeof(status_buf), "Weight: %.2f g", weight_g);
        set_status(status_buf);
    }
}

// ── Stepper screen helpers & callbacks ──────────────────────────────────────

static void stepper_refresh_indicators(void)
{
    for (int i = 0; i < STEPPER_N_DEVICES; ++i)
    {
        if (!s_step_dev_ind[i])
            continue;
        // Green = outputs enabled, dark grey = disabled / unknown
        lv_color_t c = s_step_enabled ? lv_color_hex(0x00AA44)
                                      : lv_color_hex(0x444444);
        lv_obj_set_style_bg_color(s_step_dev_ind[i], c, 0);
    }
}

static void stepper_refresh_steps_label(void)
{
    if (!s_step_lbl_steps)
        return;
    char buf[24];
    snprintf(buf, sizeof(buf), "Steps: %lu", (unsigned long)s_step_count);
    lv_label_set_text(s_step_lbl_steps, buf);
}

static void on_step_enable_all(lv_event_t *e)
{
    (void)e;
    pl_stepper_enable_t pl = {.enable = 1};
    uint8_t nack = 0;
    esp_err_t err = pico_link_send_rpc(MSG_MOTOR_STEPPER_ENABLE,
                                       &pl, sizeof(pl), 500, &nack);
    if (err == ESP_OK)
    {
        s_step_enabled = true;
        stepper_refresh_indicators();
        set_status("Steppers: ENABLED");
        ESP_LOGI(TAG, "Stepper enable all: ok");
    }
    else
    {
        char buf[56];
        snprintf(buf, sizeof(buf), "Enable failed: %s (nack=%u)",
                 esp_err_to_name(err), nack);
        set_status(buf);
        ESP_LOGW(TAG, "Stepper enable all: %s nack=%u",
                 esp_err_to_name(err), nack);
    }
}

static void on_step_disable_all(lv_event_t *e)
{
    (void)e;
    pl_stepper_enable_t pl = {.enable = 0};
    uint8_t nack = 0;
    esp_err_t err = pico_link_send_rpc(MSG_MOTOR_STEPPER_ENABLE,
                                       &pl, sizeof(pl), 500, &nack);
    if (err == ESP_OK)
    {
        s_step_enabled = false;
        stepper_refresh_indicators();
        set_status("Steppers: DISABLED");
        ESP_LOGI(TAG, "Stepper disable all: ok");
    }
    else
    {
        char buf[56];
        snprintf(buf, sizeof(buf), "Disable failed: %s (nack=%u)",
                 esp_err_to_name(err), nack);
        set_status(buf);
        ESP_LOGW(TAG, "Stepper disable all: %s nack=%u",
                 esp_err_to_name(err), nack);
    }
}

static void do_step_job(uint8_t dir)
{
    pl_stepper_stepjob_t pl = {
        .dir = dir,
        .steps = s_step_count,
        .step_delay_us = STEPPER_DELAY_US,
    };
    // Timeout = (steps × delay_µs / 1000) ms + 2 s overhead
    uint32_t timeout_ms = (uint32_t)((s_step_count * STEPPER_DELAY_US) / 1000u) + 2000u;
    uint8_t nack = 0;
    esp_err_t err = pico_link_send_rpc(MSG_MOTOR_STEPPER_STEPJOB,
                                       &pl, sizeof(pl), timeout_ms, &nack);
    if (err == ESP_OK)
    {
        char buf[48];
        snprintf(buf, sizeof(buf), "%s %lu steps done",
                 dir == 0 ? "FWD" : "REV", (unsigned long)s_step_count);
        set_status(buf);
        ESP_LOGI(TAG, "Step job %s %lu steps: ok",
                 dir == 0 ? "FWD" : "REV", (unsigned long)s_step_count);
    }
    else
    {
        char buf[64];
        snprintf(buf, sizeof(buf), "%s failed: %s (nack=%u)",
                 dir == 0 ? "FWD" : "REV",
                 esp_err_to_name(err), nack);
        set_status(buf);
        ESP_LOGW(TAG, "Step job failed: %s nack=%u",
                 esp_err_to_name(err), nack);
    }
}

static void on_step_fwd(lv_event_t *e)
{
    (void)e;
    do_step_job(0);
}
static void on_step_rev(lv_event_t *e)
{
    (void)e;
    do_step_job(1);
}

static void on_step_minus(lv_event_t *e)
{
    (void)e;
    if (s_step_count > STEPPER_STEPS_MIN)
        s_step_count -= STEPPER_STEPS_INC;
    stepper_refresh_steps_label();
}

static void on_step_plus(lv_event_t *e)
{
    (void)e;
    if (s_step_count < STEPPER_STEPS_MAX)
        s_step_count += STEPPER_STEPS_INC;
    stepper_refresh_steps_label();
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

    // fix scrolling
    lv_obj_clear_flag(scr, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_style_pad_all(scr, 10, 0);

    // status strip
    lbl_status = lv_label_create(scr);
    lv_label_set_text(lbl_status, "IDLE • Seacoast Inoculator");
    lv_obj_align(lbl_status, LV_ALIGN_TOP_LEFT, 0, 0);

    // 2-column × 3-row button grid (row 2 = full-width Stepper Test button)
    lv_obj_t *cont = lv_obj_create(scr);
    lv_obj_clear_flag(cont, LV_OBJ_FLAG_SCROLLABLE);

    lv_obj_set_size(cont, 300, 210);
    lv_obj_align(cont, LV_ALIGN_BOTTOM_MID, 0, 0);

    lv_obj_set_style_pad_all(cont, 6, 0);
    lv_obj_set_style_pad_row(cont, 6, 0);
    lv_obj_set_style_pad_column(cont, 6, 0);

    lv_obj_set_layout(cont, LV_LAYOUT_GRID);

    static lv_coord_t col_dsc[] = {LV_GRID_FR(1), LV_GRID_FR(1), LV_GRID_TEMPLATE_LAST};
    static lv_coord_t row_dsc[] = {LV_GRID_FR(1), LV_GRID_FR(1), LV_GRID_FR(1), LV_GRID_TEMPLATE_LAST};
    lv_obj_set_grid_dsc_array(cont, col_dsc, row_dsc);

    lv_obj_t *b_fwd = make_big_btn(cont, "Forward", on_fwd);
    lv_obj_t *b_rev = make_big_btn(cont, "Backward", on_rev);
    lv_obj_t *b_dose = make_big_btn(cont, "Dose", on_dose);
    lv_obj_t *b_tare = make_big_btn(cont, "Tare", on_tare);
    lv_obj_t *b_step = make_big_btn(cont, "Stepper Test", on_stepper_page);

    lv_obj_set_grid_cell(b_fwd, LV_GRID_ALIGN_STRETCH, 0, 1, LV_GRID_ALIGN_STRETCH, 0, 1);
    lv_obj_set_grid_cell(b_rev, LV_GRID_ALIGN_STRETCH, 1, 1, LV_GRID_ALIGN_STRETCH, 0, 1);
    lv_obj_set_grid_cell(b_dose, LV_GRID_ALIGN_STRETCH, 0, 1, LV_GRID_ALIGN_STRETCH, 1, 1);
    lv_obj_set_grid_cell(b_tare, LV_GRID_ALIGN_STRETCH, 1, 1, LV_GRID_ALIGN_STRETCH, 1, 1);
    // Stepper Test spans both columns in row 2 for a wide, easy touch target
    lv_obj_set_grid_cell(b_step, LV_GRID_ALIGN_STRETCH, 0, 2, LV_GRID_ALIGN_STRETCH, 2, 1);
}

void ui_show_stepper(void)
{
    lv_obj_t *scr = LVGL_ACTIVE_SCREEN();
    lv_obj_clean(scr);
    lv_obj_clear_flag(scr, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_style_pad_all(scr, 10, 0);

    // Invalidate weight label (not shown on this screen)
    lbl_weight = NULL;

    // ── Status label (top) ───────────────────────────────────────────────────
    lbl_status = lv_label_create(scr);
    lv_label_set_text(lbl_status, "Stepper Test \xE2\x80\x94 3 Devices");
    lv_obj_align(lbl_status, LV_ALIGN_TOP_LEFT, 0, 0);

    // ── Device indicator boxes (row below status) ─────────────────────────
    // Three colored boxes: grey = disabled/unknown, green = enabled.
    // Width per box: (300 - 2*4) / 3 = 97 px; gap = 4 px between boxes.
    static const char *dev_names[STEPPER_N_DEVICES] = {"D0", "D1", "D2"};
    for (int i = 0; i < STEPPER_N_DEVICES; ++i)
    {
        lv_obj_t *box = lv_obj_create(scr);
        lv_obj_set_size(box, 97, 28);
        lv_obj_clear_flag(box, LV_OBJ_FLAG_SCROLLABLE);
        lv_obj_set_style_pad_all(box, 2, 0);
        lv_obj_align(box, LV_ALIGN_TOP_LEFT, i * 101, 22);
        s_step_dev_ind[i] = box;

        lv_obj_t *lbl = lv_label_create(box);
        lv_label_set_text(lbl, dev_names[i]);
        lv_obj_center(lbl);
    }
    stepper_refresh_indicators();

    // ── Enable / Disable buttons ─────────────────────────────────────────────
    // Two 148-px buttons side-by-side, 4-px gap (148+4+148=300).
    lv_obj_t *btn_en = lv_btn_create(scr);
    lv_obj_set_size(btn_en, 148, 40);
    lv_obj_align(btn_en, LV_ALIGN_TOP_LEFT, 0, 58);
    lv_obj_add_event_cb(btn_en, on_step_enable_all, LV_EVENT_CLICKED, NULL);
    lv_obj_t *l_en = lv_label_create(btn_en);
    lv_label_set_text(l_en, "Enable All");
    lv_obj_center(l_en);

    lv_obj_t *btn_dis = lv_btn_create(scr);
    lv_obj_set_size(btn_dis, 148, 40);
    lv_obj_align(btn_dis, LV_ALIGN_TOP_RIGHT, 0, 58);
    lv_obj_add_event_cb(btn_dis, on_step_disable_all, LV_EVENT_CLICKED, NULL);
    lv_obj_t *l_dis = lv_label_create(btn_dis);
    lv_label_set_text(l_dis, "Disable All");
    lv_obj_center(l_dis);

    // ── FWD / REV step buttons ────────────────────────────────────────────────
    lv_obj_t *btn_fwd = lv_btn_create(scr);
    lv_obj_set_size(btn_fwd, 148, 40);
    lv_obj_align(btn_fwd, LV_ALIGN_TOP_LEFT, 0, 102);
    lv_obj_add_event_cb(btn_fwd, on_step_fwd, LV_EVENT_CLICKED, NULL);
    lv_obj_t *l_fwd = lv_label_create(btn_fwd);
    lv_label_set_text(l_fwd, LV_SYMBOL_LEFT " FWD");
    lv_obj_center(l_fwd);

    lv_obj_t *btn_rev = lv_btn_create(scr);
    lv_obj_set_size(btn_rev, 148, 40);
    lv_obj_align(btn_rev, LV_ALIGN_TOP_RIGHT, 0, 102);
    lv_obj_add_event_cb(btn_rev, on_step_rev, LV_EVENT_CLICKED, NULL);
    lv_obj_t *l_rev = lv_label_create(btn_rev);
    lv_label_set_text(l_rev, "REV " LV_SYMBOL_RIGHT);
    lv_obj_center(l_rev);

    // ── Step-count row:  [-]  [Steps: NNN]  [+] ──────────────────────────────
    lv_obj_t *btn_minus = lv_btn_create(scr);
    lv_obj_set_size(btn_minus, 60, 36);
    lv_obj_align(btn_minus, LV_ALIGN_TOP_LEFT, 0, 146);
    lv_obj_add_event_cb(btn_minus, on_step_minus, LV_EVENT_CLICKED, NULL);
    lv_obj_t *l_minus = lv_label_create(btn_minus);
    lv_label_set_text(l_minus, LV_SYMBOL_MINUS);
    lv_obj_center(l_minus);

    s_step_lbl_steps = lv_label_create(scr);
    stepper_refresh_steps_label();
    lv_obj_align(s_step_lbl_steps, LV_ALIGN_TOP_MID, 0, 154); // vertically centred in the row

    lv_obj_t *btn_plus = lv_btn_create(scr);
    lv_obj_set_size(btn_plus, 60, 36);
    lv_obj_align(btn_plus, LV_ALIGN_TOP_RIGHT, 0, 146);
    lv_obj_add_event_cb(btn_plus, on_step_plus, LV_EVENT_CLICKED, NULL);
    lv_obj_t *l_plus = lv_label_create(btn_plus);
    lv_label_set_text(l_plus, LV_SYMBOL_PLUS);
    lv_obj_center(l_plus);

    // ── Delay info label ─────────────────────────────────────────────────────
    lv_obj_t *lbl_delay = lv_label_create(scr);
    char dbuf[32];
    snprintf(dbuf, sizeof(dbuf), "Delay: %u \xCE\xBCs/step", STEPPER_DELAY_US);
    lv_label_set_text(lbl_delay, dbuf);
    lv_obj_set_style_text_font(lbl_delay, &lv_font_montserrat_14, 0);
    lv_obj_align(lbl_delay, LV_ALIGN_TOP_MID, 0, 186);

    // ── Home button (bottom) ─────────────────────────────────────────────────
    lv_obj_t *btn_home = lv_btn_create(scr);
    lv_obj_set_size(btn_home, 300, 36);
    lv_obj_align(btn_home, LV_ALIGN_BOTTOM_MID, 0, 0);
    lv_obj_add_event_cb(btn_home, on_home, LV_EVENT_CLICKED, NULL);
    lv_obj_t *l_home = lv_label_create(btn_home);
    lv_label_set_text(l_home, "Home");
    lv_obj_center(l_home);
}

// dosing screens tub
void ui_show_dosing(void)
{
    lv_obj_t *scr = LVGL_ACTIVE_SCREEN();
    lv_obj_clean(scr);
    lv_obj_clear_flag(scr, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_style_pad_all(scr, 10, 0);

    lbl_status = lv_label_create(scr);
    lv_label_set_text(lbl_status, "DOSING…");
    lv_obj_align(lbl_status, LV_ALIGN_TOP_LEFT, 0, 0);

    // Weight display label
    lbl_weight = lv_label_create(scr);
    char weight_str[32];
    snprintf(weight_str, sizeof(weight_str), "Weight: %.2f g", s_last_weight_g);
    lv_label_set_text(lbl_weight, weight_str);
    lv_obj_align(lbl_weight, LV_ALIGN_CENTER, 0, -20);
    lv_obj_set_style_text_font(lbl_weight, &lv_font_montserrat_14, 0);

    lv_obj_t *btn_home = lv_btn_create(scr);
    lv_obj_set_size(btn_home, 140, 70);
    lv_obj_align(btn_home, LV_ALIGN_BOTTOM_LEFT, 0, 0);
    lv_obj_add_event_cb(btn_home, on_home, LV_EVENT_CLICKED, NULL);
    lv_obj_t *lp = lv_label_create(btn_home);
    lv_label_set_text(lp, "Home");
    lv_obj_center(lp);

    lv_obj_t *btn_read_weight = lv_btn_create(scr);
    lv_obj_set_size(btn_read_weight, 140, 70);
    lv_obj_align(btn_read_weight, LV_ALIGN_BOTTOM_RIGHT, 0, 0);
    lv_obj_add_event_cb(btn_read_weight, on_read_weight, LV_EVENT_CLICKED, NULL);
    lv_obj_t *ls = lv_label_create(btn_read_weight);
    lv_label_set_text(ls, "Read Weight");
    lv_obj_center(ls);
}
