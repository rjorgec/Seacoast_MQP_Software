#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_err.h"
#include "esp_log.h"
#include "nvs_flash.h"

#include "display.h"
#include "ui.h"

#include "touch_ns2009.h"
#include "touch_cal.h"
#include "esp_lvgl_port.h"

#include "control.h"
#include "sys_sequence.h"
#include "ui_screens.h"

#include "motor_hal.h"
#include "pico_link.h"
#include "pico_power.h"
#include "proto.h"

// #include "recipes.h"
// #include "wifi_ap.h"
// #include "web_server.h"

static void pico_rx_cb(uint8_t type, uint16_t seq, const uint8_t *pl, uint16_t len)
{
    sys_state_t seq_state = sys_sequence_get_state();

    switch (type)
    {
    case MSG_MOTION_DONE:
        if (len >= sizeof(pl_motion_done_t))
        {
            const pl_motion_done_t *motion = (const pl_motion_done_t *)pl;
            ui_ops_on_motion_done(motion);

            switch (seq_state)
            {
            case SYS_SETUP_LOAD:
            case SYS_CUTTING_TIP:
            case SYS_ROTATING_TO_ACCEPT:
            case SYS_INTAKE_WEIGHING:
            case SYS_OPENING_BAG:
            case SYS_OPEN_RECOVERING:
            case SYS_POST_DOSE:
            case SYS_EJECTING:
            case SYS_ROTATING_TO_INTAKE:
            case SYS_CONTINUE_RESTART:
                sys_sequence_notify_motion_done(motion->subsystem, motion->result);
                break;
            default:
                break;
            }
        }
        break;
    case MSG_VACUUM_STATUS:
        if (len >= sizeof(pl_vacuum_status_t))
        {
            ui_ops_on_vacuum_status((const pl_vacuum_status_t *)pl);
        }
        break;
    case MSG_ARM_SEAL_EVENT:
        if (len >= sizeof(pl_arm_seal_event_t))
        {
            const pl_arm_seal_event_t *seal = (const pl_arm_seal_event_t *)pl;
            ui_ops_on_arm_seal_event(seal);
            sys_sequence_notify_arm_seal_event(seal->event, seal->reason);
        }
        break;
    case MSG_SPAWN_STATUS:
        if (len >= sizeof(pl_spawn_status_t))
        {
            const pl_spawn_status_t *spawn = (const pl_spawn_status_t *)pl;
            ui_dosing_on_spawn_status(spawn);
            if (seq_state == SYS_INOCULATING)
            {
                sys_sequence_notify_spawn_status(spawn->status);
            }
        }
        break;
    case MSG_HX711_MEASURE:
        ESP_LOGI("app_main", "RX HX711_MEASURE seq=%u len=%u", (unsigned)seq, (unsigned)len);
        if (len >= 4 && pl != NULL)
        {
            int32_t first_4_bytes = 0;
            memcpy(&first_4_bytes, pl, 4);
            ESP_LOGI("app_main", 
                     "  First 4 bytes (little-endian int32): %ld (0x%08lx) micrograms = %.3f g", 
                     (long)first_4_bytes, (unsigned long)first_4_bytes, (float)first_4_bytes / 1000000.0f);
        }
        if (len >= 5 && pl != NULL)
        {
            ESP_LOGI("app_main", "  Byte 4 (unit): 0x%02x (%u)", pl[4], pl[4]);
        }
        ui_screens_pico_rx_handler(type, seq, pl, len);
        break;
    case MSG_NACK:
        ESP_LOGW("app_main", "RX NACK seq=%u len=%u", (unsigned)seq, (unsigned)len);
        ui_screens_pico_rx_handler(type, seq, pl, len);
        break;
    default:
        /* Forward to UI screens handler for weight display */
        ui_screens_pico_rx_handler(type, seq, pl, len);
        break;
    }
}
static touch_ns2009_t g_touch;

void app_main(void)
{
    ESP_ERROR_CHECK(nvs_flash_init());

    /* Ensure the 12 V relay is open before anything else on the board gets a
     * chance to observe GPIO10's reset default. The Pico's Phase 1 pin setup
     * runs in parallel; neither side should see motor-driver power until the
     * PING handshake completes below. */
    ESP_ERROR_CHECK(pico_power_init());

    // //recipe upload pipeline
    // ESP_ERROR_CHECK(recipes_init());          //mounts SPIFFS (/spiffs)
    // ESP_ERROR_CHECK(wifi_ap_start());         //starts AP + DHCP (default 192.168.4.1)
    // ESP_ERROR_CHECK(web_server_start());
    pico_link_cfg_t link = {
        .uart_num = UART_NUM_1,
        .tx_gpio = 5, // pick free pins
        .rx_gpio = 4,
        .baud = 115200,
        .on_rx = pico_rx_cb};
    ESP_ERROR_CHECK(pico_link_init(&link));

    /* PING handshake. The Pico only ACKs once Phase 1 has run and its UART
     * handler is live, so an ACK here means IN2 is already HIGH (pump 2
     * held off) and it is safe to close the 12 V relay. If the Pico is
     * missing or wedged we leave the relay open and continue booting the UI
     * so the operator can see the error; motors will NACK until relay is
     * closed. 30 s covers a worst-case USB-CDC enumeration + startup on
     * the Pico. */
    {
        uint8_t nack_code = 0;
        esp_err_t ping_err = pico_link_send_rpc(MSG_PING, NULL, 0, 30000u, &nack_code);
        if (ping_err == ESP_OK)
        {
            ESP_ERROR_CHECK(pico_power_enable());
        }
        else
        {
            ESP_LOGE("app_main",
                     "Pico PING handshake failed (%s, nack=%u); motor driver"
                     " power left OFF — UI will still boot.",
                     esp_err_to_name(ping_err), (unsigned)nack_code);
        }
    }

    // display/ui
    display_handles_t disp = display_init();
    ui_init(&disp);

    ESP_ERROR_CHECK(control_start()); // ONLY ONCE
    ESP_ERROR_CHECK(sys_sequence_init());

    ESP_ERROR_CHECK(touch_ns2009_init(&g_touch, 6, 7, 100000, 0x48, 320, 240));

    // calibration
    touch_cal_t cal = {.version = 1, .x_min = 412, .x_max = 3677, .y_min = 602, .y_max = 3755};
    if (touch_cal_load(&cal) == ESP_OK)
    {
        touch_cal_apply(&g_touch, &cal);
    }
    else
    {
        touch_cal_apply(&g_touch, &cal);
        // touch_cal_save(&cal); //persist defaults once
    }

    g_touch.swap_xy = true;
    g_touch.mirror_x = false;
    g_touch.mirror_y = false;

    lvgl_port_lock(0);
    touch_ns2009_register_lvgl(&g_touch);
    ui_show_home();
    lvgl_port_unlock();

    while (1)
        vTaskDelay(pdMS_TO_TICKS(1000));
}
