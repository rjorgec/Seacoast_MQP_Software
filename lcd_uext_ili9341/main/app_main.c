#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_err.h"
#include "nvs_flash.h"

#include "display.h"
#include "ui.h"

#include "touch_ns2009.h"
#include "touch_cal.h"
#include "esp_lvgl_port.h"

#include "control.h"
#include "ui_screens.h"

#include "motor_hal.h"
#include "pico_link.h"
#include "proto/proto.h"

// #include "recipes.h"
// #include "wifi_ap.h"
// #include "web_server.h"

static void pico_rx_cb(uint8_t type, uint16_t seq, const uint8_t *pl, uint16_t len)
{
    switch (type)
    {
    case MSG_MOTION_DONE:
        if (len >= sizeof(pl_motion_done_t))
        {
            ui_ops_on_motion_done((const pl_motion_done_t *)pl);
        }
        break;
    case MSG_VACUUM_STATUS:
        if (len >= sizeof(pl_vacuum_status_t))
        {
            ui_ops_on_vacuum_status((const pl_vacuum_status_t *)pl);
        }
        break;
    case MSG_SPAWN_STATUS:
        if (len >= sizeof(pl_spawn_status_t))
        {
            ui_dosing_on_spawn_status((const pl_spawn_status_t *)pl);
        }
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

    // ping on boot
    ESP_ERROR_CHECK(pico_link_send(MSG_PING, NULL, 0, NULL));

    // display/ui
    display_handles_t disp = display_init();
    ui_init(&disp);

    ESP_ERROR_CHECK(control_start()); // ONLY ONCE

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
