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

static touch_ns2009_t g_touch;

void app_main(void)
{
    ESP_ERROR_CHECK(nvs_flash_init());

    display_handles_t disp = display_init();
    ui_init(&disp);

    ESP_ERROR_CHECK(touch_ns2009_init(&g_touch, 6, 7, 100000, 0x48, 320, 240));

 //calibration
    touch_cal_t cal = {.version = 1, .x_min = 412, .x_max = 3677, .y_min = 602, .y_max = 3755};
    if (touch_cal_load(&cal) == ESP_OK) {
        touch_cal_apply(&g_touch, &cal);
    } else {
        touch_cal_apply(&g_touch, &cal);
        
        // touch_cal_save(&cal); //persist defaults once
    }

    g_touch.swap_xy  = true;
    g_touch.mirror_x = false;
    g_touch.mirror_y = false;

    ESP_ERROR_CHECK(control_start());

    lvgl_port_lock(0);
    touch_ns2009_register_lvgl(&g_touch);
    ui_show_home();
    lvgl_port_unlock();

    while (1) vTaskDelay(pdMS_TO_TICKS(1000));
}
