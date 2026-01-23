#include <stdio.h>
#include <assert.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_log.h"
#include "esp_err.h"
#include "driver/spi_master.h"

#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_ili9341.h"

#include "lvgl.h"
#include "esp_lvgl_port.h"

static const char *TAG = "lcd_uext";

//ESP32-C6-EVB UEXT -> GPIO mapping
#define PIN_NUM_MOSI   18
#define PIN_NUM_CLK    19
#define PIN_NUM_CS     21
//IMPORTANT: MOD-LCD2.8RTP uses the UEXT "MISO" wire as D/C
#define PIN_NUM_DC     20
//No reset line on UEXT by default
#define PIN_NUM_RST    -1

#define LCD_HOST       SPI2_HOST

//ILI9341 native resolution (portrait)
#define LCD_H_RES      320
#define LCD_V_RES      240

//LVGL 9 uses lv_screen_active(). LVGL 8 uses lv_scr_act().
#if defined(LVGL_VERSION_MAJOR) && (LVGL_VERSION_MAJOR >= 9)
#define LVGL_ACTIVE_SCREEN() lv_screen_active()
#else
#define LVGL_ACTIVE_SCREEN() lv_scr_act()
#endif

void app_main(void)
{
    ESP_LOGI(TAG, "Initialize SPI bus");

    //LVGL buffer size: number of pixels (not bytes) in esp_lvgl_port config
    //bus max_transfer_sz is in bytes
    const int slice_h = 40;
    const size_t max_transfer_bytes = LCD_H_RES * slice_h * sizeof(uint16_t);

    //macros from esp_lcd_ili9341 component (keeps MISO disabled internally)
    const spi_bus_config_t bus_config =
        ILI9341_PANEL_BUS_SPI_CONFIG(PIN_NUM_CLK, PIN_NUM_MOSI, max_transfer_bytes);

    ESP_ERROR_CHECK(spi_bus_initialize(LCD_HOST, &bus_config, SPI_DMA_CH_AUTO));

    ESP_LOGI(TAG, "Install panel IO");
    esp_lcd_panel_io_handle_t io_handle = NULL;

    //macros from esp_lcd_ili9341 component (DC + CS set here)
    const esp_lcd_panel_io_spi_config_t io_config =
        ILI9341_PANEL_IO_SPI_CONFIG(PIN_NUM_CS, PIN_NUM_DC, NULL, NULL);

    ESP_ERROR_CHECK(esp_lcd_new_panel_io_spi((esp_lcd_spi_bus_handle_t)LCD_HOST,
                                            &io_config, &io_handle));

    ESP_LOGI(TAG, "Install ILI9341 panel driver");
    esp_lcd_panel_handle_t panel_handle = NULL;

    const esp_lcd_panel_dev_config_t panel_config = {
        .reset_gpio_num = PIN_NUM_RST,                // -1 if not used
        .rgb_ele_order  = LCD_RGB_ELEMENT_ORDER_RGB,  // R-G-B
        .bits_per_pixel = 16,
    };

    ESP_ERROR_CHECK(esp_lcd_new_panel_ili9341(io_handle, &panel_config, &panel_handle));
    ESP_ERROR_CHECK(esp_lcd_panel_reset(panel_handle));
    ESP_ERROR_CHECK(esp_lcd_panel_init(panel_handle));
    ESP_ERROR_CHECK(esp_lcd_panel_disp_on_off(panel_handle, true));

 
    ESP_ERROR_CHECK(esp_lcd_panel_invert_color(panel_handle, true)); //comment uncomment for invert

    // ---------------- LVGL glue ----------------
    ESP_LOGI(TAG, "Init LVGL port");
    const lvgl_port_cfg_t lvgl_cfg = ESP_LVGL_PORT_INIT_CONFIG();
    ESP_ERROR_CHECK(lvgl_port_init(&lvgl_cfg));

    ESP_LOGI(TAG, "Register LVGL display");
    lvgl_port_display_cfg_t disp_cfg = {0};   
    disp_cfg.io_handle = io_handle;
    disp_cfg.panel_handle = panel_handle;

    disp_cfg.hres = LCD_H_RES;
    disp_cfg.vres = LCD_V_RES;

    //buffer size in pixels
    disp_cfg.buffer_size = disp_cfg.hres * 40;
    disp_cfg.double_buffer = true;
    disp_cfg.monochrome = false;
    disp_cfg.color_format = LV_COLOR_FORMAT_RGB565;

    //rotation flags
    disp_cfg.rotation.swap_xy = true;
    disp_cfg.rotation.mirror_x = true;
    disp_cfg.rotation.mirror_y = true;

    //dma buffers
    disp_cfg.flags.buff_dma = true;
    disp_cfg.flags.swap_bytes = false;

    lv_disp_t *disp = lvgl_port_add_disp(&disp_cfg);
    assert(disp);

    // ---------------- Simple UI ----------------
    lvgl_port_lock(0);
    lv_obj_t *label = lv_label_create(LVGL_ACTIVE_SCREEN());
    lv_label_set_text(label, "Hello from ESP32-C6 + ILI9341!\n(LVGL running)");
    lv_obj_center(label);
    lvgl_port_unlock();

    ESP_LOGI(TAG, "LVGL UI created");

    //lvgl runs in its own task, this keeps main alive
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
