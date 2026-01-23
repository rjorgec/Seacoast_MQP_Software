#include "display.h"
#include "esp_log.h"
#include "driver/spi_master.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_ili9341.h"

static const char *TAG = "display";

//UEXT mapping
#define PIN_NUM_MOSI   18
#define PIN_NUM_CLK    19
#define PIN_NUM_CS     21
#define PIN_NUM_DC     20
#define PIN_NUM_RST    -1
#define LCD_HOST       SPI2_HOST

//landscape
#define LCD_H_RES      320
#define LCD_V_RES      240

display_handles_t display_init(void)
{
    const int slice_h = 40;
    const size_t max_transfer_bytes = LCD_H_RES * slice_h * sizeof(uint16_t);

    spi_bus_config_t bus_config =
        ILI9341_PANEL_BUS_SPI_CONFIG(PIN_NUM_CLK, PIN_NUM_MOSI, max_transfer_bytes);

    ESP_LOGI(TAG, "Init SPI bus");
    ESP_ERROR_CHECK(spi_bus_initialize(LCD_HOST, &bus_config, SPI_DMA_CH_AUTO));

    ESP_LOGI(TAG, "Init panel IO");
    esp_lcd_panel_io_handle_t io = NULL;
    esp_lcd_panel_io_spi_config_t io_config =
        ILI9341_PANEL_IO_SPI_CONFIG(PIN_NUM_CS, PIN_NUM_DC, NULL, NULL);

    ESP_ERROR_CHECK(esp_lcd_new_panel_io_spi((esp_lcd_spi_bus_handle_t)LCD_HOST, &io_config, &io));

    ESP_LOGI(TAG, "Init ILI9341 panel");
    esp_lcd_panel_handle_t panel = NULL;
    esp_lcd_panel_dev_config_t panel_config = {
        .reset_gpio_num = PIN_NUM_RST,
        .rgb_ele_order  = LCD_RGB_ELEMENT_ORDER_RGB,
        .bits_per_pixel = 16,
    };

    ESP_ERROR_CHECK(esp_lcd_new_panel_ili9341(io, &panel_config, &panel));
    ESP_ERROR_CHECK(esp_lcd_panel_reset(panel));
    ESP_ERROR_CHECK(esp_lcd_panel_init(panel));
    ESP_ERROR_CHECK(esp_lcd_panel_disp_on_off(panel, true));


    // ESP_ERROR_CHECK(esp_lcd_panel_swap_xy(panel, true)); //old panel transforms
    // ESP_ERROR_CHECK(esp_lcd_panel_mirror(panel, true, true));
    ESP_ERROR_CHECK(esp_lcd_panel_invert_color(panel, true));
    display_handles_t out = {
        .io = io,
        .panel = panel,
        .hres = LCD_H_RES,
        .vres = LCD_V_RES,
    };
    return out;
}
