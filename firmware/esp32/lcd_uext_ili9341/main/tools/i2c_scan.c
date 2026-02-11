#include <stdio.h>
#include "esp_log.h"
#include "esp_err.h"
#include "driver/gpio.h"
#include "driver/i2c_master.h"

static const char *TAG = "i2c_scan";


#define I2C_SDA_GPIO   6
#define I2C_SCL_GPIO   7

static void log_line_levels(const char *when)
{
    int sda = gpio_get_level(I2C_SDA_GPIO);
    int scl = gpio_get_level(I2C_SCL_GPIO);
    ESP_LOGI(TAG, "%s: SDA=%d SCL=%d (expect both = 1 when idle)", when, sda, scl);
}

esp_err_t i2c_scan_run(void)
{
    ESP_LOGI(TAG, "Starting I2C scan on SDA=%d SCL=%d", I2C_SDA_GPIO, I2C_SCL_GPIO);

    gpio_reset_pin(I2C_SDA_GPIO);
    gpio_reset_pin(I2C_SCL_GPIO);
    gpio_set_direction(I2C_SDA_GPIO, GPIO_MODE_INPUT);
    gpio_set_direction(I2C_SCL_GPIO, GPIO_MODE_INPUT);


    log_line_levels("Before bus init");

    i2c_master_bus_handle_t bus_handle = NULL;

    i2c_master_bus_config_t bus_cfg = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        //-1 lets IDF choose a free I2C controller
        .i2c_port = -1,
        .sda_io_num = I2C_SDA_GPIO,
        .scl_io_num = I2C_SCL_GPIO,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,  
    };

    ESP_ERROR_CHECK(i2c_new_master_bus(&bus_cfg, &bus_handle));

    log_line_levels("After bus init (internal pullups enabled)");

    int found = 0;

    //scan addresses (7 bit)
    for (uint8_t addr = 1; addr < 0x7F; addr++) {
        
        esp_err_t err = i2c_master_probe(bus_handle, addr, 100);

        if (err == ESP_OK) {
            ESP_LOGI(TAG, "Found device at 0x%02X", addr);
            found++;
        } else if (err == ESP_ERR_TIMEOUT) {
        
        }
    }

    if (!found) {
        ESP_LOGW(TAG,
            "No I2C devices found.\n"
            "If you saw many ESP_ERR_TIMEOUT probes, assume missing/weak pull-ups.\n"
            "MOD-LCD2.8RTP touch is typically AR1021 @ 0x4D :contentReference[oaicite:6]{index=6} or NS2009 @ 0x48/0x49 :contentReference[oaicite:7]{index=7}.");
    } else {
        ESP_LOGI(TAG, "I2C scan done: %d device(s) found", found);
    }

    ESP_ERROR_CHECK(i2c_del_master_bus(bus_handle));
    return ESP_OK;
}
