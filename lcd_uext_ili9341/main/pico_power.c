#include "pico_power.h"

#include "driver/gpio.h"
#include "esp_log.h"

#define PICO_POWER_RELAY_GPIO 10

static const char *TAG = "pico_power";
static bool s_enabled = false;

esp_err_t pico_power_init(void)
{
    gpio_config_t cfg = {
        .pin_bit_mask = 1ULL << PICO_POWER_RELAY_GPIO,
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    esp_err_t err = gpio_config(&cfg);
    if (err != ESP_OK)
    {
        return err;
    }
    err = gpio_set_level(PICO_POWER_RELAY_GPIO, 0);
    s_enabled = false;
    ESP_LOGI(TAG, "GPIO%d configured, relay OPEN (12 V off)", PICO_POWER_RELAY_GPIO);
    return err;
}

esp_err_t pico_power_enable(void)
{
    esp_err_t err = gpio_set_level(PICO_POWER_RELAY_GPIO, 1);
    if (err == ESP_OK)
    {
        s_enabled = true;
        ESP_LOGI(TAG, "relay CLOSED — 12 V applied to motor drivers");
    }
    return err;
}

esp_err_t pico_power_disable(void)
{
    esp_err_t err = gpio_set_level(PICO_POWER_RELAY_GPIO, 0);
    if (err == ESP_OK)
    {
        s_enabled = false;
        ESP_LOGW(TAG, "relay OPEN — motor driver power dropped");
    }
    return err;
}

bool pico_power_is_enabled(void)
{
    return s_enabled;
}
