#include "motor_hal.h"

#include "pico_link.h"
#include "proto/proto.h"
#include "esp_log.h"

static const char *TAG = "motor_hal_pico";

#define MOTOR_RPC_TIMEOUT_MS 300u

// Defaults for the “quick UI buttons”
#define LINACT_LOW_TH 0u
#define LINACT_HIGH_TH 4095u
#define LINACT_INTERVAL_MS 50u

static esp_err_t linact_start_dir(uint8_t dir,
                                  uint16_t speed,
                                  uint16_t low_th,
                                  uint16_t high_th,
                                  uint32_t interval_ms)
{
    pl_drv8163_start_mon_t payload = {
        .speed = speed,
        .low_th = low_th,
        .high_th = high_th,
        .interval_ms = interval_ms,
        .dir = dir,
    };

    uint8_t nack_code = 0u;
    esp_err_t err = pico_link_send_rpc(MSG_MOTOR_DRV8163_START_MON,
                                       &payload,
                                       (uint16_t)sizeof(payload),
                                       MOTOR_RPC_TIMEOUT_MS,
                                       &nack_code);

    if (err != ESP_OK)
    {
        ESP_LOGW(TAG, "linact start rpc failed (%s), nack=%u",
                 esp_err_to_name(err), (unsigned)nack_code);
    }
    return err;
}

// Existing API (used by flap.c): keep it defaulting to FWD for now.
esp_err_t motor_linact_start_monitor_dir(motor_dir_t dir,
                                         uint16_t speed,
                                         uint16_t low_th,
                                         uint16_t high_th,
                                         uint32_t interval_ms)
{
    return linact_start_dir(dir, speed, low_th, high_th, interval_ms);
}

esp_err_t motor_linact_stop_monitor(void)
{
    uint8_t nack_code = 0u;
    esp_err_t err = pico_link_send_rpc(MSG_MOTOR_DRV8163_STOP_MON,
                                       NULL,
                                       0u,
                                       MOTOR_RPC_TIMEOUT_MS,
                                       &nack_code);

    if (err != ESP_OK)
    {
        ESP_LOGW(TAG, "linact stop rpc failed (%s), nack=%u",
                 esp_err_to_name(err), (unsigned)nack_code);
    }
    return err;
}

// NEW: simple UI-friendly calls
esp_err_t motor_linact_run(motor_dir_t dir, uint16_t speed)
{
    if (speed > 4095)
        speed = 4095;
    return linact_start_dir((uint8_t)dir, speed, LINACT_LOW_TH, LINACT_HIGH_TH, LINACT_INTERVAL_MS);
}

esp_err_t motor_linact_stop(void)
{
    return motor_linact_stop_monitor();
}

// Stepper not supported yet
esp_err_t motor_stepper_enable(bool en)
{
    (void)en;
    return ESP_ERR_NOT_SUPPORTED;
}

esp_err_t motor_stepper_step(motor_dir_t dir, uint32_t steps, uint32_t step_delay_us)
{
    (void)dir;
    (void)steps;
    (void)step_delay_us;
    return ESP_ERR_NOT_SUPPORTED;
}
