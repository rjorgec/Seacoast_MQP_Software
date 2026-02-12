#include "motor_hal.h"

#include "pico_link.h"
#include "proto/proto.h"

#define MOTOR_RPC_TIMEOUT_MS 300u

esp_err_t motor_linact_start_monitor(uint16_t speed,
                                     uint16_t low_th,
                                     uint16_t high_th,
                                     uint32_t interval_ms) {
    pl_drv8163_start_mon_t payload = {
        .speed = speed,
        .low_th = low_th,
        .high_th = high_th,
        .interval_ms = interval_ms,
    };

    uint8_t nack_code = 0u;
    esp_err_t err = pico_link_send_rpc(MSG_MOTOR_DRV8163_START_MON,
                                       &payload,
                                       (uint16_t)sizeof(payload),
                                       MOTOR_RPC_TIMEOUT_MS,
                                       &nack_code);
    (void)nack_code;
    return err;
}

esp_err_t motor_linact_stop_monitor(void) {
    uint8_t nack_code = 0u;
    esp_err_t err = pico_link_send_rpc(MSG_MOTOR_DRV8163_STOP_MON,
                                       NULL,
                                       0u,
                                       MOTOR_RPC_TIMEOUT_MS,
                                       &nack_code);
    (void)nack_code;
    return err;
}

esp_err_t motor_stepper_enable(bool en) {
    (void)en;
    return ESP_ERR_NOT_SUPPORTED;
}

esp_err_t motor_stepper_step(motor_dir_t dir, uint32_t steps, uint32_t step_delay_us) {
    (void)dir;
    (void)steps;
    (void)step_delay_us;
    return ESP_ERR_NOT_SUPPORTED;
}
