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
    pl_drv8263_start_mon_t payload = {
        .speed = speed,
        .low_th = low_th,
        .high_th = high_th,
        .interval_ms = interval_ms,
        .dir = dir,
    };

    uint8_t nack_code = 0u;
    esp_err_t err = pico_link_send_rpc(MSG_MOTOR_DRV8263_START_MON,
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
    esp_err_t err = pico_link_send_rpc(MSG_MOTOR_DRV8263_STOP_MON,
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

// Soft torque limit for stepper motion (0 = disabled).
// This value is sent over UART to the Pico and used by the motion engine
// to stop a move when torque drops below the threshold.
#ifndef STEPPER_SOFT_TORQUE_LIMIT
#define STEPPER_SOFT_TORQUE_LIMIT PROTO_STEPPER_SOFT_TORQUE_LIMIT_DEFAULT
#endif

esp_err_t motor_stepper_enable(bool en)
{
    pl_stepper_enable_t payload = {.enable = en ? 1u : 0u};
    uint8_t nack_code = 0u;
    esp_err_t err = pico_link_send_rpc(MSG_MOTOR_STEPPER_ENABLE,
                                       &payload,
                                       (uint16_t)sizeof(payload),
                                       MOTOR_RPC_TIMEOUT_MS,
                                       &nack_code);
    if (err != ESP_OK)
    {
        ESP_LOGW(TAG, "stepper enable rpc failed (%s), nack=%u",
                 esp_err_to_name(err), (unsigned)nack_code);
    }
    return err;
}

esp_err_t motor_stepper_step(motor_dir_t dir, uint32_t steps, uint32_t step_delay_us)
{
    pl_stepper_stepjob_t payload = {
        .dir = (uint8_t)dir,
        .steps = steps,
        .step_delay_us = step_delay_us,
        .torque_limit = (uint16_t)STEPPER_SOFT_TORQUE_LIMIT,
    };

    uint8_t nack_code = 0u;
    esp_err_t err = pico_link_send_rpc(MSG_MOTOR_STEPPER_STEPJOB,
                                       &payload,
                                       (uint16_t)sizeof(payload),
                                       MOTOR_RPC_TIMEOUT_MS,
                                       &nack_code);
    if (err != ESP_OK)
    {
        ESP_LOGW(TAG, "stepper step rpc failed (%s), nack=%u",
                 esp_err_to_name(err), (unsigned)nack_code);
    }
    return err;
}

/* ------------------------------------------------------------------ */
/*  State-based actuator commands (state machine, not atomic steps)    */
/* ------------------------------------------------------------------ */

esp_err_t motor_flap_open(void)
{
    /* MSG_FLAPS_OPEN has no payload (len=0) */
    uint8_t nack_code = 0u;
    esp_err_t err = pico_link_send_rpc(MSG_FLAPS_OPEN,
                                       NULL,
                                       0u,
                                       MOTOR_RPC_TIMEOUT_MS,
                                       &nack_code);
    if (err != ESP_OK)
    {
        ESP_LOGW(TAG, "flap_open rpc failed (%s), nack=%u",
                 esp_err_to_name(err), (unsigned)nack_code);
    }
    return err;
}

esp_err_t motor_flap_close(void)
{
    /* MSG_FLAPS_CLOSE has no payload (len=0) */
    uint8_t nack_code = 0u;
    esp_err_t err = pico_link_send_rpc(MSG_FLAPS_CLOSE,
                                       NULL,
                                       0u,
                                       MOTOR_RPC_TIMEOUT_MS,
                                       &nack_code);
    if (err != ESP_OK)
    {
        ESP_LOGW(TAG, "flap_close rpc failed (%s), nack=%u",
                 esp_err_to_name(err), (unsigned)nack_code);
    }
    return err;
}

esp_err_t motor_arm_move(uint8_t pos)
{
    pl_arm_move_t pl = {.position = pos};
    uint8_t nack_code = 0u;
    esp_err_t err = pico_link_send_rpc(MSG_ARM_MOVE,
                                       &pl,
                                       (uint16_t)sizeof(pl),
                                       MOTOR_RPC_TIMEOUT_MS,
                                       &nack_code);
    if (err != ESP_OK)
    {
        ESP_LOGW(TAG, "arm_move rpc failed (%s), nack=%u",
                 esp_err_to_name(err), (unsigned)nack_code);
    }
    return err;
}

esp_err_t motor_rack_move(uint8_t pos)
{
    pl_rack_move_t pl = {.position = pos};
    uint8_t nack_code = 0u;
    esp_err_t err = pico_link_send_rpc(MSG_RACK_MOVE,
                                       &pl,
                                       (uint16_t)sizeof(pl),
                                       MOTOR_RPC_TIMEOUT_MS,
                                       &nack_code);
    if (err != ESP_OK)
    {
        ESP_LOGW(TAG, "rack_move rpc failed (%s), nack=%u",
                 esp_err_to_name(err), (unsigned)nack_code);
    }
    return err;
}

esp_err_t motor_turntable_goto(uint8_t pos)
{
    pl_turntable_goto_t pl = {.position = pos};
    uint8_t nack_code = 0u;
    esp_err_t err = pico_link_send_rpc(MSG_TURNTABLE_GOTO,
                                       &pl,
                                       (uint16_t)sizeof(pl),
                                       MOTOR_RPC_TIMEOUT_MS,
                                       &nack_code);
    if (err != ESP_OK)
    {
        ESP_LOGW(TAG, "turntable_goto rpc failed (%s), nack=%u",
                 esp_err_to_name(err), (unsigned)nack_code);
    }
    return err;
}

esp_err_t motor_turntable_home(void)
{
    /* MSG_TURNTABLE_HOME has no payload (len=0) */
    uint8_t nack_code = 0u;
    esp_err_t err = pico_link_send_rpc(MSG_TURNTABLE_HOME,
                                       NULL,
                                       0u,
                                       MOTOR_RPC_TIMEOUT_MS,
                                       &nack_code);
    if (err != ESP_OK)
    {
        ESP_LOGW(TAG, "turntable_home rpc failed (%s), nack=%u",
                 esp_err_to_name(err), (unsigned)nack_code);
    }
    return err;
}

esp_err_t motor_hotwire_set(bool enable)
{
    pl_hotwire_set_t pl = {.enable = enable ? 1u : 0u};
    uint8_t nack_code = 0u;
    esp_err_t err = pico_link_send_rpc(MSG_HOTWIRE_SET,
                                       &pl,
                                       (uint16_t)sizeof(pl),
                                       MOTOR_RPC_TIMEOUT_MS,
                                       &nack_code);
    if (err != ESP_OK)
    {
        ESP_LOGW(TAG, "hotwire_set rpc failed (%s), nack=%u",
                 esp_err_to_name(err), (unsigned)nack_code);
    }
    return err;
}

esp_err_t motor_vacuum_set(bool enable)
{
    pl_vacuum_set_t pl = {.enable = enable ? 1u : 0u};
    uint8_t nack_code = 0u;
    esp_err_t err = pico_link_send_rpc(MSG_VACUUM_SET,
                                       &pl,
                                       (uint16_t)sizeof(pl),
                                       MOTOR_RPC_TIMEOUT_MS,
                                       &nack_code);
    if (err != ESP_OK)
    {
        ESP_LOGW(TAG, "vacuum_set rpc failed (%s), nack=%u",
                 esp_err_to_name(err), (unsigned)nack_code);
    }
    return err;
}

esp_err_t motor_vacuum2_set(bool enable)
{
    pl_vacuum2_set_t pl = {.enable = enable ? 1u : 0u};
    uint8_t nack_code = 0u;
    esp_err_t err = pico_link_send_rpc(MSG_VACUUM2_SET,
                                       &pl,
                                       (uint16_t)sizeof(pl),
                                       MOTOR_RPC_TIMEOUT_MS,
                                       &nack_code);
    if (err != ESP_OK)
    {
        ESP_LOGW(TAG, "vacuum2_set rpc failed (%s), nack=%u",
                 esp_err_to_name(err), (unsigned)nack_code);
    }
    return err;
}

esp_err_t motor_hotwire_traverse(bool cut)
{
    pl_hotwire_traverse_t pl = {.direction = cut ? 0u : 1u};
    uint8_t nack_code = 0u;
    esp_err_t err = pico_link_send_rpc(MSG_HOTWIRE_TRAVERSE,
                                       &pl,
                                       (uint16_t)sizeof(pl),
                                       MOTOR_RPC_TIMEOUT_MS,
                                       &nack_code);
    if (err != ESP_OK)
    {
        ESP_LOGW(TAG, "hotwire_traverse rpc failed (%s), nack=%u",
                 esp_err_to_name(err), (unsigned)nack_code);
    }
    return err;
}

esp_err_t motor_indexer_move(uint8_t position)
{
    pl_indexer_move_t pl = {.position = position};
    uint8_t nack_code = 0u;
    esp_err_t err = pico_link_send_rpc(MSG_INDEXER_MOVE,
                                       &pl,
                                       (uint16_t)sizeof(pl),
                                       MOTOR_RPC_TIMEOUT_MS,
                                       &nack_code);
    if (err != ESP_OK)
    {
        ESP_LOGW(TAG, "indexer_move rpc failed (%s), nack=%u",
                 esp_err_to_name(err), (unsigned)nack_code);
    }
    return err;
}
