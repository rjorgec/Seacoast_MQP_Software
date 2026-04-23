#pragma once
#include "esp_err.h"
#include <stdbool.h>
#include <stdint.h>

typedef enum { MOTOR_DIR_FWD = 0, MOTOR_DIR_REV = 1 } motor_dir_t;

// DRV8263: your “current monitor task”
esp_err_t motor_linact_start_monitor_dir(motor_dir_t dir, uint16_t speed,
                                         uint16_t low_th, uint16_t high_th,
                                         uint32_t interval_ms);

esp_err_t motor_linact_stop_monitor(void);
esp_err_t motor_linact_run(motor_dir_t dir, uint16_t speed);
esp_err_t motor_linact_stop(void);

/* ------------------------------------------------------------------ */
/*  State-based actuator commands (state machine, not atomic steps)    */
/* ------------------------------------------------------------------ */

/**
 * @brief Drive both flaps to their open-circuit endpoint (full extension).
 *        The Pico auto-stops when current drops (open-circuit endpoint).
 *        MSG_MOTION_DONE is sent asynchronously by the Pico when done.
 * @return ESP_OK if the command was ACK'd, error otherwise.
 */
esp_err_t motor_flap_open(void);

/**
 * @brief Drive both flaps closed until torque threshold is reached.
 *        MSG_MOTION_DONE is sent asynchronously by the Pico when done.
 * @return ESP_OK if the command was ACK'd, error otherwise.
 */
esp_err_t motor_flap_close(void);

/**
 * @brief Sensorlessly home the arm stepper against its positive hard stop,
 *        then back off by a fixed amount.
 * @return ESP_OK if the command was ACK'd, error otherwise.
 */
esp_err_t motor_arm_home(void);

/**
 * @brief Move the arm stepper to a named position.
 * @param pos  One of: ARM_POS_PRESS, ARM_POS_1, ARM_POS_2, ARM_POS_HOME
 *             ARM_POS_HOME moves to step 0 (released-home position established
 * by motor_arm_home()) without re-running the homing search.
 * @return ESP_OK if the command was ACK'd, error otherwise.
 */
esp_err_t motor_arm_move(uint8_t pos);

/**
 * @brief Move the rack stepper to a named position.
 * @param pos  One of: RACK_POS_HOME, RACK_POS_EXTEND, RACK_POS_PRESS
 * (rack_pos_t cast to uint8_t)
 * @return ESP_OK if the command was ACK'd, error otherwise.
 */
esp_err_t motor_rack_move(uint8_t pos);

/**
 * @brief Move the turntable stepper to a named absolute position.
 * @param pos  One of: TURNTABLE_POS_INTAKE=0, TURNTABLE_POS_TRASH=1,
 * TURNTABLE_POS_EJECT=2 TURNTABLE_POS_INTAKE is the physical hard endstop (same
 * as after motor_turntable_home()).
 * @return ESP_OK if the command was ACK'd, error otherwise.
 */
esp_err_t motor_turntable_goto(uint8_t pos);

/**
 * @brief Send turntable home command (zero position counter on Pico).
 * @return ESP_OK if the command was ACK'd, error otherwise.
 */
esp_err_t motor_turntable_home(void);

/**
 * @brief Enable or disable the hot wire constant-current PWM.
 * @param enable  true = on, false = off
 * @return ESP_OK if the command was ACK'd, error otherwise.
 */
esp_err_t motor_hotwire_set(bool enable);

/**
 * @brief Turn the vacuum pump on or off.
 * @param enable  true = on, false = off
 * @return ESP_OK if the command was ACK'd, error otherwise.
 */
esp_err_t motor_vacuum_set(bool enable);

/**
 * @brief Turn the second vacuum pump on or off.
 *        Drives the IN2 independent half-bridge output of the DRV8263.
 *        Can run simultaneously with the hot wire (IN1) — no interlock.
 * @param enable  true = on, false = off
 * @return ESP_OK if the command was ACK'd, error otherwise.
 */
esp_err_t motor_vacuum2_set(bool enable);

/**
 * @brief Traverse the hotwire stepper (STEPPER_DEV_HW_CARRIAGE, device 3).
 * @param cut  true = cut direction (forward), false = return direction
 *             (retrace to home; Pico zeroes traverse position on success)
 * @return ESP_OK if the command was ACK'd, error otherwise.
 *         Returns ESP_ERR_NOT_SUPPORTED if the device is not wired.
 *         MSG_MOTION_DONE is sent asynchronously by the Pico when done.
 */
esp_err_t motor_hotwire_traverse(bool cut);

/**
 * @brief Move the bag depth/eject rack (indexer) to a named position.
 * @param position  indexer_pos_t cast to uint8_t:
 *                  INDEXER_POS_OPEN=0, INDEXER_POS_CENTER=1,
 * INDEXER_POS_EJECT=2
 * @return ESP_OK if the command was ACK'd, error otherwise.
 *         Returns ESP_ERR_NOT_SUPPORTED if the device is not wired.
 *         MSG_MOTION_DONE is sent asynchronously by the Pico when done.
 */
esp_err_t motor_indexer_move(uint8_t position);

/**
 * @brief Trigger a standalone agitation cycle (AGITATOR_N_CYCLES
 * forward+reverse strokes) on the agitator eccentric arm (DRV8434S device 3).
 *
 * Sends MSG_AGITATE with len=0 (all defaults from board_pins.h; no homing).
 * MSG_MOTION_DONE(SUBSYS_AGITATOR) is sent asynchronously when complete.
 *
 * NOTE: Agitation should be used as a standalone pre-dose or between-cycle
 * maintenance step.  It is no longer automatically triggered inside the dosing
 * state machine (SPAWN_MAX_RETRIES / SPAWN_AGITATE_MS path).
 *
 * @return ESP_OK if the command was ACK'd, error otherwise.
 *         Returns ESP_ERR_NOT_SUPPORTED via NACK if the agitator is not wired.
 */
esp_err_t motor_agitate(void);

/**
 * @brief Home the agitator via sensorless stall-detect, then run the default
 *        agitation cycle count (AGITATOR_N_CYCLES).
 *
 * Sends MSG_AGITATE with pl_agitate_t{flags=AGITATE_FLAG_DO_HOME, n_cycles=0}.
 * The Pico drives the agitator toward its mechanical endstop (stall-detect
 * similar to MSG_ARM_HOME), backs off, then performs AGITATOR_N_CYCLES cycles.
 * MSG_MOTION_DONE(SUBSYS_AGITATOR) is sent asynchronously when complete.
 *
 * @return ESP_OK if the command was ACK'd, error otherwise.
 */
esp_err_t motor_agitate_home(void);
