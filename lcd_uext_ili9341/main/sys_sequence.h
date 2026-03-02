/**
 * @file sys_sequence.h
 * @brief System-level inoculation sequence state machine (ESP32 / FreeRTOS).
 *
 * This module implements the whiteboard process flow as a FreeRTOS task.
 * It coordinates all subsystems (turntable, arm, rack, indexer, flaps,
 * hotwire, vacuum pumps, scale) to process bags end-to-end.
 *
 * Operator commands are submitted via sys_sequence_send_cmd().
 * The current state is readable via sys_sequence_get_state().
 * UI updates are delivered through the pico_link RX callback registered
 * in ui_screens.c (MSG_MOTION_DONE, MSG_SPAWN_STATUS, MSG_VACUUM_STATUS).
 *
 * Preferred dosing path: sends MSG_DISPENSE_SPAWN to the Pico and waits
 * for MSG_SPAWN_STATUS asynchronously.  The ESP32-side dosing.c module is
 * a legacy path kept for the manual dosing screen.
 */

#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C"
{
#endif

    /* ── System states ─────────────────────────────────────────────────────────── */

    typedef enum
    {
        SYS_IDLE = 0,                /**< Power on, no spawn bag loaded */
        SYS_SETUP_LOAD = 1,          /**< Operator loading spawn bag; subsystems to load positions */
        SYS_CUTTING_TIP = 2,         /**< Hot wire heating, flaps closing, cutting spawn bag tip */
        SYS_ROTATING_TO_ACCEPT = 3,  /**< Platform rotating to bag intake position */
        SYS_INTAKE_WAITING = 4,      /**< Indexer open, waiting for substrate bag (weight signal) */
        SYS_INTAKE_WEIGHING = 5,     /**< Bag detected, indexer closed, weighing bag */
        SYS_OPENING_BAG = 6,         /**< Multi-step bag opening sub-sequence */
        SYS_INOCULATING = 7,         /**< Closed-loop spawn dispensing (Pico-side) */
        SYS_POST_DOSE = 8,           /**< Flaps closed, arms returning to neutral */
        SYS_EJECTING = 9,            /**< Platform to eject, pushing bag out */
        SYS_ROTATING_TO_INTAKE = 10, /**< Platform returning to accept position */
        SYS_SPAWN_EMPTY = 11,        /**< Spawn exhausted; prompt operator */
        SYS_CONTINUE_RESTART = 12,   /**< Replacing spawn: rotate to trash, open flaps, restart */
        SYS_ERROR = 13,              /**< Unrecoverable error; safe-stop all */
        SYS_ESTOP = 14,              /**< Emergency stop */
    } sys_state_t;

    /* ── Operator commands submitted to the sequence task queue ────────────────── */

    typedef enum
    {
        SYS_CMD_SETUP_LOAD = 0,    /**< "Setup/Load" button pressed */
        SYS_CMD_START = 1,         /**< "Start" button pressed */
        SYS_CMD_ABORT = 2,         /**< "Abort" button pressed */
        SYS_CMD_REPLACE_SPAWN = 3, /**< "Replace Spawn" button pressed */
        SYS_CMD_ESTOP = 4,         /**< Emergency stop */
    } sys_cmd_t;

    /* ── Public API ────────────────────────────────────────────────────────────── */

    /**
     * @brief Initialise and start the system sequence FreeRTOS task.
     *
     * Must be called once from app_main() after pico_link and motor_hal are ready.
     * @return ESP_OK on success, ESP_FAIL if task creation failed.
     */
    esp_err_t sys_sequence_init(void);

    /**
     * @brief Send an operator command to the sequence task.
     *
     * Thread-safe; can be called from any FreeRTOS task or ISR context.
     * @param cmd  The command to queue.
     * @return ESP_OK if the command was enqueued, ESP_ERR_TIMEOUT if the queue is full.
     */
    esp_err_t sys_sequence_send_cmd(sys_cmd_t cmd);

    /**
     * @brief Return the current system state (thread-safe read).
     */
    sys_state_t sys_sequence_get_state(void);

    /**
     * @brief Return the number of bags inoculated this run (thread-safe read).
     */
    uint32_t sys_sequence_get_bag_count(void);

    /**
     * @brief Notify the sequence task that a MSG_MOTION_DONE was received.
     *
     * Called from the pico_link RX callback (ui_screens.c) when MSG_MOTION_DONE
     * arrives.  The sequence task waits on this notification when a motion command
     * has been sent and must complete before the next step.
     *
     * @param subsystem  subsystem_id_t value from the received message.
     * @param result     motion_result_t value from the received message.
     */
    void sys_sequence_notify_motion_done(uint8_t subsystem, uint8_t result);

    /**
     * @brief Notify the sequence task that a MSG_SPAWN_STATUS was received.
     *
     * Called from the pico_link RX callback when MSG_SPAWN_STATUS arrives.
     * @param status  spawn_status_code_t value from the received message.
     */
    void sys_sequence_notify_spawn_status(uint8_t status);

    /**
     * @brief Return a human-readable string for the given system state.
     */
    const char *sys_sequence_state_name(sys_state_t state);

#ifdef __cplusplus
}
#endif
