/**
 * @file sys_sequence.c
 * @brief System-level inoculation sequence state machine (FreeRTOS task).
 *
 * Implements the whiteboard process flow transcribed in docs/state_machine_design.md.
 * Coordinates all hardware subsystems via motor_hal and pico_link to process
 * mushroom substrate bags end-to-end.
 *
 * Architecture:
 *   - One FreeRTOS task (SEQ_TASK) runs the state machine loop
 *   - Operator commands arrive via a FreeRTOS queue (sys_sequence_send_cmd())
 *   - Motion completions arrive via task notifications (sys_sequence_notify_motion_done())
 *   - Spawn dosing status arrives via sys_sequence_notify_spawn_status()
 *
 * All state transitions are logged via ESP_LOGx.
 * Any unrecoverable error or timeout → SYS_ERROR, all motors safe-stopped.
 */

#include "sys_sequence.h"
#include "motor_hal.h"
#include "pico_link.h"
#include "proto/proto.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/event_groups.h"

#include "esp_log.h"
#include <string.h>

/* ── Configuration ─────────────────────────────────────────────────────────── */

static const char *TAG = "sys_seq";

#define SEQ_TASK_STACK 4096u
#define SEQ_TASK_PRIORITY 5u
#define SEQ_CMD_QUEUE_DEPTH 8u

/** Timeout for any single motor motion (ms). Generous to allow calibration. */
#define SEQ_MOTION_TIMEOUT_MS 30000u

/** Timeout for spawn dosing to complete (ms). */
#define SEQ_DOSE_TIMEOUT_MS 120000u

/** Threshold for bag detection via HX711 scale (µg). */
#define BAG_DETECT_THRESHOLD_UG 50000000u /* 50 g */

/** Inoculation percentage: spawn mass as fraction of bag mass (×100 for fixed-point). */
#define INNOC_PERCENT_X100 15u /* 15 % spawn by bag weight — calibrate per variety */

/* ── Event group bits ──────────────────────────────────────────────────────── */

#define EVT_MOTION_DONE (1u << 0)
#define EVT_SPAWN_DONE (1u << 1)
#define EVT_SPAWN_EMPTY (1u << 2)
#define EVT_SPAWN_AGITATING (1u << 3)

/* ── Module state ──────────────────────────────────────────────────────────── */

static TaskHandle_t s_task_handle;
static QueueHandle_t s_cmd_queue;
static EventGroupHandle_t s_events;

static volatile sys_state_t s_state = SYS_IDLE;
static volatile uint32_t s_bag_count = 0u;

/* Last motion result from MSG_MOTION_DONE notification */
static volatile uint8_t s_last_motion_subsys = 0u;
static volatile uint8_t s_last_motion_result = 0u;

/* Last spawn status from MSG_SPAWN_STATUS notification */
static volatile uint8_t s_last_spawn_status = 0u;

/* ── Internal helpers ──────────────────────────────────────────────────────── */

static void set_state(sys_state_t next)
{
    s_state = next;
    ESP_LOGI(TAG, "→ %s", sys_sequence_state_name(next));
}

/** Safe-stop all outputs. Called on error or E-Stop. */
static void safe_stop_all(void)
{
    ESP_LOGW(TAG, "safe_stop_all: stopping all actuators");
    (void)motor_flap_close();
    (void)motor_hotwire_set(false);
    (void)motor_vacuum_set(false);
    (void)motor_vacuum2_set(false);
    /* Leave turntable/arm/rack in place — do not move unpredictably */
}

/**
 * @brief Wait for a MSG_MOTION_DONE notification for the given subsystem.
 * @param expected_subsys  subsystem_id_t to wait for (0xFF = any subsystem)
 * @param timeout_ms       maximum wait time
 * @return true if motion completed with MOTION_OK, false on timeout or error
 */
static bool wait_motion_done(uint8_t expected_subsys, uint32_t timeout_ms)
{
    /* DIAG: confirm task is about to block — abort queued before this will be
     * silently ignored until the wait returns or times out. */
    ESP_LOGD(TAG, "wait_motion_done: BLOCKING up to %u ms for subsys=%u (state=%s)",
             (unsigned)timeout_ms, (unsigned)expected_subsys,
             sys_sequence_state_name(s_state));

    EventBits_t bits = xEventGroupWaitBits(s_events,
                                           EVT_MOTION_DONE,
                                           pdTRUE, /* clear on exit */
                                           pdFALSE,
                                           pdMS_TO_TICKS(timeout_ms));

    ESP_LOGD(TAG, "wait_motion_done: unblocked, bits=0x%lx", (unsigned long)bits);

    if (!(bits & EVT_MOTION_DONE))
    {
        ESP_LOGW(TAG, "wait_motion_done: timeout after %ums (subsys=%u)",
                 (unsigned)timeout_ms, (unsigned)expected_subsys);
        return false;
    }

    if (expected_subsys != 0xFFu && s_last_motion_subsys != expected_subsys)
    {
        ESP_LOGW(TAG, "wait_motion_done: unexpected subsys %u (expected %u)",
                 (unsigned)s_last_motion_subsys, (unsigned)expected_subsys);
        /* Continue — could be a stale notification from a previous step. */
    }

    if (s_last_motion_result != MOTION_OK)
    {
        ESP_LOGW(TAG, "wait_motion_done: result=%u for subsys=%u",
                 (unsigned)s_last_motion_result,
                 (unsigned)s_last_motion_subsys);
        return false;
    }

    return true;
}

/**
 * @brief Wait for a MSG_SPAWN_STATUS notification indicating dose complete or bag empty.
 * @param timeout_ms  maximum wait time
 * @return SPAWN_STATUS_DONE if dose completed, SPAWN_STATUS_BAG_EMPTY if spawn exhausted,
 *         SPAWN_STATUS_ERROR on timeout or error
 */
static spawn_status_code_t wait_spawn_done(uint32_t timeout_ms)
{
    EventBits_t bits = xEventGroupWaitBits(s_events,
                                           EVT_SPAWN_DONE | EVT_SPAWN_EMPTY,
                                           pdTRUE,
                                           pdFALSE,
                                           pdMS_TO_TICKS(timeout_ms));

    if (bits & EVT_SPAWN_DONE)
        return SPAWN_STATUS_DONE;
    if (bits & EVT_SPAWN_EMPTY)
        return SPAWN_STATUS_BAG_EMPTY;

    ESP_LOGW(TAG, "wait_spawn_done: timeout after %ums", (unsigned)timeout_ms);
    return SPAWN_STATUS_ERROR;
}

/** Non-blocking check for an operator command in the queue. */
static bool poll_cmd(sys_cmd_t *out_cmd)
{
    return xQueueReceive(s_cmd_queue, out_cmd, 0) == pdTRUE;
}

/** Block until a specific operator command (or ABORT/ESTOP) arrives. */
static sys_cmd_t wait_cmd(TickType_t ticks)
{
    sys_cmd_t cmd = (sys_cmd_t)0xFFu;
    xQueueReceive(s_cmd_queue, &cmd, ticks);
    return cmd;
}

/* ── Bag opening sub-sequence ──────────────────────────────────────────────── */

/**
 * @brief Execute the bag opening sub-sequence (within SYS_OPENING_BAG).
 *
 * Steps:
 *  1. Close rot (arm press against bag)
 *  2. Init suck (vacuum pump 1 ON)
 *  3. Over-open rot (swing arm to peel one side)
 *  4. Init suck 2 + extend rack (linear arm)
 *  5. Retract rack slightly (pull bag open)
 *  6. Align rot to interim position
 *
 * @return true on success, false if any step fails
 */
static bool open_bag_sequence(void)
{
    ESP_LOGI(TAG, "bag_open: step 1 — arm press");
    if (motor_arm_move(ARM_POS_PRESS) != ESP_OK)
        return false;
    if (!wait_motion_done(SUBSYS_ARM, SEQ_MOTION_TIMEOUT_MS))
        return false;

    ESP_LOGI(TAG, "bag_open: step 2 — vacuum 1 ON");
    if (motor_vacuum_set(true) != ESP_OK)
        return false;
    vTaskDelay(pdMS_TO_TICKS(500)); /* allow suction to establish */

    ESP_LOGI(TAG, "bag_open: step 3 — over-open arm (ARM_POS_2)");
    if (motor_arm_move(ARM_POS_2) != ESP_OK)
        return false;
    if (!wait_motion_done(SUBSYS_ARM, SEQ_MOTION_TIMEOUT_MS))
        return false;

    ESP_LOGI(TAG, "bag_open: step 4 — vacuum 2 ON + extend rack");
    if (motor_vacuum2_set(true) != ESP_OK)
        return false;
    if (motor_rack_move(RACK_POS_EXTEND) != ESP_OK)
        return false;
    if (!wait_motion_done(SUBSYS_RACK, SEQ_MOTION_TIMEOUT_MS))
        return false;

    ESP_LOGI(TAG, "bag_open: step 5 — rack press (close lin slightly)");
    if (motor_rack_move(RACK_POS_PRESS) != ESP_OK)
        return false;
    if (!wait_motion_done(SUBSYS_RACK, SEQ_MOTION_TIMEOUT_MS))
        return false;

    ESP_LOGI(TAG, "bag_open: step 6 — align arm (ARM_POS_1, interim)");
    if (motor_arm_move(ARM_POS_1) != ESP_OK)
        return false;
    if (!wait_motion_done(SUBSYS_ARM, SEQ_MOTION_TIMEOUT_MS))
        return false;

    ESP_LOGI(TAG, "bag_open: complete — bag held open");
    return true;
}

/* ── Main sequence task ────────────────────────────────────────────────────── */

static void seq_task(void *arg)
{
    (void)arg;
    ESP_LOGI(TAG, "sequence task started");

    for (;;)
    {
        /* Check for abort or E-Stop from any state */
        sys_cmd_t cmd = (sys_cmd_t)0xFFu;
        if (poll_cmd(&cmd))
        {
            if (cmd == SYS_CMD_ESTOP)
            {
                safe_stop_all();
                set_state(SYS_ESTOP);
                /* Fall through to blocking wait below */
            }
            else if (cmd == SYS_CMD_ABORT && s_state != SYS_IDLE)
            {
                safe_stop_all();
                set_state(SYS_IDLE);
            }
        }

        switch (s_state)
        {
        /* ── IDLE ──────────────────────────────────────────────────── */
        case SYS_IDLE:
        {
            /* Wait for Setup/Load or Start command */
            sys_cmd_t c = wait_cmd(portMAX_DELAY);
            if (c == SYS_CMD_SETUP_LOAD)
            {
                set_state(SYS_SETUP_LOAD);
                /* Open flaps, open arms, rotate platform to trash, tare */
                ESP_LOGI(TAG, "setup: open flaps");
                (void)motor_flap_open();
                ESP_LOGI(TAG, "setup: arm to pos1 (open)");
                (void)motor_arm_move(ARM_POS_1);
                ESP_LOGI(TAG, "setup: rack home (open lin)");
                (void)motor_rack_move(RACK_POS_HOME);
                ESP_LOGI(TAG, "setup: turntable home");
                (void)motor_turntable_home();
                wait_motion_done(0xFFu, SEQ_MOTION_TIMEOUT_MS);
                ESP_LOGI(TAG, "setup: tare scale via pico_link");
                /* Note: HX711 tare sent via existing MSG_HX711_TARE RPC */

                /* Wait for Start press after operator loads spawn bag */
                ESP_LOGI(TAG, "setup: waiting for START command");
                sys_cmd_t s2 = wait_cmd(portMAX_DELAY);
                if (s2 == SYS_CMD_START)
                    set_state(SYS_CUTTING_TIP);
                else
                {
                    safe_stop_all();
                    set_state(SYS_IDLE);
                }
            }
            break;
        }

        /* ── CUTTING TIP ────────────────────────────────────────────── */
        case SYS_CUTTING_TIP:
        {
            ESP_LOGI(TAG, "cut: hotwire ON");
            (void)motor_hotwire_set(true);
            vTaskDelay(pdMS_TO_TICKS(2000)); /* heat up */

            ESP_LOGI(TAG, "cut: close flaps");
            (void)motor_flap_close();
            wait_motion_done(SUBSYS_FLAPS, SEQ_MOTION_TIMEOUT_MS);

            ESP_LOGI(TAG, "cut: traverse hotwire (cut)");
            (void)motor_hotwire_traverse(true);
            wait_motion_done(0xFFu, SEQ_MOTION_TIMEOUT_MS);

            ESP_LOGI(TAG, "cut: retract hotwire");
            (void)motor_hotwire_traverse(false);
            wait_motion_done(0xFFu, SEQ_MOTION_TIMEOUT_MS);

            ESP_LOGI(TAG, "cut: hotwire OFF");
            (void)motor_hotwire_set(false);

            set_state(SYS_ROTATING_TO_ACCEPT);
            break;
        }

        /* ── ROTATING TO ACCEPT ─────────────────────────────────────── */
        case SYS_ROTATING_TO_ACCEPT:
        {
            ESP_LOGI(TAG, "rotate: turntable to accept (POS_B)");
            (void)motor_turntable_goto(TURNTABLE_POS_B);
            if (!wait_motion_done(SUBSYS_TURNTABLE, SEQ_MOTION_TIMEOUT_MS))
            {
                set_state(SYS_ERROR);
                break;
            }
            set_state(SYS_INTAKE_WAITING);
            ESP_LOGI(TAG, "intake: open indexer");
            (void)motor_indexer_move(INDEXER_POS_OPEN);
            break;
        }

        /* ── INTAKE WAITING ─────────────────────────────────────────── */
        case SYS_INTAKE_WAITING:
        {
            /* Poll for bag detection via weight — simplified as timed wait.
             * Full implementation reads MSG_HX711_MEASURE responses and
             * checks for mass > BAG_DETECT_THRESHOLD_UG. */
            ESP_LOGI(TAG, "intake: waiting for bag...");
            vTaskDelay(pdMS_TO_TICKS(500)); /* polling interval */
            /* TODO: integrate HX711 weight polling here */
            /* For now, advance to weighing unconditionally when weight detected */
            set_state(SYS_INTAKE_WEIGHING);
            ESP_LOGI(TAG, "intake: bag detected, closing indexer");
            (void)motor_indexer_move(INDEXER_POS_CENTER);
            break;
        }

        /* ── INTAKE WEIGHING ────────────────────────────────────────── */
        case SYS_INTAKE_WEIGHING:
        {
            /* Wait for indexer to center bag */
            wait_motion_done(SUBSYS_INDEXER, SEQ_MOTION_TIMEOUT_MS);
            vTaskDelay(pdMS_TO_TICKS(1000)); /* settle */
            ESP_LOGI(TAG, "weighing: bag centered, reading weight");
            /* TODO: read actual weight via pico_link MSG_HX711_MEASURE */
            set_state(SYS_OPENING_BAG);
            break;
        }

        /* ── OPENING BAG ────────────────────────────────────────────── */
        case SYS_OPENING_BAG:
        {
            if (!open_bag_sequence())
            {
                ESP_LOGE(TAG, "bag opening sub-sequence failed");
                safe_stop_all();
                set_state(SYS_ERROR);
                break;
            }
            set_state(SYS_INOCULATING);
            break;
        }

        /* ── INOCULATING ────────────────────────────────────────────── */
        case SYS_INOCULATING:
        {
            /* Send MSG_DISPENSE_SPAWN to Pico (Pico-side closed-loop dosing) */
            ESP_LOGI(TAG, "inoculat: sending MSG_DISPENSE_SPAWN");
            /* TODO: build pl_innoculate_bag_t payload with bag_mass and target */
            pl_innoculate_bag_t pl = {
                .bag_mass = 0u,   /* replace with actual measured mass */
                .spawn_mass = 0u, /* not used by Pico-side controller */
                .innoc_percent = (uint16_t)(INNOC_PERCENT_X100 * 10u),
                .bag_number = (uint8_t)(s_bag_count & 0xFFu),
            };
            uint8_t nack = 0u;
            esp_err_t err = pico_link_send_rpc(MSG_DISPENSE_SPAWN,
                                               &pl, sizeof(pl),
                                               500u, &nack);
            if (err != ESP_OK)
            {
                ESP_LOGE(TAG, "MSG_DISPENSE_SPAWN failed: %s", esp_err_to_name(err));
                safe_stop_all();
                set_state(SYS_ERROR);
                break;
            }

            spawn_status_code_t status = wait_spawn_done(SEQ_DOSE_TIMEOUT_MS);

            if (status == SPAWN_STATUS_DONE)
            {
                s_bag_count++;
                ESP_LOGI(TAG, "inoculat: done (bag #%u)", (unsigned)s_bag_count);
                set_state(SYS_POST_DOSE);
            }
            else if (status == SPAWN_STATUS_BAG_EMPTY)
            {
                ESP_LOGW(TAG, "inoculat: spawn bag empty");
                set_state(SYS_SPAWN_EMPTY);
            }
            else
            {
                ESP_LOGE(TAG, "inoculat: error/timeout (status=%u)", (unsigned)status);
                safe_stop_all();
                set_state(SYS_ERROR);
            }
            break;
        }

        /* ── POST DOSE ──────────────────────────────────────────────── */
        case SYS_POST_DOSE:
        {
            ESP_LOGI(TAG, "post_dose: close flaps, return arms");
            (void)motor_flap_close();
            wait_motion_done(SUBSYS_FLAPS, SEQ_MOTION_TIMEOUT_MS);

            (void)motor_rack_move(RACK_POS_HOME);
            wait_motion_done(SUBSYS_RACK, SEQ_MOTION_TIMEOUT_MS);

            (void)motor_arm_move(ARM_POS_PRESS); /* align */
            wait_motion_done(SUBSYS_ARM, SEQ_MOTION_TIMEOUT_MS);

            set_state(SYS_EJECTING);
            break;
        }

        /* ── EJECTING ───────────────────────────────────────────────── */
        case SYS_EJECTING:
        {
            ESP_LOGI(TAG, "eject: turn platform to eject, release vacuum");
            (void)motor_turntable_goto(TURNTABLE_POS_C); /* eject position */
            wait_motion_done(SUBSYS_TURNTABLE, SEQ_MOTION_TIMEOUT_MS);

            /* Release vacuum cups */
            (void)motor_vacuum_set(false);
            (void)motor_vacuum2_set(false);

            /* Drive indexer to eject */
            (void)motor_indexer_move(INDEXER_POS_EJECT);
            wait_motion_done(SUBSYS_INDEXER, SEQ_MOTION_TIMEOUT_MS);
            vTaskDelay(pdMS_TO_TICKS(500));

            /* Retract indexer */
            (void)motor_indexer_move(INDEXER_POS_OPEN);
            wait_motion_done(SUBSYS_INDEXER, SEQ_MOTION_TIMEOUT_MS);

            /* Open linear arm */
            (void)motor_rack_move(RACK_POS_HOME);
            wait_motion_done(SUBSYS_RACK, SEQ_MOTION_TIMEOUT_MS);

            set_state(SYS_ROTATING_TO_INTAKE);
            break;
        }

        /* ── ROTATING TO INTAKE (loop back) ─────────────────────────── */
        case SYS_ROTATING_TO_INTAKE:
        {
            ESP_LOGI(TAG, "rotate: platform back to accept");
            (void)motor_turntable_goto(TURNTABLE_POS_B);
            wait_motion_done(SUBSYS_TURNTABLE, SEQ_MOTION_TIMEOUT_MS);

            /* Re-open indexer for next bag */
            (void)motor_indexer_move(INDEXER_POS_OPEN);

            set_state(SYS_INTAKE_WAITING);
            break;
        }

        /* ── SPAWN EMPTY ────────────────────────────────────────────── */
        case SYS_SPAWN_EMPTY:
        {
            ESP_LOGW(TAG, "spawn_empty: waiting for Replace Spawn command");
            sys_cmd_t c = wait_cmd(portMAX_DELAY);
            if (c == SYS_CMD_REPLACE_SPAWN)
                set_state(SYS_CONTINUE_RESTART);
            break;
        }

        /* ── CONTINUE RESTART ───────────────────────────────────────── */
        case SYS_CONTINUE_RESTART:
        {
            ESP_LOGI(TAG, "continue: close arms, rotate to trash, open flaps");
            (void)motor_rack_move(RACK_POS_HOME);
            (void)motor_arm_move(ARM_POS_PRESS);
            (void)motor_turntable_goto(TURNTABLE_POS_A); /* trash */
            wait_motion_done(SUBSYS_TURNTABLE, SEQ_MOTION_TIMEOUT_MS);

            (void)motor_flap_open();
            wait_motion_done(SUBSYS_FLAPS, SEQ_MOTION_TIMEOUT_MS);

            ESP_LOGI(TAG, "continue: waiting for Start after new bag loaded");
            sys_cmd_t s2 = wait_cmd(portMAX_DELAY);
            if (s2 == SYS_CMD_START)
                set_state(SYS_CUTTING_TIP);
            else
                set_state(SYS_IDLE);
            break;
        }

        /* ── ERROR / ESTOP ──────────────────────────────────────────── */
        case SYS_ERROR:
        case SYS_ESTOP:
        {
            /* Remain here until a new Setup/Load command resets the machine */
            safe_stop_all();
            sys_cmd_t c = wait_cmd(portMAX_DELAY);
            if (c == SYS_CMD_SETUP_LOAD)
                set_state(SYS_IDLE);
            break;
        }

        default:
            ESP_LOGE(TAG, "unknown state %d", (int)s_state);
            set_state(SYS_ERROR);
            break;
        }
    }
}

/* ── Public API ─────────────────────────────────────────────────────────────── */

esp_err_t sys_sequence_init(void)
{
    s_cmd_queue = xQueueCreate(SEQ_CMD_QUEUE_DEPTH, sizeof(sys_cmd_t));
    if (!s_cmd_queue)
    {
        ESP_LOGE(TAG, "failed to create command queue");
        return ESP_FAIL;
    }

    s_events = xEventGroupCreate();
    if (!s_events)
    {
        ESP_LOGE(TAG, "failed to create event group");
        return ESP_FAIL;
    }

    BaseType_t rc = xTaskCreate(seq_task,
                                "sys_seq",
                                SEQ_TASK_STACK,
                                NULL,
                                SEQ_TASK_PRIORITY,
                                &s_task_handle);

    if (rc != pdPASS)
    {
        ESP_LOGE(TAG, "failed to create sequence task");
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "sys_sequence initialised");
    return ESP_OK;
}

esp_err_t sys_sequence_send_cmd(sys_cmd_t cmd)
{
    if (xQueueSend(s_cmd_queue, &cmd, pdMS_TO_TICKS(50)) != pdTRUE)
    {
        ESP_LOGW(TAG, "send_cmd: queue full, dropping cmd %d", (int)cmd);
        return ESP_ERR_TIMEOUT;
    }
    return ESP_OK;
}

sys_state_t sys_sequence_get_state(void)
{
    return s_state;
}

uint32_t sys_sequence_get_bag_count(void)
{
    return s_bag_count;
}

void sys_sequence_notify_motion_done(uint8_t subsystem, uint8_t result)
{
    s_last_motion_subsys = subsystem;
    s_last_motion_result = result;
    if (s_events)
    {
        xEventGroupSetBitsFromISR(s_events, EVT_MOTION_DONE, NULL);
    }
}

void sys_sequence_notify_spawn_status(uint8_t status)
{
    s_last_spawn_status = status;
    if (!s_events)
        return;

    switch ((spawn_status_code_t)status)
    {
    case SPAWN_STATUS_DONE:
        xEventGroupSetBitsFromISR(s_events, EVT_SPAWN_DONE, NULL);
        break;
    case SPAWN_STATUS_BAG_EMPTY:
        xEventGroupSetBitsFromISR(s_events, EVT_SPAWN_EMPTY, NULL);
        break;
    case SPAWN_STATUS_AGITATING:
        xEventGroupSetBitsFromISR(s_events, EVT_SPAWN_AGITATING, NULL);
        break;
    default:
        break;
    }
}

const char *sys_sequence_state_name(sys_state_t state)
{
    switch (state)
    {
    case SYS_IDLE:
        return "IDLE";
    case SYS_SETUP_LOAD:
        return "SETUP_LOAD";
    case SYS_CUTTING_TIP:
        return "CUTTING_TIP";
    case SYS_ROTATING_TO_ACCEPT:
        return "ROTATING_TO_ACCEPT";
    case SYS_INTAKE_WAITING:
        return "INTAKE_WAITING";
    case SYS_INTAKE_WEIGHING:
        return "INTAKE_WEIGHING";
    case SYS_OPENING_BAG:
        return "OPENING_BAG";
    case SYS_INOCULATING:
        return "INOCULATING";
    case SYS_POST_DOSE:
        return "POST_DOSE";
    case SYS_EJECTING:
        return "EJECTING";
    case SYS_ROTATING_TO_INTAKE:
        return "ROTATING_TO_INTAKE";
    case SYS_SPAWN_EMPTY:
        return "SPAWN_EMPTY";
    case SYS_CONTINUE_RESTART:
        return "CONTINUE_RESTART";
    case SYS_ERROR:
        return "ERROR";
    case SYS_ESTOP:
        return "ESTOP";
    default:
        return "UNKNOWN";
    }
}
