// control.c (ESP32 side) — updated to forward commands over UART to the Pico
#include "control.h"

#include "uart_link.h"
#include "seacoast_proto.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "esp_err.h"
#include "esp_log.h"

static const char *TAG = "control";
static QueueHandle_t s_q;
static portMUX_TYPE s_status_lock = portMUX_INITIALIZER_UNLOCKED;
static ctrl_status_t s_status;
static TickType_t s_last_rx_tick;

static void control_on_frame(const sc_frame_view_t *f)
{
    if (!f) return;

    portENTER_CRITICAL(&s_status_lock);
    s_status.pico_online = true;
    s_last_rx_tick = xTaskGetTickCount();
    portEXIT_CRITICAL(&s_status_lock);

    if (f->type == SC_MSG_ACK && f->len == sizeof(sc_ack_t)) {
        const sc_ack_t *ack = (const sc_ack_t *)f->payload;
        portENTER_CRITICAL(&s_status_lock);
        s_status.last_ack_cmd = ack->cmd_type;
        s_status.last_ack_result = ack->result;
        portEXIT_CRITICAL(&s_status_lock);
    } else if (f->type == SC_MSG_STATUS && f->len == sizeof(sc_status_t)) {
        const sc_status_t *st = (const sc_status_t *)f->payload;
        portENTER_CRITICAL(&s_status_lock);
        s_status.pico_state = st->state;
        s_status.mass_g = st->mass_g;
        s_status.fault = st->fault;
        portEXIT_CRITICAL(&s_status_lock);
    }
}

static void control_task(void *arg)
{
    (void)arg;

    // Bring up the UART bridge (ESP -> Pico)
    ESP_ERROR_CHECK(uart_link_init());

    while (1) {
        uart_link_rx_pump(control_on_frame);

        TickType_t now = xTaskGetTickCount();
        if ((now - s_last_rx_tick) > pdMS_TO_TICKS(2000)) {
            portENTER_CRITICAL(&s_status_lock);
            s_status.pico_online = false;
            portEXIT_CRITICAL(&s_status_lock);
        }

        ctrl_cmd_t cmd;
        if (xQueueReceive(s_q, &cmd, pdMS_TO_TICKS(20)) != pdTRUE) {
            continue;
        }

        switch (cmd.id) {
            case CTRL_CMD_TARE:
                ESP_LOGI(TAG, "Tare requested -> UART");
                ESP_ERROR_CHECK(uart_link_send(SC_MSG_CMD_TARE, NULL, 0));
                break;

            case CTRL_CMD_START_DOSE: {
                ESP_LOGI(TAG, "Start dose target=%.2f g -> UART", (double)cmd.target_g);

                // current ctrl_cmd_t doesn't carry recipe_id; set to 0 for now.
                sc_cmd_start_t p = {
                    .target_g  = cmd.target_g,
                    .recipe_id = 0,
                };

                ESP_ERROR_CHECK(uart_link_send(SC_MSG_CMD_START, &p, (uint8_t)sizeof(p)));
                break;
            }

            case CTRL_CMD_ABORT:
                ESP_LOGI(TAG, "Abort requested -> UART (STOP)");
                ESP_ERROR_CHECK(uart_link_send(SC_MSG_CMD_STOP, NULL, 0));
                break;

            case CTRL_CMD_CLEAN:
                ESP_LOGI(TAG, "CLEAN requested -> UART");
                ESP_ERROR_CHECK(uart_link_send(SC_MSG_CMD_CLEAN, NULL, 0));
                break;

            case CTRL_CMD_HOME:
                ESP_LOGI(TAG, "HOME requested -> UART");
                ESP_ERROR_CHECK(uart_link_send(SC_MSG_CMD_HOME, NULL, 0));
                break;

            default:
                ESP_LOGW(TAG, "Unknown cmd id=%d", (int)cmd.id);
                break;
        }
    }
}

void control_init(void)
{
    if (s_q) return;

    s_status = (ctrl_status_t){0};
    s_status.last_ack_result = -127;
    s_last_rx_tick = xTaskGetTickCount();

    s_q = xQueueCreate(8, sizeof(ctrl_cmd_t));
    configASSERT(s_q);

    BaseType_t ok = xTaskCreate(control_task, "control_task", 4096, NULL, 10, NULL);
    configASSERT(ok == pdTRUE);
}

bool control_send(const ctrl_cmd_t *cmd)
{
    if (!s_q || !cmd) return false;
    return xQueueSend(s_q, cmd, 0) == pdTRUE;
}

ctrl_status_t control_get_status(void)
{
    ctrl_status_t out;
    portENTER_CRITICAL(&s_status_lock);
    out = s_status;
    portEXIT_CRITICAL(&s_status_lock);
    return out;
}
