#include "control.h"

#include "dosing.h"
#include "loadcell.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "esp_log.h"
#include "esp_err.h"

static const char *TAG = "control";
static QueueHandle_t s_q;

//persistent dosing context
static dosing_ctx_t s_dose;

static void handle_cmd(const ctrl_cmd_t *cmd)
{
    switch (cmd->type) {
        case CTRL_CMD_TARE:
            ESP_LOGI(TAG, "Tare");
            ESP_ERROR_CHECK(loadcell_tare());
            break;

        case CTRL_CMD_START:
            ESP_LOGI(TAG, "Start target=%.2fg recipe=%u",
                     (double)cmd->target_g, (unsigned)cmd->recipe_id);

                    //recipe ignored for now
            {
                esp_err_t err = dosing_start(&s_dose, cmd->target_g);
                if (err != ESP_OK) {
                    ESP_LOGW(TAG, "dosing_start failed: %s", esp_err_to_name(err));
                }
            }
            break;

        case CTRL_CMD_STOP:
            ESP_LOGI(TAG, "Stop/Abort");
            ESP_ERROR_CHECK(dosing_abort(&s_dose));
            break;

        case CTRL_CMD_PAUSE:
            ESP_LOGI(TAG, "Pause (TODO) -> for now, abort dosing");
            ESP_ERROR_CHECK(dosing_abort(&s_dose));
            break;

        case CTRL_CMD_CLEAN:
            ESP_LOGI(TAG, "Clean (TODO)");
            break;

        case CTRL_CMD_HOME:
            ESP_LOGI(TAG, "Home (TODO)");
            break;

        default:
            break;
    }
}

static void control_task(void *arg)
{
    (void)arg;


    ESP_ERROR_CHECK(dosing_init(&s_dose));
    //must tick!

    const TickType_t tick_period = pdMS_TO_TICKS(20);

    while (1) {
        //dosing logic (non-blocking)
        (void)dosing_tick(&s_dose);

        //poll for commands
        ctrl_cmd_t cmd;
        if (xQueueReceive(s_q, &cmd, tick_period) == pdTRUE) {
            handle_cmd(&cmd);
        }
    }
}

esp_err_t control_start(void)
{
    if (s_q) return ESP_OK;

    s_q = xQueueCreate(8, sizeof(ctrl_cmd_t));
    if (!s_q) return ESP_ERR_NO_MEM;

    BaseType_t ok = xTaskCreate(control_task, "control_task", 4096, NULL, 10, NULL);
    if (ok != pdTRUE) return ESP_FAIL;

    return ESP_OK;
}

bool control_send(const ctrl_cmd_t *cmd)
{
    if (!s_q || !cmd) return false;
    return xQueueSend(s_q, cmd, 0) == pdTRUE;
}
