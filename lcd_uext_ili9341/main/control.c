#include "control.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"

static const char *TAG = "control";
static QueueHandle_t s_q;

static void control_task(void *arg)
{
    (void)arg;
    ctrl_cmd_t cmd;

    ESP_LOGI(TAG, "Control task started");

    while (1) {
        if (xQueueReceive(s_q, &cmd, portMAX_DELAY) == pdTRUE) {
            switch (cmd.type) {
                case CTRL_CMD_START:
                    ESP_LOGI(TAG, "START recipe=%d target=%.1fg", cmd.recipe_id, (double)cmd.target_g);
                    // TODO: run state machine, actuators, sensors
                    break;
                case CTRL_CMD_PAUSE:
                    ESP_LOGI(TAG, "PAUSE");
                    break;
                case CTRL_CMD_STOP:
                    ESP_LOGI(TAG, "STOP");
                    break;
                case CTRL_CMD_CLEAN:
                    ESP_LOGI(TAG, "CLEAN");
                    break;
                case CTRL_CMD_TARE:
                    ESP_LOGI(TAG, "TARE");
                    break;
                default:
                    break;
            }
        }
    }
}

esp_err_t control_start(void)
{
    s_q = xQueueCreate(8, sizeof(ctrl_cmd_t));
    if (!s_q) return ESP_ERR_NO_MEM;

    xTaskCreate(control_task, "control_task", 4096, NULL, 8, NULL);
    return ESP_OK;
}

bool control_send(const ctrl_cmd_t *cmd)
{
    if (!s_q || !cmd) return false;
    return xQueueSend(s_q, cmd, 0) == pdTRUE;
}
