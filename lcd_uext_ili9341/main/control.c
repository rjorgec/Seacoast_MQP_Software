#include "control.h"
#include "dosing.h"
#include "loadcell.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"

static const char *TAG = "control";
static QueueHandle_t s_q;

static void control_task(void *arg)
{
    (void)arg;

    // init subsystems
    ESP_ERROR_CHECK(dosing_init());
    // (loadcell_init is already called in dosing_init() in the earlier scaffold)

    while (1) {
        ctrl_cmd_t cmd;
        if (xQueueReceive(s_q, &cmd, portMAX_DELAY) == pdTRUE) {
            switch (cmd.id) {
                case CTRL_CMD_TARE:
                    ESP_LOGI(TAG, "Tare requested");
                    ESP_ERROR_CHECK(loadcell_tare());
                    break;

                case CTRL_CMD_START_DOSE: {
                    ESP_LOGI(TAG, "Start dose target=%.2f g", (double)cmd.target_g);

                    dose_cfg_t cfg = {
                        .target_g = cmd.target_g,
                        .throttle_opening = 0.30f,
                        .settle_band_g = 0.5f,
                        .start_close_margin_g = 3.0f,
                        .latency_s = 0.20f,
                        .max_time_s = 25.0f,
                    };

                    esp_err_t err = dosing_start(&cfg);
                    if (err != ESP_OK) {
                        ESP_LOGW(TAG, "dosing_start failed: %s", esp_err_to_name(err));
                    }
                    break;
                }

                case CTRL_CMD_ABORT:
                    ESP_LOGI(TAG, "Abort requested");
                    dosing_abort();
                    break;

                case CTRL_CMD_CLEAN:
                    ESP_LOGI(TAG, "CLEAN placeholder (not implemented yet)");
                    // later: run a sequence / state machine
                    break;

                case CTRL_CMD_HOME:
                    ESP_LOGI(TAG, "HOME placeholder (not implemented yet)");
                    break;

                default:
                    break;
            }
        }
    }
}

void control_init(void)
{
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
