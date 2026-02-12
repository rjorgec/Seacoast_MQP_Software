#pragma once
#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"

typedef enum { MOTOR_DIR_FWD=0, MOTOR_DIR_REV=1 } motor_dir_t;

// DRV8163: your “current monitor task”
esp_err_t motor_linact_start_monitor_dir(motor_dir_t dir,
                                        uint16_t speed,
                                        uint16_t low_th,
                                        uint16_t high_th,
                                        uint32_t interval_ms);

esp_err_t motor_linact_stop_monitor(void);
esp_err_t motor_linact_run(motor_dir_t dir, uint16_t speed);
esp_err_t motor_linact_stop(void);


// DRV8434S: simple stepping API (upgrade later to queued motion)
esp_err_t motor_stepper_enable(bool en);
esp_err_t motor_stepper_step(motor_dir_t dir, uint32_t steps, uint32_t step_delay_us);

