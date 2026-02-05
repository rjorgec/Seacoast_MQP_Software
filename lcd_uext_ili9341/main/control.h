#pragma once
#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"

typedef enum {
    CTRL_CMD_NONE = 0,

    CTRL_CMD_START,   //(target_g, recipe_id)
    CTRL_CMD_PAUSE,
    CTRL_CMD_STOP,

    CTRL_CMD_TARE,
    CTRL_CMD_CLEAN,
    CTRL_CMD_HOME,
} ctrl_cmd_type_t;

typedef struct {
    ctrl_cmd_type_t type;
    float target_g;
    uint16_t recipe_id;
} ctrl_cmd_t;

esp_err_t control_start(void);        //create queue + task
bool control_send(const ctrl_cmd_t *cmd);
