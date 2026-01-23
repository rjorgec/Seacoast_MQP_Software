#pragma once
#include <stdbool.h>
#include "esp_err.h"

typedef enum {
    CTRL_CMD_START,
    CTRL_CMD_PAUSE,
    CTRL_CMD_STOP,
    CTRL_CMD_CLEAN,
    CTRL_CMD_TARE,
} ctrl_cmd_type_t;

typedef struct {
    ctrl_cmd_type_t type;
    float target_g;     // used by START
    int recipe_id;      // used by START
} ctrl_cmd_t;

esp_err_t control_start(void);
bool control_send(const ctrl_cmd_t *cmd);
