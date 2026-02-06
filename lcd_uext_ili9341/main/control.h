#pragma once
#include <stdint.h>
#include <stdbool.h>

typedef enum {
    CTRL_CMD_NONE = 0,
    CTRL_CMD_TARE,
    CTRL_CMD_START_DOSE,
    CTRL_CMD_ABORT,
    CTRL_CMD_CLEAN,     //placeholder
    CTRL_CMD_HOME,
} ctrl_cmd_id_t;

typedef struct {
    ctrl_cmd_id_t id;
    float target_g;     //used for start_dose
} ctrl_cmd_t;

void control_init(void);
bool control_send(const ctrl_cmd_t *cmd);   //false if queue full
