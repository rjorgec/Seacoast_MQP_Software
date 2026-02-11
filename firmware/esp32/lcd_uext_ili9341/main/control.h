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

typedef struct {
    bool pico_online;
    uint8_t pico_state;
    float mass_g;
    uint16_t fault;
    uint8_t last_ack_cmd;
    int8_t last_ack_result;
} ctrl_status_t;

void control_init(void);
bool control_send(const ctrl_cmd_t *cmd);   //false if queue full
ctrl_status_t control_get_status(void);
