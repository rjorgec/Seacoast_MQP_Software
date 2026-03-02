#pragma once
#include <stdint.h>
#include "proto/proto.h"

void ui_show_home(void);
void ui_show_dosing(void);
void ui_show_operations(void);
void ui_status_set(const char *s);
void ui_screens_pico_rx_handler(uint8_t msg_type, uint16_t seq, const uint8_t *payload, uint16_t len);

/** Call from pico_rx_cb when MSG_MOTION_DONE is received */
void ui_ops_on_motion_done(const pl_motion_done_t *pl);

/** Call from pico_rx_cb when MSG_VACUUM_STATUS is received */
void ui_ops_on_vacuum_status(const pl_vacuum_status_t *pl);

/** Call from pico_rx_cb when MSG_SPAWN_STATUS is received */
void ui_dosing_on_spawn_status(const pl_spawn_status_t *pl);
