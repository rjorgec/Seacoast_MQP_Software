#pragma once
#include <stdint.h>

void ui_show_home(void);
void ui_show_dosing(void);
void ui_show_stepper(void);
void ui_status_set(const char *s);
void ui_screens_pico_rx_handler(uint8_t msg_type, uint16_t seq, const uint8_t *payload, uint16_t len);
