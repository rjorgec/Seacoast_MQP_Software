#pragma once
#include <stdbool.h>
#include <stdint.h>
#define TUNING

void uart_server_init(void);
void uart_server_poll(void); // call frequently in main loop
