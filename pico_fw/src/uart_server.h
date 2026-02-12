#pragma once
#include <stdint.h>
#include <stdbool.h>

void uart_server_init(void);
void uart_server_poll(void);   // call frequently in main loop
