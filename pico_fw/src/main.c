#include "pico/stdlib.h"
#include <stdio.h>
#include "uart_server.h"

int main() {
    stdio_init_all();
    sleep_ms(500);

    uart_server_init();

    while (true) {
        uart_server_poll();
        tight_loop_contents();
    }
}
