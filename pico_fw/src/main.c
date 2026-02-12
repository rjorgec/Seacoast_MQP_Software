#include "pico/stdlib.h"
#include <stdio.h>

#include "uart_server.h"

int main(void)
{
    stdio_init_all();
    sleep_ms(10000);

    printf("Pico FW boot\n");

    uart_server_init();

    while (true)
    {
        uart_server_poll();
        sleep_ms(5);
    }
}
