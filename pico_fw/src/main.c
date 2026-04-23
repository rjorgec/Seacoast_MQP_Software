#include "pico/stdlib.h"
#include <stdio.h>

#include "uart_server.h"

/* USB boot delay — held long enough for `picotool`/host USB-CDC enumeration
 * to open the port and capture early printf output. Previously 10 s; reduced
 * now that the ESP handshake (PING RPC, 30 s window) removes any need for the
 * Pico to stall on a fixed delay before the relay closes. */
#define PICO_USB_BOOT_DELAY_MS 2000u

int main(void)
{
    /* Phase 1 of the system startup (see docs/specs/02-pico-firmware).
     * Pin modes must be set before 12 V is applied to the motor drivers;
    * in particular HOTWIRE_PIN_IN2 has to be LOW so pump 2 stays off when
    * the ESP closes the relay. This runs
     * before stdio_init_all() and before any boot delay on purpose. */
    uart_server_phase1_pinmodes();

    stdio_init_all();
    sleep_ms(PICO_USB_BOOT_DELAY_MS);

    printf("Pico FW boot\n");

    /* Phase 1 communication bringup: UART + ring buffer + PING handler.
     * Motor drivers, HX711, and the DRV8434S SPI chain are deferred to
     * Phase 3, which uart_server_poll() runs after the first PING ACK. */
    uart_server_init_early();

    while (true)
    {
        uart_server_poll();
        sleep_ms(5);
    }
}
