#pragma once

// UART between ESP <-> Pico
#ifndef LINK_UART_ID
#define LINK_UART_ID uart0
#endif
#ifndef LINK_UART_BAUD
#define LINK_UART_BAUD 115200
#endif
#ifndef LINK_UART_TX_PIN
#define LINK_UART_TX_PIN 0
#endif
#ifndef LINK_UART_RX_PIN
#define LINK_UART_RX_PIN 1
#endif

// DRV8434S SPI + control pins (defaults = old demo)
#ifndef DRV8434S_SPI_PORT
#define DRV8434S_SPI_PORT spi0
#endif
#ifndef DRV8434S_PIN_SCK
#define DRV8434S_PIN_SCK 18
#endif
#ifndef DRV8434S_PIN_MOSI
#define DRV8434S_PIN_MOSI 19
#endif
#ifndef DRV8434S_PIN_MISO
#define DRV8434S_PIN_MISO 16
#endif
#ifndef DRV8434S_PIN_CS
#define DRV8434S_PIN_CS 13
#endif
#ifndef DRV8434S_PIN_RST
#define DRV8434S_PIN_RST 12
#endif
#ifndef DRV8434S_PIN_NFAULT
#define DRV8434S_PIN_NFAULT 11
#endif
