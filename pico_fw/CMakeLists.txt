cmake_minimum_required(VERSION 3.13)

include(pico_sdk_import.cmake)

project(pico_fw C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

pico_sdk_init()

add_executable(pico_fw
  src/main.c
  src/uart_server.c
  src/stepper8434s.c
  shared/proto_crc.c
  drivers/drv8163/drv8163.c
  drivers/drv8163/drv8163_pico.c
  drivers/drv8434s/drv8434s.c
)

target_include_directories(pico_fw PRIVATE
  src
  shared
  drivers/drv8163
  drivers/drv8434s
)

target_link_libraries(pico_fw
  pico_stdlib
  hardware_uart
  hardware_spi
  hardware_pwm
  hardware_adc
  hardware_irq
  hardware_timer
)

pico_enable_stdio_usb(pico_fw 1)
pico_enable_stdio_uart(pico_fw 0)

pico_add_extra_outputs(pico_fw)
