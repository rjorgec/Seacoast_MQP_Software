# == DO NOT EDIT THE FOLLOWING LINES for the Raspberry Pi Pico VS Code Extension to work ==
if(WIN32)
    set(USERHOME $ENV{USERPROFILE})
else()
    set(USERHOME $ENV{HOME})
endif()
set(sdkVersion 2.2.0)
set(toolchainVersion 14_2_Rel1)
set(picotoolVersion 2.2.0-a4)
set(picoVscode ${USERHOME}/.pico-sdk/cmake/pico-vscode.cmake)
if (EXISTS ${picoVscode})
    include(${picoVscode})
endif()
# ====================================================================================
set(PICO_BOARD pico2 CACHE STRING "Board type")

cmake_minimum_required(VERSION 3.12)

include(pico_sdk_import.cmake)

project(UART_TEST LANGUAGES C CXX)

pico_sdk_init()

add_executable(${PROJECT_NAME} main.c uart_server.c ../drivers/drv8163/drv8163.c ../shared/cobs.c ../shared/proto_crc.c)

# 1. Move up from 'src' to 'Project_Root'
get_filename_component(BASE_DIR "${CMAKE_CURRENT_SOURCE_DIR}" DIRECTORY)

# 2. Reference 'drivers' inside 'Project_Root'
set(EXTERNAL_HEADERS_DIR "${BASE_DIR}")

# 3. Apply to target
target_include_directories(${PROJECT_NAME} PRIVATE "${EXTERNAL_HEADERS_DIR}")

target_link_libraries(${PROJECT_NAME} pico_stdlib hardware_gpio hardware_pwm hardware_adc hardware_uart)

# Enable USB CDC for printf output
pico_enable_stdio_usb(${PROJECT_NAME} 1)

# Generate .uf2 file for easy flashing
pico_add_extra_outputs(${PROJECT_NAME})
