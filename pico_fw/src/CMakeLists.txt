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

add_executable(${PROJECT_NAME} main.c uart_server.c ../drivers/drv8163/drv8163.c ../../shared/proto/cobs.c ../../shared/proto/proto_crc.c)

get_filename_component(BASE_DIR "${CMAKE_CURRENT_SOURCE_DIR}" DIRECTORY)

# Get the project root directory (parent of pico_fw)
get_filename_component(PROJECT_ROOT "${BASE_DIR}" DIRECTORY)

# Set up include directories for source dir and shared dir
set(EXTERNAL_HEADERS_DIR "${CMAKE_CURRENT_SOURCE_DIR}")

# Apply to target
target_include_directories(${PROJECT_NAME} PRIVATE
    "${EXTERNAL_HEADERS_DIR}"
    "${BASE_DIR}"
    "${PROJECT_ROOT}"
)

# Add pico-scale library from extern directory
add_subdirectory(../../extern/pico-scale ${CMAKE_BINARY_DIR}/pico-scale)

target_link_libraries(${PROJECT_NAME} pico_stdlib pico_stdio hardware_gpio hardware_pwm hardware_adc hardware_uart hardware_pio pico-scale)

# Enable USB CDC for printf output, disable UART stdio to avoid conflict with uart_server
pico_enable_stdio_usb(${PROJECT_NAME} 1)
pico_enable_stdio_uart(${PROJECT_NAME} 0)

# Generate .uf2 file for easy flashing
pico_add_extra_outputs(${PROJECT_NAME})
