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

# Automatically adds the PICO SDK as a submodule if not already present
include(pico_sdk_import.cmake)

project(drv8163_pico_discrete LANGUAGES C CXX)

pico_sdk_init()

# Add the main source files for the DRV8163 driver and example
# Use the CMake project name as the target so names are consistent with examples
add_executable(${PROJECT_NAME}
    drv8163.c
    drv8163_pico.c
    main.c
    ../drv8434s/drv8434s.c
    ../../common/src/seacoast_proto.c
)

# Pull in Raspberry Pi Pico SDK libraries
target_link_libraries(${PROJECT_NAME}
    pico_stdlib
    hardware_spi
    hardware_gpio
    hardware_pwm
    hardware_adc
    hardware_uart
)

# Include sibling drv8434s directory so the example can include its header
target_include_directories(${PROJECT_NAME} PRIVATE
    ${CMAKE_SOURCE_DIR}/../drv8434s
    ${CMAKE_SOURCE_DIR}/../../common/include
)

# Enable USB CDC for printf output
pico_enable_stdio_usb(${PROJECT_NAME} 1)

# Generate .uf2 file for easy flashing
pico_add_extra_outputs(${PROJECT_NAME})
