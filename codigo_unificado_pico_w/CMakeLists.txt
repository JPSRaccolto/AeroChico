# Generated Cmake Pico project file

cmake_minimum_required(VERSION 3.13)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Initialise pico_sdk from installed location
# (note this can come from environment, CMake cache etc)


set(PICO_BOARD pico_w CACHE STRING "Board type")

# Pull in Raspberry Pi Pico SDK (must be before project)
include(pico_sdk_import.cmake)

project(codigo_unificado_pico_w C CXX ASM)

# Initialise the Raspberry Pi Pico SDK
pico_sdk_init()

# Add executable. Default name is the project name, version 0.1

add_executable(codigo_unificado_pico_w codigo_unificado_pico_w.c lib/bme680.c lib/mpu6500.c lib/GPS_neo_6.c lib/bme680_custom.c)

pico_set_program_name(codigo_unificado_pico_w "codigo_unificado_pico_w")
pico_set_program_version(codigo_unificado_pico_w "0.1")

# Modify the below lines to enable/disable output over UART/USB
pico_enable_stdio_uart(codigo_unificado_pico_w 1)
pico_enable_stdio_usb(codigo_unificado_pico_w 1)

# Add the standard library to the build
target_link_libraries(codigo_unificado_pico_w
        pico_stdlib
        hardware_i2c
        hardware_uart)

# Add the standard include files to the build
target_include_directories(codigo_unificado_pico_w PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}/lib
)


pico_add_extra_outputs(codigo_unificado_pico_w)

