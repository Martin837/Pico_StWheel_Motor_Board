cmake_minimum_required(VERSION 3.12)
include(pico_sdk_import.cmake)
include(pico_extras_import_optional.cmake)

project(PicoProject C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

pico_sdk_init()


add_compile_options(-Wall
    -Wno-format          
    -Wno-unused-function 
    -Wno-maybe-uninitialized
)

add_executable(PicoProject
    main.c
    martinlib.c
    i2cslave.c
)

target_link_libraries(PicoProject pico_stdlib cmsis_core hardware_pwm hardware_i2c hardware_adc pico_i2c_slave)

pico_enable_stdio_usb(PicoProject 1)
pico_enable_stdio_uart(PicoProject 0)

pico_add_extra_outputs(PicoProject)
