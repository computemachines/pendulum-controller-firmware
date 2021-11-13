cmake_minimum_required(VERSION 3.12)

include($ENV{PICO_SDK_PATH}/pico_sdk_init.cmake)
project(pendulum-controller-pico C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

pico_sdk_init()

add_executable(pendulum-controller)

target_sources(pendulum-controller PRIVATE src/main.c)

target_link_libraries(pendulum-controller pico_stdlib pico_time hardware_gpio hardware_pwm hardware_irq)

pico_add_extra_outputs(pendulum-controller)
pico_enable_stdio_usb(pendulum-controller 0)
pico_enable_stdio_uart(pendulum-controller 1)