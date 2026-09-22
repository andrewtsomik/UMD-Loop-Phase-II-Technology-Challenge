cmake_minimum_required(VERSION 3.20.0)

# Directories that CONTAIN dts/bindings. Must come before find_package.
list(APPEND DTS_ROOT
    ${CMAKE_CURRENT_SOURCE_DIR}/modules/servo
    ${CMAKE_CURRENT_SOURCE_DIR}/modules/hex_led
    ${CMAKE_CURRENT_SOURCE_DIR}/modules/tone
    ${CMAKE_CURRENT_SOURCE_DIR}/modules/receiver
)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(system1)

target_sources(app PRIVATE
    src/main.c
    modules/servo/src/servo.c
    modules/hex_led/src/hex_led.c
    modules/tone/src/tone.c
    modules/receiver/src/receiver.c
)

target_include_directories(app PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/modules/servo/include
    ${CMAKE_CURRENT_SOURCE_DIR}/modules/hex_led/include
    ${CMAKE_CURRENT_SOURCE_DIR}/modules/tone/include
    ${CMAKE_CURRENT_SOURCE_DIR}/modules/receiver/include
)
