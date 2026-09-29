cmake_minimum_required(VERSION 3.20.0)

set(ZEPHYR_EXTRA_MODULES "D:/MCU/hpm/zephyr_sdk_glue")

set(HPM_SDK_DIR "D:/MCU/hpm/hpm_sdk" CACHE PATH "")

set(CONF_FILE
    ${CMAKE_CURRENT_SOURCE_DIR}/prj.conf
    ${CMAKE_CURRENT_SOURCE_DIR}/Bsp/bsp_fdcan/fdcan.conf
    ${CMAKE_CURRENT_SOURCE_DIR}/Bsp/bsp_gpio/gpio.conf
    ${CMAKE_CURRENT_SOURCE_DIR}/Bsp/bsp_pwm/pwm.conf
    ${CMAKE_CURRENT_SOURCE_DIR}/Bsp/bsp_spi/spi.conf
    ${CMAKE_CURRENT_SOURCE_DIR}/Bsp/bsp_uart/uart.conf
    ${CMAKE_CURRENT_SOURCE_DIR}/Topic/topic.conf
    ${CMAKE_CURRENT_SOURCE_DIR}/Device/remote/remote.conf
    ${CMAKE_CURRENT_SOURCE_DIR}/Device/imu/imu.conf
    CACHE STRING "Application configuration files" FORCE
)
set(DTC_OVERLAY_FILE
    ${CMAKE_CURRENT_SOURCE_DIR}/Bsp/bsp_fdcan/fdcan.overlay
    ${CMAKE_CURRENT_SOURCE_DIR}/Bsp/bsp_pwm/pwm.overlay
    ${CMAKE_CURRENT_SOURCE_DIR}/Bsp/bsp_uart/uart.overlay
    ${CMAKE_CURRENT_SOURCE_DIR}/Device/remote/remote.overlay
    ${CMAKE_CURRENT_SOURCE_DIR}/Device/imu/imu.overlay
    ${CMAKE_CURRENT_SOURCE_DIR}/Device/motor/dji/c610/c610.overlay
    ${CMAKE_CURRENT_SOURCE_DIR}/Device/motor/dji/c620/c620.overlay
    ${CMAKE_CURRENT_SOURCE_DIR}/Device/motor/dm/dm.overlay
    CACHE STRING "Application devicetree overlays" FORCE
)

set(BOARD_DIR ${CMAKE_CURRENT_SOURCE_DIR}/Board/hpm5361 CACHE PATH "" FORCE)
set(BOARD_DIRECTORIES ${CMAKE_CURRENT_SOURCE_DIR}/Board/hpm5361
    CACHE INTERNAL "Board directories" FORCE)

set(CMAKE_WARN_DEPRECATED OFF CACHE BOOL "Suppress deprecated CMake warnings" FORCE)

list(APPEND DTS_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/Board/hpm5361)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})

project(Dust_HeavyHero)

add_subdirectory(Algorithm)
add_subdirectory(App)
add_subdirectory(Bsp)
add_subdirectory(Device)
add_subdirectory(Module)
add_subdirectory(Topic)
add_subdirectory(Communication)

set_source_files_properties(
    "D:/MCU/hpm/zephyr_sdk_glue/drivers/serial/uart_hpmicro.c"
    PROPERTIES COMPILE_OPTIONS "-Wno-unused-function"
)
