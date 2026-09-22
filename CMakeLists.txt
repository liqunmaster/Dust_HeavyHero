cmake_minimum_required(VERSION 3.20.0)

set(ZEPHYR_EXTRA_MODULES "D:/MCU/hpm/zephyr_sdk_glue")

set(HPM_SDK_DIR "D:/MCU/hpm/hpm_sdk" CACHE PATH "")

set(CONF_FILE
    ${CMAKE_CURRENT_SOURCE_DIR}/prj.conf
    ${CMAKE_CURRENT_SOURCE_DIR}/Bsp/bsp_fdcan/fdcan.conf
    ${CMAKE_CURRENT_SOURCE_DIR}/Bsp/bsp_spi/spi.conf
    ${CMAKE_CURRENT_SOURCE_DIR}/Bsp/bsp_uart/uart.conf
    ${CMAKE_CURRENT_SOURCE_DIR}/Topic/topic.conf
    ${CMAKE_CURRENT_SOURCE_DIR}/Device/remote/remote.conf
)
set(DTC_OVERLAY_FILE ${CMAKE_CURRENT_SOURCE_DIR}/Device/remote/remote.overlay)

list(APPEND BOARD_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/Board/hpm5361)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})

project(Dust_HeavyHero)

add_subdirectory(Algorithm)
add_subdirectory(App)
add_subdirectory(Bsp)
add_subdirectory(Device)
add_subdirectory(Module)
add_subdirectory(Topic)
