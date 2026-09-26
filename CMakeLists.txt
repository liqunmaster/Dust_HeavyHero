cmake_minimum_required(VERSION 3.20.0)

set(ZEPHYR_EXTRA_MODULES "D:/MCU/hpm/zephyr_sdk_glue")

set(HPM_SDK_DIR "D:/MCU/hpm/hpm_sdk" CACHE PATH "")

set(CONF_FILE
    ${CMAKE_CURRENT_SOURCE_DIR}/prj.conf
    ${CMAKE_CURRENT_SOURCE_DIR}/Bsp/bsp_fdcan/fdcan.conf
    ${CMAKE_CURRENT_SOURCE_DIR}/Bsp/bsp_uart/uart.conf
    ${CMAKE_CURRENT_SOURCE_DIR}/Topic/topic.conf
    ${CMAKE_CURRENT_SOURCE_DIR}/Device/remote/remote.conf
)
set(DTC_OVERLAY_FILE
    ${CMAKE_CURRENT_SOURCE_DIR}/Bsp/bsp_fdcan/fdcan.overlay
    ${CMAKE_CURRENT_SOURCE_DIR}/Device/remote/remote.overlay
    ${CMAKE_CURRENT_SOURCE_DIR}/Device/motor/dji/c620/c620.overlay
)

set(BOARD_DIR ${CMAKE_CURRENT_SOURCE_DIR}/Board/hpm5361 CACHE PATH "" FORCE)
set(BOARD_DIRECTORIES ${CMAKE_CURRENT_SOURCE_DIR}/Board/hpm5361
    CACHE INTERNAL "Board directories" FORCE)

# 将 Board/hpm5361 加入 DTS_ROOT，使 Board/hpm5361/dts/bindings 下的
# rp,dji-motor.yaml 等自定义 binding 进入 devicetree 搜索路径
list(APPEND DTS_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/Board/hpm5361)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})

project(Dust_HeavyHero)

add_subdirectory(Algorithm)
add_subdirectory(App)
add_subdirectory(Bsp)
add_subdirectory(Device)
add_subdirectory(Module)
add_subdirectory(Topic)
