cmake_minimum_required(VERSION 3.20.0)

set(ZEPHYR_EXTRA_MODULES "D:/MCU/hpm/zephyr_sdk_glue")

set(HPM_SDK_DIR "D:/MCU/hpm/hpm_sdk" CACHE PATH "")

list(APPEND BOARD_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/Board/hpm5361)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})

project(Dust_HeavyHero)

target_sources(app PRIVATE App/core/main.cpp)
add_subdirectory(Algorithm)
add_subdirectory(Bsp)
add_subdirectory(Device/motor/dji/c620)
zephyr_linker_sources(SECTIONS Bsp/mcan_ram.ld)
