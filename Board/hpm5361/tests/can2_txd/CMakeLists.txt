cmake_minimum_required(VERSION 3.20.0)

set(ZEPHYR_EXTRA_MODULES "D:/MCU/hpm/zephyr_sdk_glue")
set(HPM_SDK_DIR "D:/MCU/hpm/hpm_sdk" CACHE PATH "")
get_filename_component(HPM5361_BOARD_DIR "${CMAKE_CURRENT_LIST_DIR}/../.." ABSOLUTE)
set(BOARD_DIR "${HPM5361_BOARD_DIR}" CACHE PATH "" FORCE)
set(BOARD_DIRECTORIES "${HPM5361_BOARD_DIR}" CACHE INTERNAL "Board directories" FORCE)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(can2_txd_probe)

target_sources(app PRIVATE src/main.c)
