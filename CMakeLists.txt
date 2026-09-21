cmake_minimum_required(VERSION 3.20.0)

set(ZEPHYR_EXTRA_MODULES "D:/MCU/hpm/zephyr_sdk_glue")

set(HPM_SDK_DIR "D:/MCU/hpm/hpm_sdk" CACHE PATH "")

list(APPEND BOARD_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/Board/hpm5361)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})

project(Dust_HeavyHero)

target_sources(app PRIVATE App/core/main.cpp)