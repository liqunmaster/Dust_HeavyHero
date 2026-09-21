# Copyright (c) 2026 HPMicro
# SPDX-License-Identifier: BSD-3-Clause

cmake_minimum_required(VERSION 3.13)

find_package(hpm-sdk REQUIRED HINTS $ENV{HPM_SDK_BASE})

project(dust_hero)

sdk_app_src(App/Core/main.c)
sdk_app_inc(App/Core)

generate_ide_projects()