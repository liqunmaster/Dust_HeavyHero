# Copyright 2026 HPMicro
# SPDX-License-Identifier: Apache-2.0

set(OPENOCD_CONFIG_RELATIVE ${ZEPHYR_BASE}/../sdk_env/hpm_sdk/boards/openocd)
get_filename_component(OPENOCD_CONFIG_ABSOLUTE ${OPENOCD_CONFIG_RELATIVE} ABSOLUTE)
set(OPENOCD_CONFIG_DIR ${OPENOCD_CONFIG_ABSOLUTE} CACHE PATH "hpmicro openocd cfg root directory")

if(${BOARD} STREQUAL "hpm5361icb")
	board_runner_args(openocd "--config=${OPENOCD_CONFIG_DIR}/soc/hpm5361-single-core.cfg"
	                            "--config=${OPENOCD_CONFIG_DIR}/boards/hpm5361evk.cfg"
	                            "--openocd-search=${OPENOCD_CONFIG_DIR}")
else()
	message(FATAL_ERROR "${BOARD} is not supported now")
endif()

include(${ZEPHYR_BASE}/boards/common/openocd.board.cmake)