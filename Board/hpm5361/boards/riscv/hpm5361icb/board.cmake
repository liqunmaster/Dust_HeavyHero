get_filename_component(HPM_OPENOCD_ROOT "${HPM_SDK_DIR}/../tools/openocd" ABSOLUTE)
get_filename_component(HPM_OPENOCD_CONFIG
  "${CMAKE_CURRENT_LIST_DIR}/support/openocd.cfg" ABSOLUTE)
board_runner_args(openocd
  "--openocd=${HPM_OPENOCD_ROOT}/openocd.exe"
  "--openocd-search=${HPM_OPENOCD_ROOT}/tcl"
  "--config=${HPM_OPENOCD_CONFIG}"
  "--cmd-pre-init=set HPM_SDK_DIR {${HPM_SDK_DIR}}"
  "--cmd-reset-halt=hpm_prepare_flash"
  "--verify")

include(${ZEPHYR_BASE}/boards/common/openocd.board.cmake)
