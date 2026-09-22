@echo off
setlocal

set "WEST=D:\MCU\zephyr\zephyrproject\.venv\Scripts\west.exe"
set "HPM_WORKSPACE=%~dp0.."
set "HPM_ELF=%HPM_WORKSPACE%\build\zephyr\Dust_HeavyHero.elf"
set "HPM_OPENOCD=D:\MCU\hpm\tools\openocd\openocd.exe"
set "HPM_OPENOCD_TCL=D:\MCU\hpm\hpm_sdk\boards\openocd"

echo [1/2] Incremental build Zephyr hpm5361icb for on-chip flash ...
pushd "D:\MCU\zephyr\zephyrproject"
"%WEST%" build -b hpm5361icb -s "%HPM_WORKSPACE%" -d "%HPM_WORKSPACE%\build"
set "BUILD_RESULT=%ERRORLEVEL%"
popd
if not "%BUILD_RESULT%"=="0" exit /b %BUILD_RESULT%

if not exist "%HPM_ELF%" (
    echo ERROR: ELF not found: %HPM_ELF%
    exit /b 1
)

echo [2/2] Use HPMicro official programmer for HPM5361 on-chip flash ...
"%HPM_OPENOCD%" ^
    -s "%HPM_OPENOCD_TCL%" ^
    -f "%HPM_OPENOCD_TCL%\probes\cmsis_dap.cfg" ^
    -f "%HPM_OPENOCD_TCL%\soc\hpm5300.cfg" ^
    -f "%HPM_OPENOCD_TCL%\boards\hpm5300evk.cfg" ^
    -c "adapter speed 500" ^
    -c "program {%HPM_ELF%} verify" ^
    -c "reset run" ^
    -c "shutdown"
if errorlevel 1 exit /b 1

echo DONE. HPMicro official programmer verified the firmware in on-chip flash.
exit /b 0
