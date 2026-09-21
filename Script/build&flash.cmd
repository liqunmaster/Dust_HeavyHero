@echo off
rem ============================================================
rem  Dust_Hero - HPM5361 构建 & 烧录脚本
rem  使用 SDK 内置 hpm5300evk 板(SOC=HPM5361) + flash_xip
rem ============================================================
setlocal

set SDK=C:\Users\63902\Desktop\GAME\project\Dust_Hero
set HPM_SDK_BASE=D:\MCU\hpm\hpm_sdk
set GNURISCV_TOOLCHAIN_PATH=D:\MCU\hpm\toolchains\rv32imac_zicsr_zifencei_multilib_b_ext-win
set PATH=%CD%\build\tools\cmake\bin;D:\MCU\hpm\tools\cmake\bin;D:\MCU\hpm\tools\ninja;D:\MCU\hpm\tools\openocd;%GNURISCV_TOOLCHAIN_PATH%\bin;%PATH%

cd /d %SDK%

echo.
echo [1/3] CMake configure ...
cmake -G Ninja -B build -DBOARD=hpm5300evk -DHPM_BUILD_TYPE=flash_xip -DCMAKE_EXPORT_COMPILE_COMMANDS=ON .
if errorlevel 1 ( echo CONFIGURE FAILED & exit /b 1 )

echo.
echo [2/3] Build ...
ninja -C build -j %NUMBER_OF_PROCESSORS%
if errorlevel 1 ( echo BUILD FAILED & exit /b 1 )

echo.
echo [3/3] Flash over OpenOCD (needs FTDI/HPMicro debug probe connected) ...
openocd -c "set HPM_SDK_BASE %HPM_SDK_BASE:\=/%; set BOARD hpm5300evk; set PROBE ft2232;" -s %HPM_SDK_BASE%\boards\openocd -c "program build/output/demo.elf verify reset exit"
if errorlevel 1 ( echo FLASH FAILED - check probe & exit /b 1 )

echo.
echo DONE. demo.bin flashed.
endlocal