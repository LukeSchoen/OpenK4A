@echo off
rem The dev-loop build: cpc, the compiler beside the OC tree.
rem
rem cpc is fast enough to rebuild on every run, which is why the dev loop uses
rem it; build_clang.cmd builds the same sources through clang for measuring.
rem Everything the Windows API contributes is resolved at run time through
rem LoadLibrary/GetProcAddress, so both builds need the same import library
rem set - in this case none.
rem
rem Every command is one line on purpose: cmd.exe's caret continuation needs
rem CRLF line endings, and a checkout that normalises them to LF turns the
rem script into a pile of stray arguments.
setlocal
cd /d "%~dp0"

if not exist bin mkdir bin

set "CPC=cpc"
if exist "%CD%\cpc.exe" set "CPC=%CD%\cpc.exe"
if not exist "%CD%\cpc.exe" if exist "C:\Luke\Src\OC\cpc.exe" set "CPC=C:\Luke\Src\OC\cpc.exe"
if not exist "%CD%\cpc.exe" if exist "C:\Luke\Src\CPrime\cpc.exe" set "CPC=C:\Luke\Src\CPrime\cpc.exe"

"%CPC%" src\openk4a_util.c src\openk4a_win.c src\openk4a_usb.c src\openk4a_depth_mcu.c src\openk4a_color_mcu.c src\openk4a_frame.c src\openk4a_depth_model.c src\openk4a_calibration.c src\openk4a_image.c src\openk4a_capture.c src\openk4a_allocator.c src\openk4a_logging.c src\openk4a_transform.c src\openk4a_imu.c src\openk4a_mf.c src\openk4a_color.c src\openk4a_device.c tools\openk4a.c -o bin\openk4a.exe -I src -I include -D_CRT_SECURE_NO_WARNINGS -DK4A_STATIC_DEFINE
if errorlevel 1 exit /b 1

echo Built bin\openk4a.exe with cpc
