@echo off
rem The measuring build: everything, through clang and lld.
rem
rem clang finds the Windows SDK's headers and libraries itself; the MSVC
rem import libraries are named at the end, and only for the subsystems that
rem need them. Nothing here is clang-specific, so build_cpc.cmd builds the
rem same sources with the tree's dev compiler.
rem
rem Every command is one line on purpose. cmd.exe's caret continuation needs
rem CRLF line endings, and a checkout that normalises them to LF turns the
rem whole script into a pile of stray arguments; a long line cannot be
rem broken by anything.
setlocal
cd /d "%~dp0"

if not exist bin mkdir bin

rem Prefer a clang beside this script; then the machine's copies.
set "CLANG=%CD%\clang.exe"
set "LLD=%CD%\lld-link.exe"
if not exist "%CLANG%" set "CLANG=C:\Luke\Src\Clang\clang.exe"
if not exist "%LLD%" set "LLD=C:\Luke\Src\Clang\lld-link.exe"
if not exist "%CLANG%" set "CLANG=C:\Luke\Src\Kinect\Kinect4\clang.exe"
if not exist "%LLD%" set "LLD=C:\Luke\Src\Kinect\Kinect4\lld-link.exe"
if not exist "%CLANG%" (
    echo Missing clang: set CLANG to one on the command line
    exit /b 1
)

set "PATH=%CLANG%"
for %%D in ("%CLANG%") do set "PATH=%%~dpD;%PATH%"
if exist "%LLD%" for %%D in ("%LLD%") do set "PATH=%%~dpD;%PATH%"

rem -mavx2 -mfma are what the depth pass's filter runs eight pixels at a time
rem on; the scalar path is compiled either way and both are held to the same
rem output by the MiniKinect tree's smoke test.
"%CLANG%" src\openk4a_util.c src\openk4a_win.c src\openk4a_usb.c src\openk4a_depth_mcu.c src\openk4a_color_mcu.c src\openk4a_frame.c src\openk4a_depth_model.c src\openk4a_calibration.c src\openk4a_image.c src\openk4a_capture.c src\openk4a_allocator.c src\openk4a_logging.c src\openk4a_transform.c src\openk4a_imu.c src\openk4a_mf.c src\openk4a_color.c src\openk4a_device.c tools\openk4a.c -o bin\openk4a.exe --target=x86_64-pc-windows-msvc -I src -I include -O2 -mavx2 -mfma -D_CRT_SECURE_NO_WARNINGS -DK4A_STATIC_DEFINE -fuse-ld=lld -lkernel32 -ladvapi32 -lole32
if errorlevel 1 exit /b 1

echo Built bin\openk4a.exe with clang
