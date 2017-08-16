@echo off
REM =========================================================
REM   Automated generation of all the dirs for win builds
REM    for x32/amd64 and for MSVC{9,10,11} and MinGW
REM 
REM  Copy this script to "d:\code" (in my laptop!), adjust 
REM   all the paths below and execute. 
REM 
REM                              Jose Luis Blanco, 2012
REM =========================================================

REM  === THIS IS WHERE OpenCV SOURCE TREE IS FROM THE CWD ===
set OPENCV_BASE_DIR=opencv-3.1.0
set OPENCV_CONTRIB_DIR=opencv_contrib-3.1.0

REM =================== SET ALL IMPORTANT PATHS ===================

set msvc14_DIR=C:\Program Files (x86)\Microsoft Visual Studio 14.0
set msvc141_DIR=D:\Program Files (x86)\Microsoft Visual Studio\2017\Community
REM MinGW directories will be: %MINGW_ROOT%-32 and %MINGW_ROOT%-64  
REM  (NOTE: Use "/" for paths in this one)
set MINGW_ROOT=d:/MinGW
set MINGW_ROOT_BKSLH=d:\MinGW
set CMAKE_DIR=C:\Program Files\CMake\bin\
REM ==============================================================

del BUILD_ALL_OPENCV.bat 2> NUL

goto gen141

REM msvc14 ========================
:gen14
set COMP=msvc14
set ARCHN=32
call :subGen

set ARCHN=64
call :subGen

REM msvc141 ========================
:gen141
set COMP=msvc141
set ARCHN=32
call :subGen

set ARCHN=64
call :subGen

goto End

:MINGW_PARTS
REM MinGW ========================
set COMP=mingw
set ARCHN=32
call :subGen

set ARCHN=64
call :subGen

goto End



REM ===== Subroutine: Generate project dir ============
:subGen

set ARCH=x%ARCHN%
set DIR=opencv-%COMP%-%ARCH%
if %ARCHN%==32 set ARCH_NAME=x86
if %ARCHN%==64 set ARCH_NAME=amd64

if %COMP%==mingw GOTO :subGen_mingw
REM Visual Studio --------------------------
if %COMP%==msvc14 set MSVC_DIR=%msvc14_DIR%
if %COMP%==msvc14 set CMAKE_GEN=Visual Studio 14 2015

if %COMP%==msvc141 set MSVC_DIR=%msvc15_DIR%
if %COMP%==msvc141 set CMAKE_GEN=Visual Studio 15 2017

if %ARCHN%==64 set CMAKE_GEN=%CMAKE_GEN% Win64

set CMAKE_EXTRA1=
set CMAKE_EXTRA2=
set CMAKE_EXTRA3=

GOTO :subGen_common

REM MinGw (32 or 64) -----------------------

:subGen_mingw
set CMAKE_GEN=MinGW Makefiles
set CMAKE_EXTRA1=-DCMAKE_C_COMPILER=%MINGW_ROOT%-%ARCHN%/bin/gcc.exe
set CMAKE_EXTRA2=-DCMAKE_CXX_COMPILER=%MINGW_ROOT%-%ARCHN%/bin/g++.exe
set CMAKE_EXTRA3=-DCMAKE_MAKE_PROGRAM=%MINGW_ROOT%-%ARCHN%/bin/mingw32-make.exe

REM Common part to all compilers -----------
:subGen_common


mkdir %DIR%
cd %DIR%

REM ---------------- Create compilation script ----------------
set PATH_FIL=paths_%COMP%_%ARCH_NAME%
set PATH_FIL=%PATH_FIL%.bat

if NOT %COMP%==mingw set EXTRA_MINGW_PATHS=
if %COMP%==mingw set EXTRA_MINGW_PATHS=;%MINGW_ROOT_BKSLH%-%ARCHN%\bin

echo SET PATH=C:\Windows\system32;C:\Windows%EXTRA_MINGW_PATHS%;%CMAKE_DIR%;%CD%\bin\Release;%CD%\bin\Debug > %PATH_FIL%

echo call %PATH_FIL% > AUTOBUILD.bat
if NOT %COMP%==mingw echo cmake --build . --config Release >> AUTOBUILD.bat
if NOT %COMP%==mingw echo cmake --build . --config Debug >> AUTOBUILD.bat
if %COMP%==mingw echo %MINGW_ROOT_BKSLH%-%ARCHN%\bin\mingw32-make -j4 >> AUTOBUILD.bat

REM ---------------- Call CMake ----------------
call %PATH_FIL%
set ALL_PARAMS=-DBUILD_EXAMPLES=OFF -DBUILD_TESTS=OFF -DBUILD_PERF_TESTS=OFF -DWITH_CUDA=OFF -DOPENCV_EXTRA_MODULES_PATH=../%OPENCV_CONTRIB_DIR%/modules -DBUILD_opencv_aruco=OFF -DBUILD_opencv_bioinspired=OFF -DBUILD_opencv_contrib_world=OFF -DBUILD_opencv_fuzzy=OFF -DBUILD_opencv_latentsvm=OFF -DBUILD_opencv_photo=OFF -DBUILD_opencv_rgbd=OFF -DBUILD_opencv_stitching=OFF -DBUILD_opencv_superres=OFF -DBUILD_opencv_ts=OFF -DBUILD_opencv_world=OFF -DBUILD_opencv_xobjdetect=OFF -DBUILD_opencv_xphoto=OFF

REM Create Project:
echo on
"%CMAKE_DIR%\cmake.exe" ../%OPENCV_BASE_DIR% -G "%CMAKE_GEN%" %ALL_PARAMS% %CMAKE_EXTRA1% %CMAKE_EXTRA2% %CMAKE_EXTRA3% 

REM and insist to make sure all vars have been fixed:
"%CMAKE_DIR%\cmake.exe" . %ALL_PARAMS%
echo off


cd ..

rem UPDATE THE "BUILD ALL" SCRIPT
echo cd %CD% >> BUILD_ALL_OPENCV.bat
echo cd %DIR% >> BUILD_ALL_OPENCV.bat
echo call AUTOBUILD.bat >> BUILD_ALL_OPENCV.bat

REM End of Subroutine
GOTO :EOF


REM =========== The END =========== 
:End
