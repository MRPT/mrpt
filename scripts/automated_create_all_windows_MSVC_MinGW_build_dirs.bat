@echo off
REM =========================================================
REM   Automated generation of all the dirs for win builds
REM    for x32/amd64, with/without Kinect support, and 
REM    for MSVC{9,10,11} and MinGW
REM   To launch the build themselves and build packages, 
REM    see "automated_build_msvc_binary_package.bat"
REM 
REM  Copy this script to "d:\code" (in my laptop!), adjust 
REM   all the paths below and execute. 
REM 
REM                              Jose Luis Blanco, 2011-12
REM =========================================================

REM  === THIS IS WHERE MRPT SOURCE TREE IS FROM THE CWD ===
set MRPT_BASE_DIR=mrpt-1.0.3

REM =================== SET ALL IMPORTANT PATHS ===================

set msvc9_DIR=E:\Program Files (x86)\Microsoft Visual Studio 9.0
rem set msvc10_DIR=E:\Program Files (x86)\Microsoft Visual Studio 10.0
set msvc11_DIR=E:\Archivos de programa (x86)\Microsoft Visual Studio 11.0
set msvc12_DIR=E:\Archivos de programa (x86)\Microsoft Visual Studio 12.0

set CMAKE_DIR=E:\Program Files (x86)\CMake\bin\
set LIBUSBDIR=E:\code\libusb-win32-bin-1.2.6.0
REM MinGW directories will be: %MINGW_ROOT%-32 and %MINGW_ROOT%-64  
REM  (NOTE: Use "/" for paths in this one)
set MINGW_ROOT=E:/MinGW
set MINGW_ROOT_BKSLH=E:\MinGW
REM === wxWidgets directory base name will be: %WX_ROOT%-win%ARCHN%-%COMP%
set WX_ROOT=E:/code/wxWidgets-2.8.12
REM MSVC Redistributables: %MSVC_REDIST_BASE_DIR%/%COMP%/vcredist_%ARCH%.exe
set MSVC_REDIST_BASE_DIR=E:/code/MSVC_Redist


REM ==============================================================

REM msvc9 ========================
set COMP=msvc9
set ARCHN=32
set KINECT=0
call :subGen
set KINECT=1
call :subGen

set ARCHN=64
set KINECT=0
call :subGen
set KINECT=1
call :subGen

REM msvc10 ========================
rem set COMP=msvc10
rem set ARCHN=32
rem set KINECT=0
rem call :subGen
rem set KINECT=1
rem call :subGen

rem set ARCHN=64
rem set KINECT=0
rem call :subGen
rem set KINECT=1
rem call :subGen

REM msvc11 ========================
set COMP=msvc11
set ARCHN=32
set KINECT=0
call :subGen

set KINECT=1
call :subGen

set ARCHN=64
set KINECT=0
call :subGen
set KINECT=1
call :subGen

REM msvc12 ========================
set COMP=msvc12
set ARCHN=32
set KINECT=0
call :subGen

set KINECT=1
call :subGen

set COMP=msvc12

set ARCHN=64
set KINECT=0
call :subGen
set KINECT=1
call :subGen



goto End
:MINGW_PARTS
REM MinGW ========================
set COMP=mingw
set ARCHN=32
set KINECT=0
call :subGen
set KINECT=1
call :subGen

set ARCHN=64
set KINECT=0
call :subGen
set KINECT=1
call :subGen


goto End



REM ===== Subroutine: Generate project dir ============
:subGen

set ARCH=x%ARCHN%
set DIR=%MRPT_BASE_DIR%-%COMP%-%ARCH%
if %KINECT%==1 set DIR=%DIR%-kinect
if %ARCHN%==32 set ARCH_NAME=x86
if %ARCHN%==64 set ARCH_NAME=amd64

set WXDIR=%WX_ROOT%-%COMP%-%ARCH%

set MSVC_REDIST=%MSVC_REDIST_BASE_DIR%/%COMP%/vcredist_%ARCH%.exe

if %COMP%==mingw GOTO :subGen_mingw
REM Visual Studio --------------------------
if %COMP%==msvc9 set MSVC_DIR=%msvc9_DIR%
if %COMP%==msvc10 set MSVC_DIR=%msvc10_DIR%
if %COMP%==msvc11 set MSVC_DIR=%msvc11_DIR%
if %COMP%==msvc12 set MSVC_DIR=%msvc12_DIR%
if %COMP%==msvc9 set CMAKE_GEN=Visual Studio 9 2008
if %COMP%==msvc10 set CMAKE_GEN=Visual Studio 10 2010
if %COMP%==msvc11 set CMAKE_GEN=Visual Studio 11 2012
if %COMP%==msvc12 set CMAKE_GEN=Visual Studio 12 2013
if %ARCHN%==64 set CMAKE_GEN=%CMAKE_GEN% Win64

set CMAKE_EXTRA1=-DINSTALL_MSVC_REDISTRIBUTABLE=%MSVC_REDIST%
set CMAKE_EXTRA2=
set CMAKE_EXTRA3=

set FFMPEGDIR=E:/code/ffmpeg-win%ARCHN%-dev
if %ARCHN%==32 set WXLIBDIR=%WXDIR%/lib/vc_dll
if %ARCHN%==64 set WXLIBDIR=%WXDIR%/lib/vc_amd64_dll

GOTO :subGen_common

REM MinGw (32 or 64) -----------------------

:subGen_mingw
set CMAKE_GEN=MinGW Makefiles
set CMAKE_EXTRA1=-DCMAKE_C_COMPILER=%MINGW_ROOT%-%ARCHN%/bin/gcc.exe 
set CMAKE_EXTRA2=-DCMAKE_CXX_COMPILER=%MINGW_ROOT%-%ARCHN%/bin/g++.exe
set CMAKE_EXTRA3=-DCMAKE_MAKE_PROGRAM=%MINGW_ROOT%-%ARCHN%/bin/mingw32-make.exe

set FFMPEGDIR=E:/code/ffmpeg-win%ARCHN%-dev
set WXLIBDIR=%WXDIR%/lib/gcc_lib

REM Common part to all compilers -----------
:subGen_common

mkdir %DIR%
cd %DIR%

REM ---------------- Create compilation script ----------------
set PATH_FIL=paths_%COMP%_%ARCH_NAME%
if %KINECT%==1 set PATH_FIL=%PATH_FIL%-kinect
set PATH_FIL=%PATH_FIL%.bat

if NOT %COMP%==mingw set EXTRA_MINGW_PATHS=
if %COMP%==mingw set EXTRA_MINGW_PATHS=;%MINGW_ROOT_BKSLH%-%ARCHN%\bin

echo SET PATH=C:\Windows\system32;C:\Windows%EXTRA_MINGW_PATHS%;C:\Program Files\TortoiseSVN\bin;E:\code\opencv-%COMP%-%ARCH%\bin\Release;E:\code\opencv-%COMP%-%ARCH%\bin\Debug;%WXLIBDIR%;%FFMPEGDIR%/bin;%LIBUSBDIR%\bin\%ARCH_NAME%;%CMAKE_DIR%;%CD%\bin\Release;%CD%\bin\Debug > %PATH_FIL%
if NOT %COMP%==mingw echo call "%MSVC_DIR%\VC\vcvarsall.bat" %ARCH_NAME% >> %PATH_FIL%

echo call %PATH_FIL% > AUTOBUILD.bat
rem ----- COMPILE ----- 
if NOT %COMP%==mingw echo call ..\%MRPT_BASE_DIR%\scripts\automated_build_msvc_binary_package.bat ..\%MRPT_BASE_DIR%\ >> AUTOBUILD.bat
if %COMP%==mingw echo %MINGW_ROOT_BKSLH%-%ARCHN%\bin\mingw32-make test -j4 >> AUTOBUILD.bat
if %COMP%==mingw echo %MINGW_ROOT_BKSLH%-%ARCHN%\bin\mingw32-make -j4 >> AUTOBUILD.bat
if %COMP%==mingw echo %MINGW_ROOT_BKSLH%-%ARCHN%\bin\mingw32-make package >> AUTOBUILD.bat

REM ---------------- Call CMake ----------------
call %PATH_FIL%
set ALL_PARAMS=-DDISABLE_SWISSRANGER_3DCAM_LIBS=ON -DDISABLE_PCL=ON -DDISABLE_NationalInstruments=ON -DOpenCV_DIR=E:/code/opencv-%COMP%-%ARCH% -DMRPT_HAS_FFMPEG_WIN32=ON -DFFMPEG_WIN32_ROOT_DIR=%FFMPEGDIR% -DwxWidgets_ROOT_DIR=%WXDIR% -DwxWidgets_LIB_DIR=%WXLIBDIR%

if %ARCHN%==32 set LIBUSBLIB=%LIBUSBDIR%\lib\msvc\libusb.lib 
if %ARCHN%==64 set LIBUSBLIB=%LIBUSBDIR%\lib\msvc_x64\libusb.lib 

if %KINECT%==1 set ALL_PARAMS=%ALL_PARAMS% -DBUILD_KINECT=ON -DBUILD_KINECT_USE_FREENECT=ON -DLIBUSB_1_INCLUDE_DIR=%LIBUSBDIR%/include -DLIBUSB_1_LIBRARY=%LIBUSBLIB%

REM Create Project:
echo on
"%CMAKE_DIR%\cmake.exe" ../%MRPT_BASE_DIR% -G "%CMAKE_GEN%" %ALL_PARAMS% -Wno-dev %CMAKE_EXTRA1% %CMAKE_EXTRA2% %CMAKE_EXTRA3%

REM and insist to make sure wxWidgets and other vars have been fixed:
"%CMAKE_DIR%\cmake.exe" . -Wno-dev %ALL_PARAMS%
echo off


rem ----- BUILD PACKAGES ----- 
echo move mrpt*.exe ..\%DIR%.exe >> AUTOBUILD.bat
echo rmdir /Q /S _CPack_Packages  >> AUTOBUILD.bat

cd ..

rem UPDATE THE "BUILD ALL" SCRIPT
echo cd %CD% >> BUILD_ALL_MRPT.bat
echo cd %DIR% >> BUILD_ALL_MRPT.bat
echo AUTOBUILD.bat >> BUILD_ALL_MRPT.bat


REM End of Subroutine
GOTO :EOF


REM =========== The END =========== 
:End
