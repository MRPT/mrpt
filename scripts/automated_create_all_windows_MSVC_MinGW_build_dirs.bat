@echo off
REM =========================================================
REM   Automated generation of all the dirs for win builds
REM    for x32/amd64, for all MSVC versions 
REM   To launch the build themselves and build packages, 
REM    see "automated_build_msvc_binary_package.bat"
REM 
REM  Copy this script to "d:\code" (or equivalent!), adjust 
REM   all the paths below and execute. 
REM 
REM                              Jose Luis Blanco, 2011-15
REM =========================================================

REM  === THIS IS WHERE MRPT SOURCE TREE IS FROM THE CWD ===
set MRPT_SRC_DIR=D:\BACKUPS\mrpt\mrpt_releases\MRPT-RELEASE-1.5.3\mrpt-1.5.3
set MRPT_BASE_DIR=mrpt-1.5.3

REM =================== SET ALL IMPORTANT PATHS ===================

set msvc14_DIR=C:\Program Files (x86)\Microsoft Visual Studio 14.0
set msvc141_DIR=D:\Program Files (x86)\Microsoft Visual Studio\2017\Community

set CMAKE_DIR=C:\Program Files\CMake\bin\
set LIBUSBDIR=D:\code\libusb-win32-bin
REM MinGW directories will be: %MINGW_ROOT%-32 and %MINGW_ROOT%-64  
REM  (NOTE: Use "/" for paths in this one)
set MINGW_ROOT=D:/MinGW
set MINGW_ROOT_BKSLH=D:\MinGW
REM === wxWidgets directory base name will be: %WX_ROOT%
set WX_ROOT=D:/code/wxWidgets-3.1.0
REM MSVC Redistributables: %MSVC_REDIST_BASE_DIR%/%COMP%/vcredist_%ARCH%.exe
set MSVC_REDIST_BASE_DIR=D:/code/MSVC_Redist

REM WinPCAP
set PCAP_ROOT=D:/code/WpdPack_4_1_2

REM Since mrpt 1.3.0 we can build against libusb (for libfreenect) and will work in all systems (even w/o drivers)
set KINECT=1

REM ==============================================================


REM msvc14 ========================
:gen14
set COMP=msvc14
set ARCHN=32
call :subGen

set ARCHN=64
call :subGen

goto End

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
set DIR=%MRPT_BASE_DIR%-%COMP%-%ARCH%
if %ARCHN%==32 set ARCH_NAME=x86
if %ARCHN%==64 set ARCH_NAME=amd64

set WXDIR=%WX_ROOT%

set MSVC_REDIST=%MSVC_REDIST_BASE_DIR%/%COMP%/vcredist_%ARCH%.exe

if %COMP%==mingw GOTO :subGen_mingw
REM Visual Studio --------------------------
if %COMP%==msvc12 set MSVC_DIR=%msvc12_DIR%
if %COMP%==msvc14 set MSVC_DIR=%msvc14_DIR%
if %COMP%==msvc141 set MSVC_DIR=%msvc141_DIR%

if %COMP%==msvc12 set CMAKE_GEN=Visual Studio 12 2013
if %COMP%==msvc14 set CMAKE_GEN=Visual Studio 14 2015
if %COMP%==msvc141 set CMAKE_GEN=Visual Studio 15 2017
if %ARCHN%==64 set CMAKE_GEN=%CMAKE_GEN% Win64

set CMAKE_EXTRA1=-DINSTALL_MSVC_REDISTRIBUTABLE=%MSVC_REDIST%
set CMAKE_EXTRA2=
set CMAKE_EXTRA3=

if %COMP%==msvc14 set WXLIB_DIR=vc140
if %COMP%==msvc141 set WXLIB_DIR=vc141
if %ARCHN%==64 set WXLIB_DIR=%WXLIB_DIR%_x64


set FFMPEGDIR=D:/code/ffmpeg-win%ARCHN%-dev
set WXLIBDIR=%WXDIR%/lib/%WXLIB_DIR%_dll

set PCAP_LIB=%PCAP_ROOT%/Lib/
if %ARCHN%==64 set PCAP_LIB=%PCAP_LIB%/x64
set PCAP_LIB=%PCAP_LIB%/wpcap.lib 

GOTO :subGen_common

REM MinGw (32 or 64) -----------------------

:subGen_mingw
set CMAKE_GEN=MinGW Makefiles
set CMAKE_EXTRA1=-DCMAKE_C_COMPILER=%MINGW_ROOT%-%ARCHN%/bin/gcc.exe 
set CMAKE_EXTRA2=-DCMAKE_CXX_COMPILER=%MINGW_ROOT%-%ARCHN%/bin/g++.exe
set CMAKE_EXTRA3=-DCMAKE_MAKE_PROGRAM=%MINGW_ROOT%-%ARCHN%/bin/mingw32-make.exe

set FFMPEGDIR=d:/code/ffmpeg-win%ARCHN%-dev
set WXLIBDIR=%WXDIR%/lib/gcc_lib

REM Common part to all compilers -----------
:subGen_common

mkdir %DIR%
cd %DIR%

REM ---------------- Create compilation script ----------------
set PATH_FIL=paths_%COMP%_%ARCH_NAME%
set PATH_FIL=%PATH_FIL%.bat

if NOT %COMP%==mingw set EXTRA_MINGW_PATHS=
if %COMP%==mingw set EXTRA_MINGW_PATHS=;%MINGW_ROOT_BKSLH%-%ARCHN%\bin

echo SET PATH=C:\Windows\system32;C:\Windows%EXTRA_MINGW_PATHS%;c:\code\opencv-%COMP%-%ARCH%\bin\Release;c:\code\opencv-%COMP%-%ARCH%\bin\Debug;%WXLIBDIR%;%FFMPEGDIR%/bin;%LIBUSBDIR%\bin\%ARCH_NAME%;%CMAKE_DIR%;%CD%\bin\Release;%CD%\bin\Debug > %PATH_FIL%

echo call %PATH_FIL% > AUTOBUILD.bat
rem ----- COMPILE ----- 
if NOT %COMP%==mingw echo call %MRPT_SRC_DIR%\scripts\automated_build_msvc_binary_package.bat %MRPT_SRC_DIR% >> AUTOBUILD.bat
if %COMP%==mingw echo %MINGW_ROOT_BKSLH%-%ARCHN%\bin\mingw32-make test -j4 >> AUTOBUILD.bat
if %COMP%==mingw echo %MINGW_ROOT_BKSLH%-%ARCHN%\bin\mingw32-make -j4 >> AUTOBUILD.bat
if %COMP%==mingw echo %MINGW_ROOT_BKSLH%-%ARCHN%\bin\mingw32-make package >> AUTOBUILD.bat

REM ---------------- Call CMake ----------------
call %PATH_FIL%
set ALL_PARAMS=-DDISABLE_SWISSRANGER_3DCAM_LIBS=ON -DDISABLE_PCL=ON -DDISABLE_NationalInstruments=ON -DOpenCV_DIR=c:/code/opencv-%COMP%-%ARCH% -DMRPT_HAS_FFMPEG_WIN32=ON -DFFMPEG_WIN32_ROOT_DIR=%FFMPEGDIR% -DwxWidgets_ROOT_DIR=%WXDIR% -DwxWidgets_LIB_DIR=%WXLIBDIR% -DPCAP_ROOT_DIR=%PCAP_ROOT% -DPCAP_INCLUDE_DIR=%PCAP_ROOT%/include -DPCAP_LIBRARY=%PCAP_LIB%

if %ARCHN%==32 set LIBUSBLIB=%LIBUSBDIR%\lib\msvc\libusb.lib 
if %ARCHN%==64 set LIBUSBLIB=%LIBUSBDIR%\lib\msvc_x64\libusb.lib 

if %KINECT%==1 set ALL_PARAMS=%ALL_PARAMS% -DBUILD_KINECT=ON -DBUILD_KINECT_USE_FREENECT=ON -DLIBUSB_1_INCLUDE_DIR=%LIBUSBDIR%/include -DLIBUSB_1_LIBRARY=%LIBUSBLIB%

REM Create Project:
echo on
"%CMAKE_DIR%\cmake.exe" %MRPT_SRC_DIR% -G "%CMAKE_GEN%" %ALL_PARAMS% -Wno-dev %CMAKE_EXTRA1% %CMAKE_EXTRA2% %CMAKE_EXTRA3%

echo off


rem ----- BUILD PACKAGES ----- 
echo move mrpt*.exe ..\%DIR%.exe >> AUTOBUILD.bat
echo rmdir /Q /S _CPack_Packages  >> AUTOBUILD.bat

cd ..

rem UPDATE THE "BUILD ALL" SCRIPT
echo cd %CD% >> BUILD_ALL_MRPT.bat
echo cd %DIR% >> BUILD_ALL_MRPT.bat
echo call AUTOBUILD.bat >> BUILD_ALL_MRPT.bat


REM End of Subroutine
GOTO :EOF


REM =========== The END =========== 
:End
