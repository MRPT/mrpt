@echo off
set MRPT_BASE_DIR=mrpt-0.9.5

set msvc9_DIR=d:\Program Files (x86)\Microsoft Visual Studio 9.0
set msvc10_DIR=c:\Program Files (x86)\Microsoft Visual Studio 10.0
set CMAKE_DIR=C:\Program Files (x86)\CMake 2.8\bin
set LIBUSBDIR=D:\code\libusb-win32-bin-1.2.2.0

REM msvc9 ========================
set COMP=msvc9
set ARCHN=32
set KINECT=0
call :subGen
set KINECT=1
call :subGen


set COMP=msvc9
set ARCHN=64
set KINECT=0
call :subGen
set KINECT=1
call :subGen

REM msvc10 ========================
set COMP=msvc10
set ARCHN=32
set KINECT=0
call :subGen
set KINECT=1
call :subGen

set COMP=msvc10
set ARCHN=64
set KINECT=0
call :subGen
set KINECT=1
call :subGen

goto End



REM ===== Subroutine: Generate project dir ============
:subGen

if %COMP%==msvc9 set MSVC_DIR=%msvc9_DIR%
if %COMP%==msvc10 set MSVC_DIR=%msvc10_DIR%
if %COMP%==msvc9 set CMAKE_GEN=Visual Studio 9 2008
if %COMP%==msvc10 set CMAKE_GEN=Visual Studio 10
if %ARCHN%==64 set CMAKE_GEN=%CMAKE_GEN% Win64
set ARCH=x%ARCHN%
set DIR=%MRPT_BASE_DIR%-%COMP%-%ARCH%
if %KINECT%==1 set DIR=%DIR%-kinect
if %ARCHN%==32 set ARCH_NAME=x86
if %ARCHN%==64 set ARCH_NAME=amd64
set FFMPEGDIR=D:/code/ffmpeg-git-5501afa-win%ARCHN%-dev
set WXDIR=D:/wxWidgets-2.9.2-win%ARCHN%-%COMP%
if %ARCHN%==32 set WXLIBDIR=%WXDIR%/lib/vc_dll
if %ARCHN%==64 set WXLIBDIR=%WXDIR%/lib/vc_amd64_dll

mkdir %DIR%
cd %DIR%
set ALL_PARAMS=-DDISABLE_SWISSRANGER_3DCAM_LIBS=ON -DOpenCV_DIR=d:/code/opencv-%COMP%-%ARCH% -DMRPT_HAS_FFMPEG_WIN32=ON -DFFMPEG_WIN32_ROOT_DIR=%FFMPEGDIR% -DwxWidgets_ROOT_DIR=%WXDIR% -DwxWidgets_LIB_DIR=%WXLIBDIR%

if %KINECT%==1 set ALL_PARAMS=%ALL_PARAMS% -DBUILD_KINECT=ON -DBUILD_KINECT_USE_FREENECT=ON -DLIBUSB_1_INCLUDE_DIR=%LIBUSBDIR%/include -DLIBUSB_1_LIBRARY=%LIBUSBDIR%/lib/msvc_%ARCH_NAME%/libusb.lib 

REM Create Project:
echo on
cmake ../%MRPT_BASE_DIR% -G "%CMAKE_GEN%" %ALL_PARAMS%

REM and insist to make sure wxWidgets and other vars have been fixed:
cmake . %ALL_PARAMS%
echo off

REM Create compilation script:
echo SET PATH=C:\Windows\system32;C:\Windows;C:\Program Files\TortoiseSVN\bin;D:\code\opencv-%COMP%-%ARCH%\bin\Release;D:\code\opencv-%COMP%-%ARCH%\bin\Debug;%WXLIBDIR%;%FFMPEGDIR%;%LIBUSBDIR%\bin\%ARCH_NAME%;%CMAKE_DIR%;%CD%\bin\Release;%CD%\bin\Debug >> AUTOBUILD.bat
echo call "%MSVC_DIR%\VC\vcvarsall.bat" %ARCH_NAME% >> AUTOBUILD.bat
echo call ..\%MRPT_BASE_DIR%\scripts\automated_build_msvc_binary_package.bat ..\%MRPT_BASE_DIR%\ >> AUTOBUILD.bat

cd ..

REM End of Subroutine
GOTO :EOF


REM =========== The END =========== 
:End
