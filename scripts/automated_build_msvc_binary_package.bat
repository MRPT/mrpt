REM @echo off
REM ---------------------------------------------------------------------------
REM    Batch file for generating MRPT binary packages for MSVC binary releases
REM 
REM  Usage: 
REM    From a directory already with a CMakeCache.txt:
REM 
REM  > automated_build_msvc_binary_package.bat <PATH_TO_MRPT_SRCS> 
REM
REM ---------------------------------------------------------------------------

REM  Extra params we want on all public binary releases:
set EXTRA_CMAKE_VARS=-DDISABLE_SWISSRANGER_3DCAM_LIBS=ON -DDISABLE_PCL=ON -DDISABLE_NationalInstruments=ON -DENABLE_SOLUTION_FOLDERS=OFF 
REM set MSBUILDPARALLEL=/maxcpucount:2

REM set MSVC_VERBOSITY=minimal
set MSVC_VERBOSITY=normal

REM Make sure params are OK:
REM ------------------------------
IF "%1" == "" GOTO SHOW_USAGE
IF NOT EXIST "%1" GOTO SHOW_USAGE2
IF NOT EXIST "%1\version_prefix.txt" GOTO SHOW_USAGE2

IF EXIST ".\version_prefix.txt" GOTO NO_GOOD

REM 1) re-call CMake
REM ----------------------------------------------
cmake . %1 %EXTRA_CMAKE_VARS%
IF %ERRORLEVEL% NEQ 0 GOTO BAD_RETCODE

REM 2) Compile debug libs (so Cmake find them in the next run)
REM ----------------------------------------------
:RETRY_BUILD
REM msbuild libs\ALL_MRPT_LIBS.sln /p:Configuration=Debug %MSBUILDPARALLEL% /verbosity:%MSVC_VERBOSITY%
devenv libs\ALL_MRPT_LIBS.sln /Build Debug
REM IF %ERRORLEVEL% NEQ 0 GOTO BAD_RETCODE
REM Repeat several times with MSVC9 since its linker crashes... (yes, fuck yeah!)
IF %ERRORLEVEL% NEQ 0 GOTO RETRY_BUILD



REM 3) re-call CMake to detect the debug libs
REM ----------------------------------------------
cmake . 
IF %ERRORLEVEL% NEQ 0 GOTO BAD_RETCODE

REM 4) Do unit tests:
REM ----------------------------------------------
REM msbuild tests\tests.sln /p:Configuration=Release %MSBUILDPARALLEL% /verbosity:%MSVC_VERBOSITY%
devenv tests\tests.sln /Build Release
IF %ERRORLEVEL% NEQ 0 GOTO BAD_RETCODE

REM 5) All seem OK. Build all.
REM ----------------------------------------------
REM msbuild MRPT.sln /p:Configuration=Release %MSBUILDPARALLEL% /verbosity:%MSVC_VERBOSITY%
devenv MRPT.sln /Build Release

IF %ERRORLEVEL% NEQ 0 GOTO BAD_RETCODE


REM 6) Build package:
REM ----------------------------------------------
REM IF EXIST PACKAGE.vcproj msbuild PACKAGE.vcproj /p:Configuration=Release /verbosity:detailed
REM IF EXIST PACKAGE.vcxproj msbuild PACKAGE.vcxproj /p:Configuration=Release /verbosity:detailed
devenv MRPT.sln /Build release /project PACKAGE

goto END_BATCH
REM ============== END ====================


:SHOW_USAGE
ECHO  Usage: 
ECHO    From a directory already with a CMakeCache.txt:
ECHO.
ECHO  automated_build_msvc_binary_package.bat (PATH_TO_MRPT_SRCS)
ECHO.
goto END_BATCH

:SHOW_USAGE2
ECHO Error: Supplied path "%1" doesn't exist or is not the MRPT source root dir.
echo.
goto SHOW_USAGE

:NO_GOOD
ECHO.
ECHO Error: It's not a good idea to build in the source tree. 
echo Please try in another directory.
echo.
goto END_BATCH

:BAD_RETCODE
ECHO.
ECHO *****************************************
ECHO ** THE LAST COMMAND FAILED! Look at it **
ECHO *****************************************
ECHO.
goto END_BATCH


:END_BATCH

