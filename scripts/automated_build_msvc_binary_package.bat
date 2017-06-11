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

set MSVC_VERBOSITY=normal

REM Make sure params are OK:
REM ------------------------------
IF "%1" == "" GOTO SHOW_USAGE
IF NOT EXIST "%1" GOTO SHOW_USAGE2
IF NOT EXIST "%1\version_prefix.txt" GOTO SHOW_USAGE2

IF EXIST ".\version_prefix.txt" GOTO NO_GOOD

REM 1) Compile debug libs (so Cmake find them in the next run)
REM ----------------------------------------------
cmake --build . --config Debug --target all_mrpt_libs
IF %ERRORLEVEL% NEQ 0 GOTO BAD_RETCODE

REM 2) re-call CMake to detect the debug libs
REM ----------------------------------------------
cmake . 
IF %ERRORLEVEL% NEQ 0 GOTO BAD_RETCODE

REM 3) Do unit tests:
REM ----------------------------------------------
cmake --build . --config Release --target test
IF %ERRORLEVEL% NEQ 0 GOTO BAD_RETCODE

REM 4) All seem OK. Build all.
REM ----------------------------------------------
cmake --build . --config Release
IF %ERRORLEVEL% NEQ 0 GOTO BAD_RETCODE

REM 5) Build package:
REM ----------------------------------------------
cmake --build . --config Release --target PACKAGE

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
