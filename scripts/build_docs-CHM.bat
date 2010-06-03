@echo off
REM --------------------------------------------------------
REM    Batch file for generating the MRPT's documentation 
REM     in Windows. Requires MinGW & MSys for Windows
REM     (see http://www.mingw.org/)
REM          			Jose Luis Blanco, Aug 2007
REM --------------------------------------------------------

REM Run the sh script:
ECHO ON

sh.exe scripts/build_docs.sh -c

@ECHO OFF
IF ERRORLEVEL 0 GOTO END_BATCH

ECHO -------------------------------------------------------------
ECHO   It seems you don't have MinGW & MSys installed in your system!
ECHO    (Couldn't find "sh.exe") 
ECHO   If it is installed, make sure it is in the PATH
ECHO   You can download MinGW freely from: http://www.mingw.org/
ECHO -------------------------------------------------------------
ECHO.

:END_BATCH
pause
