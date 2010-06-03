@echo off

del ..\CMakeCache.txt > NUL 2>&1
mkdir MSVC8_win64 > NUL 2>&1
cd MSVC8
cmake-gui -G "Visual Studio 8 2005 Win64" ../..
IF ERRORLEVEL 0 GOTO END_BATCH
pause
:END_BATCH
