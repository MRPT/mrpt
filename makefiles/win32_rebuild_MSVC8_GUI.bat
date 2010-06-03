@echo off

del ..\CMakeCache.txt > NUL 2>&1
mkdir MSVC8 > NUL 2>&1
cd MSVC8
#del CMakeCache.txt > NUL 2>&1
cmake-gui -G "Visual Studio 8 2005" ../..

IF ERRORLEVEL 0 GOTO END_BATCH
pause
:END_BATCH
