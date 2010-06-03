@echo off

del ..\CMakeCache.txt > NUL 2>&1
mkdir MSVC9 > NUL 2>&1
cd MSVC9
#del CMakeCache.txt > NUL 2>&1
cmake-gui -G "Visual Studio 9 2008" ../..
IF ERRORLEVEL 0 GOTO END_BATCH
pause
:END_BATCH
