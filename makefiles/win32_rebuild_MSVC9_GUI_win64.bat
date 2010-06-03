@echo off

del ..\CMakeCache.txt > NUL 2>&1
mkdir MSVC9_win64 > NUL 2>&1
cd MSVC9_win64
cmake-gui -G "Visual Studio 9 2008 Win64" ../..
IF ERRORLEVEL 0 GOTO END_BATCH
pause
:END_BATCH
