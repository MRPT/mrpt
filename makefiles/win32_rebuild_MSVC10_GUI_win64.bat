@echo off

del ..\CMakeCache.txt > NUL 2>&1
mkdir MSVC10_win64 > NUL 2>&1
cd MSVC10_win64
cmake-gui -G "Visual Studio 10 Win64" ../..
IF ERRORLEVEL 0 GOTO END_BATCH
pause
:END_BATCH
