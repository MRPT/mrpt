@echo off

del ..\CMakeCache.txt > NUL 2>&1
mkdir MSVC10 > NUL 2>&1
cd MSVC10
cmake-gui -G "Visual Studio 10" ../..
IF ERRORLEVEL 0 GOTO END_BATCH
pause
:END_BATCH
