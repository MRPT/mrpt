@echo off

del ..\CMakeCache.txt > NUL 2>&1
mkdir mingw-win32 > NUL 2>&1
cd mingw-win32
cmake-gui ../..
IF ERRORLEVEL 0 GOTO END_BATCH
pause
:END_BATCH
