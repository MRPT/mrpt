set DIR=%CD%

:Loop
IF "%1"=="" GOTO Continue
cd %1
del /s *.obj  
del /s *.pch
del /s *.pdb
del /s *.ilk
del /s *.idb
del /s *.gch
SHIFT
cd %DIR%
GOTO Loop
:Continue

