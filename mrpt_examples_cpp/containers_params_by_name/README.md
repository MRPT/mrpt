This is another use of mrpt::containers::yaml for parameter passing.

Example output:

~~~~~~~~~~~~~
CALL #1 ================================
'threshold' is 3.05
Is 'altitude' set? 1
Is 'level' set? 0
Level is : 666
Dump of all params:
%YAML 1.2
---
altitude: 100
threshold: 3.05

CALL #2 ================================
'threshold' is 3.05
Is 'altitude' set? 1
Is 'level' set? 1
Level is : -1
Dump of all params:
%YAML 1.2
---
altitude: 100
level: -1
threshold: 3.05
~~~~~~~~~~~~~
