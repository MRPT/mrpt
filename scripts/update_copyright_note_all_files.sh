#!/bin/bash

# This script updates the copyright comment block at the beginning of
#  every file in the MRPT, and unifies the new line characters.
#  Jose Luis Blanco, March 2008.

F1=$(find ../src/ -name '*.cpp')
F2=$(find ../include/mrpt/ -name '*.h' | grep -v  otherlibs)
F3=$(find ../apps/ -name '*.cpp')
F4=$(find ../apps/ -name '*.h')
F5=$(find ../samples/ -name '*.cpp')


for F in $F1 $F2 $F3 $F4 $F5
do
	replace-header/replace-header $F
	dos2unix $F
done


