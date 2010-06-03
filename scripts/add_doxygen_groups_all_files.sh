#!/bin/bash

if [ -f ../libs/$1/include/mrpt/$1.h ]
then
	echo "Running on: ../libs/$1/include"
else
	echo "ERROR: Cannot find '../libs/$1/include'"
	echo "Usage: "
	echo "sh $0 vision" 
	exit 1
fi

FS=$(find ../libs/$1/include/ -name '*.h' | grep -v otherlibs)

for F in $FS
do
	add-doxygen-grouping-headers/add-doxygen-grouping-headers mrpt_$1 $F
	fromdos $F
done

