#!/bin/sh

if [ "$1" != "todos" ] && [ "$1" != "fromdos" ];
then
	echo "usage: all_files_DOS_format.sh OR all_files_UNIX_format.sh"
	exit 1
fi

#echo "Running format change with command: $1"

find . \
	-name "*.cpp" \
	-o -name "*.c" \
	-o -name "*.h" \
	-o -name "*.in" \
	-o -name "*.hpp" \
	-o -name "*.tpp" \
	-o -name "CMakeLists.txt" \
	-o -name "AUTHORS" \
	-o -name "*.ini" \
	-o -name "*.INI" \
	-o -name "*.txt" \
	| xargs $1
