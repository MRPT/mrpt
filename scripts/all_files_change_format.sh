#!/bin/sh

if [ "$1" != "todos" ] && [ "$1" != "fromdos" ];
then
	echo "usage: all_files_DOS_format.sh OR all_files_UNIX_format.sh"
	exit 1
fi

#echo "Running format change with command: $1"

find . \
	-name "*.cpp" \
	-or -name "*.c" \
	-or -name "*.h" \
	-or -name "*.in" \
	-or -name "*.hpp" \
	-or -name "*.tpp" \
	-or -name "CMakeLists.txt" \
	-or -name "AUTHORS" \
	-or -name "*.ini"
	-or -name "*.INI"
	-or -name "*.txt"
	| xargs $1
