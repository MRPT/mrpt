#!/bin/sh

if [ "$1" != "todos" ] && [ "$1" != "fromdos" ];
then
	echo "usage: all_files_DOS_format.sh OR all_files_UNIX_format.sh"
	exit 1
fi

echo "Running format change with command: $1"

find . -name "*.cpp" | xargs $1
find . -name "*.c" | xargs $1
find . -name "*.h" | xargs $1
find . -name "*.hpp" | xargs $1 
find . -name "*.tpp" | xargs $1 
find . -name "CMakeLists.txt" | xargs $1 
find . -name "AUTHORS" | xargs $1 
find samples -name "*.txt" | xargs $1 
find samples -name "*.ini" | xargs $1 
find samples -name "*.INI" | xargs $1 
find doc -name "*.js" | xargs $1 
find doc -name "*.txt" | xargs $1 

