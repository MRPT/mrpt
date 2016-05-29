#!/bin/bash
# This script is intended for automated run in Debian package generators and
# alike. It assumes the CWD is the BUILD directory.
# It will compile all unit tests and try to run them, showing the backtrace
# with GDB in the event of any failure.

set -e
set -x

make tests_build_all

find ./tests/test_mrpt_* -type f -print0 | while IFS= read -r -d $'\0' FIL; do
	echo "[#### Running test $FIL ####]"
	gdb -batch -ex "run" -ex "bt" -return-child-result $FIL
	echo "[#### End run test $FIL ####]"
done
