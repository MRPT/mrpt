#!/bin/bash
# Build all Ubuntu PPA packages and upload them.
# JLBC, 2013-2021
#
# TODO: Update script to work with gdb!

set -e

# Check:
if [ -f version_prefix.txt ];
then
	MRPT_VERSION_STR=`head -n 1 version_prefix.txt`
	echo "MRPT version: ${MRPT_VERSION_STR}"
else
	echo "ERROR: Run this script from the MRPT root directory."
	exit 1
fi

MRPTDIR=`pwd`

# Build PPA packages:
bash packaging/prepare_ubuntu_pkgs_for_ppa.sh
cd $HOME/mrpt_ubuntu

# upload:
find . -name '*.changes' | xargs -I FIL dput ppa:joseluisblancoc/mrpt FIL

exit 0
