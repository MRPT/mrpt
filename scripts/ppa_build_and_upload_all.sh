#!/bin/bash
# Build all Ubuntu PPA packages and upload them.
# JLBC, 2013

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
export MRPT_PKG_CUSTOM_CMAKE_PARAMS=""

bash scripts/prepare_ubuntu_pkgs_for_ppa.sh
cd $HOME/mrpt_ubuntu
bash $MRPTDIR/scripts/upload_all_mrpt_ppa.sh

exit 0
