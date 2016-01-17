#!/bin/bash
# Build all Ubuntu PPA packages and upload them.
# JLBC, 2013

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

# Build normal PPA:
export MRPT_PKG_CUSTOM_CMAKE_PARAMS=""

bash scripts/prepare_ubuntu_pkgs_for_ppa.sh
cd $HOME/mrpt_ubuntu
bash $MRPTDIR/scripts/upload_all_mrpt_ppa.sh

# Build non-SSE3 PPA:
cd $MRPTDIR
bash scripts/prepare_ubuntu_pkgs_for_ppa_no_SSE3.sh
cd $HOME/mrpt_ubuntu
bash $MRPTDIR/scripts/upload_all_mrpt_ppa_no_sse3.sh


exit 0

