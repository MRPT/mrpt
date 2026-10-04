#! /bin/bash
# build_docs - Builds project documentation.
# Script: build_docs.sh
# Author: Jose Luis Blanco

# Since Dec-2019 this script just redirects the action 
# to the new Sphinx-based system Makefile.

# Checks
if [ ! -f modules/mrpt_common/package.xml ]
then
	echo "ERROR: This script must be run from the MRPT root directory."
	exit 1
fi

make -C doc
