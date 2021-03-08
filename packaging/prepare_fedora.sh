#!/bin/bash
# Copies sources from SVN tree and delete some in-work projects, for preparing a public release.
# JLBC, Aug 2008

#set -o verbose # echo on
set +o verbose # echo off

# Checks
# --------------------------------
source packaging/parse_mrpt_version.sh

MRPTSRC=`pwd`
MRPT_DEB_DIR="$HOME/mrpt_release"

# Prepare a directory for building the debian package:
#
rm -fR $MRPT_DEB_DIR
mkdir $MRPT_DEB_DIR

# Are we in svn?
MRPT_SVN_VERSION=`svnversion -n`

if [ $MRPT_SVN_VERSION = "exported" ];
then
	echo "... $MRPT_DEB_DIR/mrpt-${MRPT_VERSION_STR}"
else
	# Strip the last "M", if any:
	if [ ${MRPT_SVN_VERSION:(-1)} = "M" ];
	then
		MRPT_SVN_VERSION=${MRPT_SVN_VERSION:0:${#MRPT_SVN_VERSION}-1}
	fi

#	MRPT_VERSION_STR="${MRPT_VERSION_STR}svn${MRPT_SVN_VERSION}"
fi

# Just copy as in release,
bash packaging/prepare_release.sh

# Delete zip (not needed)
cd $MRPT_DEB_DIR
rm *.zip > /dev/null 2>/dev/null

# Rename tar.gz to Fedora convention:
mv mrpt*.tar.gz mrpt-${MRPT_VERSION_STR}-$(date +%Y%m%d)svn${MRPT_SVN_VERSION}.tar.gz

ls -l

# Create source package:
cp mrpt*.gz ${HOME}/rpmbuild/SOURCES/

ls -l ${HOME}/rpmbuild/SOURCES/


exit 0
