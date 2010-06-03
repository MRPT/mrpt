#!/bin/bash
# Copies sources from SVN tree and delete some in-work projects, for preparing a public release.
# JLBC, Aug 2008

#set -o verbose # echo on
set +o verbose # echo off

# Checks
# --------------------------------
if [ -f version_prefix.txt ];
then
	MRPT_VERSION_STR=`cat version_prefix.txt`
	MRPT_VERSION_MAJOR=${MRPT_VERSION_STR:0:1}
	MRPT_VERSION_MINOR=${MRPT_VERSION_STR:2:1}
	MRPT_VERSION_PATCH=${MRPT_VERSION_STR:4:1}
	MRPT_VER_MM="${MRPT_VERSION_MAJOR}.${MRPT_VERSION_MINOR}"
	MRPT_VER_MMP="${MRPT_VERSION_MAJOR}.${MRPT_VERSION_MINOR}.${MRPT_VERSION_PATCH}"
	echo "MRPT version: ${MRPT_VER_MMP}"
else
	echo "ERROR: Run this script from the MRPT root directory."
	exit 1
fi

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
	echo "Copying sources to $MRPT_DEB_DIR/mrpt-${MRPT_VERSION_STR}"
	cp -R . $MRPT_DEB_DIR/mrpt-${MRPT_VERSION_STR}
else
	# Strip the last "M", if any:
	if [ ${MRPT_SVN_VERSION:(-1)} = "M" ];
	then
		MRPT_SVN_VERSION=${MRPT_SVN_VERSION:0:${#MRPT_SVN_VERSION}-1}
	fi

#	MRPT_VERSION_STR="${MRPT_VERSION_STR}svn${MRPT_SVN_VERSION}"
	MRPT_VERSION_STR="${MRPT_VERSION_STR}"
	echo "Exporting to $MRPT_DEB_DIR/mrpt-${MRPT_VERSION_STR}"
	svn export . $MRPT_DEB_DIR/mrpt-${MRPT_VERSION_STR}
fi

# Copy the MRPT book:
if [ -f /Trabajo/Papers/mrpt-book/mrpt-book.ps ];
then
	cp  /Trabajo/Papers/mrpt-book/mrpt-book.ps $MRPT_DEB_DIR/mrpt-${MRPT_VERSION_STR}/doc/
	ps2pdf $MRPT_DEB_DIR/mrpt-${MRPT_VERSION_STR}/doc/mrpt-book.ps $MRPT_DEB_DIR/mrpt-${MRPT_VERSION_STR}/doc/mrpt-book.pdf
	gzip $MRPT_DEB_DIR/mrpt-${MRPT_VERSION_STR}/doc/mrpt-book.ps
fi

#printf "Generating mrpt.spec ..."
#eval "echo \"`cat mrpt.spec.in`\"" > $MRPT_DEB_DIR/mrpt-${MRPT_VERSION_STR}/mrpt.spec
#printf "OK\n"


cd $MRPT_DEB_DIR/mrpt-${MRPT_VERSION_STR}
echo "Deleting some files..."

# Deletions:
rm -fR lib
rm -fR packaging

rm -fR apps/HolonomicNavigatorTester
rm -fR apps/PTG_Designer
rm -fR apps/SimpleMapsViewer
rm -fR apps/vOdometry
rm -fR share/mrpt/config_files/vOdometry

# Not stable yet...
rm -fR apps/hmt-slam
rm -fR apps/hmt-slam-gui
rm -fR apps/hmtMapViewer


rm -fr apps/*monoslam*
rm -fr libs/*monoslam*
rm -fr share/applications/monoslam.desktop

rm -fr libs/stereoslam

# And remove the corrs. lines:
#(echo "g/console2gui/d"; echo 'wq') | ex -s apps/CMakeLists.txt


# Orig tarball:
cd ..
echo "Creating orig tarball: mrpt-${MRPT_VERSION_STR}.tar.gz"
tar czf mrpt-${MRPT_VERSION_STR}.tar.gz mrpt-${MRPT_VERSION_STR}

# Create .zip file with DOS line endings
echo "Creating orig zip in DOS format: mrpt-${MRPT_VERSION_STR}.zip"
cd mrpt-${MRPT_VERSION_STR}
bash scripts/all_files_DOS_format.sh
cd ..
zip mrpt-${MRPT_VERSION_STR}.zip -r mrpt-${MRPT_VERSION_STR}/*

rm -fr mrpt-${MRPT_VERSION_STR}

exit 0

