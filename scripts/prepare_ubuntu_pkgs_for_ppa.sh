#!/bin/bash
# Creates a set of packages for each different Ubuntu distribution, with the 
# intention of uploading them to: 
#   https://launchpad.net/~joseluisblancoc/+archive/ppa-mrpt
#
# JLBC, 2010

# Checks
# --------------------------------
if [ -f version_prefix.txt ];
then
	MRPT_VERSION_STR=`cat version_prefix.txt`
	MRPT_VERSION_MAJOR=${MRPT_VERSION_STR:0:1}
	MRPT_VERSION_MINOR=${MRPT_VERSION_STR:2:1}
	MRPT_VERSION_PATCH=${MRPT_VERSION_STR:4:1}
	AUX_SVN=$(svnversion)
	#Remove the trailing "M":
	MRPT_VERSION_SVN=${AUX_SVN%M}

	MRPT_VER_MM="${MRPT_VERSION_MAJOR}.${MRPT_VERSION_MINOR}"
	MRPT_VER_MMP="${MRPT_VERSION_MAJOR}.${MRPT_VERSION_MINOR}.${MRPT_VERSION_PATCH}"
	echo "MRPT version: ${MRPT_VER_MMP} (SVN: ${MRPT_VERSION_SVN})"
else
	echo "ERROR: Run this script from the MRPT root directory."
	exit 1
fi

MRPTSRC=`pwd`
MRPT_DEB_DIR="$HOME/mrpt_debian"
MRPT_EXTERN_DEBIAN_DIR="$MRPTSRC/packaging/debian/"
EMAIL4DEB="Jose Luis Blanco (University of Malaga) <joseluisblancoc@gmail.com>"

# -------------------------------------------------------------------
# Call the standard "prepare_debian.sh" script:
# -------------------------------------------------------------------
cd ${MRPTSRC}
bash scripts/prepare_debian.sh


# -------------------------------------------------------------------
# And now create the custom packages for each Ubuntu distribution
# -------------------------------------------------------------------
#LST_DISTROS="hardy jaunty karmic lucid maverick"
LST_DISTROS="hardy"

NEW_DEBIAN_VER=1
DEBCHANGE_CMD="--newversion 1:${MRPT_VERSION_STR}-1"
for DEBIAN_DIST in ${LST_DISTROS};
do
	cd ${MRPT_DEB_DIR}/mrpt-${MRPT_VER_MMP}/debian
	pwd
	cp ${MRPT_EXTERN_DEBIAN_DIR}/changelog changelog
	echo "Changing to a new Debian version: ${MRPT_VERSION_STR}svn${MRPT_VERSION_SVN}-${NEW_DEBIAN_VER}"
	echo "Adding a new entry to debian/changelog for distribution ${DEBIAN_DIST}"
	DEBEMAIL=${EMAIL4DEB} debchange $DEBCHANGE_CMD --distribution ${DEBIAN_DIST} --force-distribution New version of upstream sources.

	echo "Now, let's build the source Deb package with 'debuild -S -sa':"
	cd ..
	debuild -S -sa
done





exit 0

