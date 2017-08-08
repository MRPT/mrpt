#!/bin/bash
# Prepare to build a Debian package.
# JLBC, 2008-2017

set -e   # end on error

APPEND_SNAPSHOT_NUM=0
IS_FOR_UBUNTU=0
SKIP_HEAVY_DOCS=0
APPEND_LINUX_DISTRO=""
VALUE_EXTRA_CMAKE_PARAMS=""
while getopts "suhd:c:" OPTION
do
     case $OPTION in
         s)
             APPEND_SNAPSHOT_NUM=1
             ;;
         u)
             IS_FOR_UBUNTU=1
             ;;
         d)
             APPEND_LINUX_DISTRO=$OPTARG
             ;;
         c)
             VALUE_EXTRA_CMAKE_PARAMS=$OPTARG
             ;;
         h)
             SKIP_HEAVY_DOCS=1
             ;;
         ?)
             echo "Unknown command line argument!"
             exit 1
             ;;
     esac
done

# Call prepare_release, which also detects MRPT version and exports it
# in MRPT_VERSION_STR, etc.
if [ -f version_prefix.txt ];
then
	MRPTSRC=`pwd`
	source scripts/prepare_release.sh
	echo
	echo "## Done prepare_release.sh"
	echo "=========== Generating MRPT ${MRPT_VER_MMP} Debian package =============="
	cd $MRPTSRC
else
	echo "ERROR: Run this script from the MRPT root directory."
	exit 1
fi

set -x
if [ -z "$MRPT_DEB_DIR" ]; then
        MRPT_DEB_DIR="$HOME/mrpt_debian"
fi
MRPT_EXTERN_DEBIAN_DIR="$MRPTSRC/packaging/debian/"

if [ -f ${MRPT_EXTERN_DEBIAN_DIR}/control.in ];
then
	echo "Using debian dir: ${MRPT_EXTERN_DEBIAN_DIR}"
else
	echo "ERROR: Cannot find ${MRPT_EXTERN_DEBIAN_DIR}"
	exit 1
fi

# Append snapshot?
if [ $APPEND_SNAPSHOT_NUM == "1" ];
then
	MRPT_SNAPSHOT_VERSION=`date +%Y%m%d-%H%M`
	MRPT_SNAPSHOT_VERSION+="-git-"
	MRPT_SNAPSHOT_VERSION+=`git rev-parse --short=8 HEAD`
	MRPT_SNAPSHOT_VERSION+="-"
	MRPT_VERSION_STR="${MRPT_VERSION_STR}~snapshot${MRPT_SNAPSHOT_VERSION}${APPEND_LINUX_DISTRO}"
else
	MRPT_VERSION_STR="${MRPT_VERSION_STR}${APPEND_LINUX_DISTRO}"
fi

MRPT_DEBSRC_DIR=$MRPT_DEB_DIR/mrpt-${MRPT_VERSION_STR}

echo "MRPT_VERSION_STR: ${MRPT_VERSION_STR}"
echo "MRPT_DEBSRC_DIR: ${MRPT_DEBSRC_DIR}"

# Prepare a directory for building the debian package:
#
rm -fR $MRPT_DEB_DIR || true
mkdir -p $MRPT_DEB_DIR

# Orig tarball:
echo "Copying orig tarball: mrpt_${MRPT_VERSION_STR}.orig.tar.gz"
cp $HOME/mrpt_release/mrpt*.tar.gz $MRPT_DEB_DIR/mrpt_${MRPT_VERSION_STR}.orig.tar.gz
cd ${MRPT_DEB_DIR}
tar -xf mrpt_${MRPT_VERSION_STR}.orig.tar.gz

if [ ! -f "${MRPT_DEBSRC_DIR}/CMakeLists.txt" ];
then
	echo "*ERROR*: Seems there was a problem copying sources to ${MRPT_DEBSRC_DIR}... aborting script."
	exit 1
fi

cd ${MRPT_DEBSRC_DIR}

# Copy debian directory:
mkdir debian
cp -r ${MRPT_EXTERN_DEBIAN_DIR}/* debian

# Export signing pub key:
mkdir debian/upstream/
gpg --export --export-options export-minimal --armor > debian/upstream/signing-key.asc

# Parse debian/ control.in --> control
mv debian/control.in debian/control
sed -i "s/@MRPT_VER_MM@/${MRPT_VER_MM}/g" debian/control

# Replace the text "REPLACE_HERE_EXTRA_CMAKE_PARAMS" in the "debian/rules" file
# with: ${${VALUE_EXTRA_CMAKE_PARAMS}}
RULES_FILE=debian/rules
sed -i -e "s/REPLACE_HERE_EXTRA_CMAKE_PARAMS/${VALUE_EXTRA_CMAKE_PARAMS}/g" $RULES_FILE
echo "Using these extra parameters for CMake: '${VALUE_EXTRA_CMAKE_PARAMS}'"

# To avoid timeout compiling in ARM build farms, skip building heavy docs:
if [ ${SKIP_HEAVY_DOCS} == "1" ];
then
	sed -i "/documentation_html/d" $RULES_FILE
	sed -i "/documentation_performance_html/d" $RULES_FILE
	sed -i "/documentation_psgz_guides/d" $RULES_FILE
fi

# Strip my custom files...
rm debian/*.new || true
# debian/source file issues for old Ubuntu distros:
if [ $IS_FOR_UBUNTU == "1" ];
then
	rm -fr debian/source
fi

# Prepare install files:
# For each library, create its "<lib>.install" file:
cd libs
LST_LIBS=$(ls -d */);   # List only directories
for lib in $LST_LIBS;
do
	lib=${lib%/}  # Remove the trailing "/"
	echo "usr/lib/libmrpt-${lib}.so.${MRPT_VER_MM}"   > ../debian/libmrpt-${lib}${MRPT_VER_MM}.install
	echo "usr/lib/libmrpt-${lib}.so.${MRPT_VER_MMP}" >> ../debian/libmrpt-${lib}${MRPT_VER_MM}.install
done
cd .. # Back to MRPT root

# Figure out the next Debian version number:
echo "Detecting next Debian version number..."

CHANGELOG_UPSTREAM_VER=$( dpkg-parsechangelog | sed -n 's/Version:.*\([0-9]\.[0-9]*\.[0-9]*.*snapshot.*\)-.*/\1/p' )
CHANGELOG_LAST_DEBIAN_VER=$( dpkg-parsechangelog | sed -n 's/Version:.*\([0-9]\.[0-9]*\.[0-9]*\).*-\([0-9]*\).*/\2/p' )

echo " -> PREVIOUS UPSTREAM: $CHANGELOG_UPSTREAM_VER -> New: ${MRPT_VERSION_STR}"
echo " -> PREVIOUS DEBIAN VERSION: $CHANGELOG_LAST_DEBIAN_VER"

# If we have the same upstream versions, increase the Debian version, otherwise create a new entry:
if [ "$CHANGELOG_UPSTREAM_VER" = "$MRPT_VERSION_STR" ];
then
	NEW_DEBIAN_VER=$[$CHANGELOG_LAST_DEBIAN_VER + 1]
	echo "Changing to a new Debian version: ${MRPT_VERSION_STR}-${NEW_DEBIAN_VER}"
	DEBCHANGE_CMD="--newversion 1:${MRPT_VERSION_STR}-${NEW_DEBIAN_VER}"
else
	DEBCHANGE_CMD="--newversion 1:${MRPT_VERSION_STR}-1"
fi

echo "Adding a new entry to debian/changelog..."

DEBEMAIL="José Luis Blanco Claraco <joseluisblancoc@gmail.com>" debchange $DEBCHANGE_CMD -b --distribution unstable --force-distribution New version of upstream sources.

echo "Copying back the new changelog to a temporary file in: ${MRPT_EXTERN_DEBIAN_DIR}changelog.new"
cp debian/changelog ${MRPT_EXTERN_DEBIAN_DIR}changelog.new

set +x

echo "=============================================================="
echo "Now, you can build the source Deb package with 'debuild -S -sa'"
echo "=============================================================="

cd ..
ls -lh

exit 0
