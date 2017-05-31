#!/bin/bash
# Copies sources from source tree and delete windows-only files, for preparing a Debian package.
# JLBC, 2008-2010

#set -o verbose # echo on
set +o verbose # echo off

APPEND_SNAPSHOT_NUM=0
IS_FOR_UBUNTU=0
LEAVE_EMBEDDED_EIGEN=0
SKIP_HEAVY_DOCS=0
APPEND_LINUX_DISTRO=""
VALUE_EXTRA_CMAKE_PARAMS=""
while getopts "suhed:c:" OPTION
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
         e)
             LEAVE_EMBEDDED_EIGEN=1
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

# Checks
# --------------------------------
if [ -f version_prefix.txt ];
then
	MRPT_VERSION_STR=`head -n 1 version_prefix.txt`
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
MRPT_SNAPSHOT_VERSION=`date +%Y%m%d-%H%M`
MRPT_SNAPSHOT_VERSION+="-git-"
MRPT_SNAPSHOT_VERSION+=`git rev-parse --short=8 HEAD`
MRPT_SNAPSHOT_VERSION+="-"

if [ $APPEND_SNAPSHOT_NUM == "1" ];
then
	MRPT_VERSION_STR="${MRPT_VERSION_STR}~snapshot${MRPT_SNAPSHOT_VERSION}${APPEND_LINUX_DISTRO}"
else
	MRPT_VERSION_STR="${MRPT_VERSION_STR}${APPEND_LINUX_DISTRO}"
fi

MRPT_DEBSRC_DIR=$MRPT_DEB_DIR/mrpt-${MRPT_VERSION_STR}

echo "MRPT_VERSION_STR: ${MRPT_VERSION_STR}"
echo "MRPT_DEBSRC_DIR: ${MRPT_DEBSRC_DIR}"

# Prepare a directory for building the debian package:
# 
rm -fR $MRPT_DEB_DIR
mkdir -p ${MRPT_DEBSRC_DIR}


# Export / copy sources to target dir:
if [ -d "$MRPTSRC/.git" ];
then
	echo "Exporting git source tree to ${MRPT_DEBSRC_DIR}"
	git archive  --format=tar HEAD | tar -x -C ${MRPT_DEBSRC_DIR}

	# Generate ./SOURCE_DATE_EPOCH with UNIX time_t
	SOURCE_DATE_EPOCH=$(git log -1 --pretty=%ct)
else
	echo "Copying sources to ${MRPT_DEBSRC_DIR}"
	cp -R . ${MRPT_DEBSRC_DIR}

	# Generate ./SOURCE_DATE_EPOCH with UNIX time_t
	SOURCE_DATE_EPOCH=$(date +%s)
fi

echo $SOURCE_DATE_EPOCH > ${MRPT_DEBSRC_DIR}/SOURCE_DATE_EPOCH

if [ ! -f "${MRPT_DEBSRC_DIR}/CMakeLists.txt" ];
then
	echo "*ERROR*: Seems there was a problem copying sources to ${MRPT_DEBSRC_DIR}... aborting script."
	exit 1
fi

cd ${MRPT_DEBSRC_DIR}
echo "Deleting Windows-only and not required files for Debian packages..."

# Deletions:
rm -fR lib
rm -fR packaging

# Not stable...
rm -fR apps/hmt-slam
rm -fR apps/hmtMapViewer

rm -fR scripts/Hha.dll scripts/hhc.exe scripts/prepare_*.sh scripts/recompile*

find . -name '.gitignore' | xargs rm 

if [ ${LEAVE_EMBEDDED_EIGEN} == "0" ];
then
	# Normal for Debian pkgs: remove embedded copy of Eigen
	rm -fR otherlibs/eigen3/
fi

# Don't use embedded version in Debian pkgs: use system pkgs.
rm -fR otherlibs/assimp


# Orig tarball:
cd ..
echo "Creating orig tarball: mrpt_${MRPT_VERSION_STR}.orig.tar.gz"
tar czf mrpt_${MRPT_VERSION_STR}.orig.tar.gz mrpt-${MRPT_VERSION_STR}

echo "LEAVE_EMBEDDED_EIGEN=${LEAVE_EMBEDDED_EIGEN}"

# Copy debian directory:
mkdir mrpt-${MRPT_VERSION_STR}/debian
cp -r ${MRPT_EXTERN_DEBIAN_DIR}/* mrpt-${MRPT_VERSION_STR}/debian

# Parse debian/ control.in --> control
mv mrpt-${MRPT_VERSION_STR}/debian/control.in mrpt-${MRPT_VERSION_STR}/debian/control
sed -i "s/@MRPT_VER_MM@/${MRPT_VER_MM}/g" mrpt-${MRPT_VERSION_STR}/debian/control 


if [ ${LEAVE_EMBEDDED_EIGEN} == "1" ];
then
	# 2) ... and relax the dependency on libeigen3-dev
	sed -i 's/libeigen3-dev ,/libeigen3-dev | perl ,/g' mrpt-${MRPT_VERSION_STR}/debian/control
	sed -i 's/libeigen3-dev,//g' mrpt-${MRPT_VERSION_STR}/debian/control
fi

# Replace the text "REPLACE_HERE_EXTRA_CMAKE_PARAMS" in the "debian/rules" file
# with: ${${VALUE_EXTRA_CMAKE_PARAMS}}
RULES_FILE=mrpt-${MRPT_VERSION_STR}/debian/rules
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
rm mrpt-${MRPT_VERSION_STR}/debian/*.new 
# debian/source file issues for old Ubuntu distros:
if [ $IS_FOR_UBUNTU == "1" ];
then
	rm -fr mrpt-${MRPT_VERSION_STR}/debian/source
fi


# Prepare install files:
cd mrpt-${MRPT_VERSION_STR}

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
echo DEBEMAIL="Jose Luis Blanco (University of Malaga) <joseluisblancoc@gmail.com>" debchange $DEBCHANGE_CMD --distribution unstable --force-distribution New version of upstream sources.

DEBEMAIL="Jose Luis Blanco (University of Malaga) <joseluisblancoc@gmail.com>" debchange $DEBCHANGE_CMD -b --distribution unstable --force-distribution New version of upstream sources.

echo "Copying back the new changelog to a temporary file in: ${MRPT_EXTERN_DEBIAN_DIR}changelog.new"
cp debian/changelog ${MRPT_EXTERN_DEBIAN_DIR}changelog.new

set +o verbose # echo off

echo "Now, you can build the source Deb package with 'debuild -S -sa'"

cd ..
ls -lh


exit 0

