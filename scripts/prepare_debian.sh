#!/bin/bash
# Copies sources from SVN tree and delete windows-only files, for preparing a Debian package.
# JLBC, 2008-2010

#set -o verbose # echo on
set +o verbose # echo off

APPEND_SVN_NUM=0
IS_FOR_UBUNTU=0
APPEND_LINUX_DISTRO=""
while getopts "sud:" OPTION
do
     case $OPTION in
         s)
             APPEND_SVN_NUM=1
             ;;
         u)
             IS_FOR_UBUNTU=1
             ;;
         d)
             APPEND_LINUX_DISTRO=$OPTARG
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
MRPT_DEB_DIR="$HOME/mrpt_debian"
MRPT_EXTERN_DEBIAN_DIR="$MRPTSRC/packaging/debian/"

if [ -f ${MRPT_EXTERN_DEBIAN_DIR}/control ];
then
	echo "Using debian dir: ${MRPT_EXTERN_DEBIAN_DIR}"
else
	echo "ERROR: Cannot find ${MRPT_EXTERN_DEBIAN_DIR}"
	exit 1
fi

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

	if [ $APPEND_SVN_NUM == "1" ];
	then
		MRPT_VERSION_STR="${MRPT_VERSION_STR}svn${MRPT_SVN_VERSION}${APPEND_LINUX_DISTRO}"
	else
		MRPT_VERSION_STR="${MRPT_VERSION_STR}${APPEND_LINUX_DISTRO}"
	fi

	echo "Exporting to $MRPT_DEB_DIR/mrpt-${MRPT_VERSION_STR}"
	svn export . $MRPT_DEB_DIR/mrpt-${MRPT_VERSION_STR}
fi

# Copy dummy "configure" script:
cp scripts/configure $MRPT_DEB_DIR/mrpt-${MRPT_VERSION_STR}/

# Copy the MRPT book:
if [ -f /Trabajo/Papers/mrpt-book/mrpt-book.ps ];
then
	cp  /Trabajo/Papers/mrpt-book/mrpt-book.ps $MRPT_DEB_DIR/mrpt-${MRPT_VERSION_STR}/doc/
	ps2pdf $MRPT_DEB_DIR/mrpt-${MRPT_VERSION_STR}/doc/mrpt-book.ps $MRPT_DEB_DIR/mrpt-${MRPT_VERSION_STR}/doc/mrpt-book.pdf
	gzip $MRPT_DEB_DIR/mrpt-${MRPT_VERSION_STR}/doc/mrpt-book.ps
fi


cd $MRPT_DEB_DIR/mrpt-${MRPT_VERSION_STR}
echo "Deleting Windows-only and not required files for Debian packages..."

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

rm -fR scripts/Hha.dll scripts/hhc.exe scripts/prepare_*.sh scripts/recompile*

rm -fR doc/papers


# And remove the corrs. lines:
#(echo "g/console2gui/d"; echo 'wq') | ex -s apps/CMakeLists.txt


# Orig tarball:
cd ..
echo "Creating orig tarball: mrpt_${MRPT_VERSION_STR}.orig.tar.gz"
tar czf mrpt_${MRPT_VERSION_STR}.orig.tar.gz mrpt-${MRPT_VERSION_STR}

# Copy debian directory:
mkdir mrpt-${MRPT_VERSION_STR}/debian
cp -r ${MRPT_EXTERN_DEBIAN_DIR}/* mrpt-${MRPT_VERSION_STR}/debian
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

CHANGELOG_UPSTREAM_VER=$( dpkg-parsechangelog | sed -n 's/Version:.*\([0-9]\.[0-9]*\.[0-9]*.*svn.*\)-.*/\1/p' )
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

