#!/bin/bash
# Prepare to build a Debian package.
# JLBC, 2008-2018

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

source packaging/parse_mrpt_version.sh

# Append snapshot?
if [ $APPEND_SNAPSHOT_NUM == "1" ];
then
        CUR_SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
        source $CUR_SCRIPT_DIR/generate_snapshot_version.sh  # populate MRPT_SNAPSHOT_VERSION

        MRPT_VERSION_STR="${MRPT_VERSION_STR}~snapshot${MRPT_SNAPSHOT_VERSION}${APPEND_LINUX_DISTRO}"
else
        MRPT_VERSION_STR="${MRPT_VERSION_STR}${APPEND_LINUX_DISTRO}"
fi

# Call prepare_release, which also detects MRPT version and exports it
# in MRPT_VERSION_STR, etc.
if [ -f version_prefix.txt ];
then
	MRPTSRC=`pwd`

  # Do not reuse the sources since each Ubuntu version may require different sources...
  # Re-add after Bionic 18.04 EOL, which right now (2020) is a special case since
  # it misses libsimpleini-dev
  #
  #if [ -f $HOME/mrpt_release/mrpt*.tar.gz ];
  #then
  #  echo "## release file already exists. Reusing it."
  #else
    source packaging/prepare_release.sh
    echo
    echo "## Done prepare_release.sh"
  #fi
else
	echo "ERROR: Run this script from the MRPT root directory."
	exit 1
fi

echo "=========== Generating MRPT ${MRPT_VER_MMP} Debian package =============="
cd $MRPTSRC

set -x
if [ -z "$MRPT_DEB_DIR" ]; then
        MRPT_DEB_DIR="$HOME/mrpt_debian"
fi
MRPT_EXTERN_DEBIAN_DIR="$MRPTSRC/packaging/debian/"
MRPT_EXTERN_UBUNTU_PPA_DIR="$MRPTSRC/packaging/ubuntu-ppa/"

if [ -f ${MRPT_EXTERN_DEBIAN_DIR}/control.in ];
then
	echo "Using debian dir: ${MRPT_EXTERN_DEBIAN_DIR}"
else
	echo "ERROR: Cannot find ${MRPT_EXTERN_DEBIAN_DIR}"
	exit 1
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
cp $HOME/mrpt_release/mrpt*.tar.gz.asc $MRPT_DEB_DIR/mrpt_${MRPT_VERSION_STR}.orig.tar.gz.asc
cd ${MRPT_DEB_DIR}

# Was:
# tar -xf mrpt_${MRPT_VERSION_STR}.orig.tar.gz

set +x

echo "=================================================================="
echo "Now, you can build the Debian package using this tarball:"
echo " $(pwd)/mrpt_${MRPT_VERSION_STR}.orig.tar.gz"
echo ""
echo "You should also do now: \"git clean -fd\" in the source code root"
echo "=================================================================="

ls -lh

exit 0
