#!/bin/bash
# Copies sources from SVN tree and delete some in-work projects, for preparing a public release.
# JLBC, Aug 2008

#set -o verbose # echo on
set +o verbose # echo off

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
MRPT_DEB_DIR="$HOME/mrpt_release"

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
	git archive --format=tar master | tar -x -C ${MRPT_DEBSRC_DIR}

	# Generate ./SOURCE_DATE_EPOCH with UNIX time_t
	SOURCE_DATE_EPOCH=$(git log -1 --pretty=%ct)
else
	echo "Copying sources to ${MRPT_DEBSRC_DIR}"
	cp -R . ${MRPT_DEBSRC_DIR}

	# Generate ./SOURCE_DATE_EPOCH with UNIX time_t
	SOURCE_DATE_EPOCH=$(date +%s)
fi

echo $SOURCE_DATE_EPOCH > ${MRPT_DEBSRC_DIR}/SOURCE_DATE_EPOCH

# Copy the MRPT book:
if [ -f /Work/MyBooks/mrpt-book/mrpt-book.ps ];
then
	cp  /Work/MyBooks/mrpt-book/mrpt-book.ps ${MRPT_DEBSRC_DIR}/doc/
	ps2pdf ${MRPT_DEBSRC_DIR}/doc/mrpt-book.ps ${MRPT_DEBSRC_DIR}/doc/mrpt-book.pdf
	gzip ${MRPT_DEBSRC_DIR}/doc/mrpt-book.ps
fi

# Try to compile guides now:
for guide in {pbmap-guide,graphslam-engine-guide}; do
    make -C $MRPTSRC/doc/$guide/
    if [ -f $MRPTSRC/doc/$guide/$guide.pdf ];
    then
	    cp $MRPTSRC/doc/$guide/$guide.pdf ${MRPT_DEBSRC_DIR}/doc/
    fi
done

#printf "Generating mrpt.spec ..."
#eval "echo \"`cat mrpt.spec.in`\"" > ${MRPT_DEBSRC_DIR}/mrpt.spec
#printf "OK\n"


cd ${MRPT_DEBSRC_DIR}
echo "Deleting some files..."

# Deletions:
rm -fR lib
rm -fR packaging

# Not stable yet...
rm -fR apps/hmt-slam
rm -fR apps/hmt-slam-gui
rm -fR apps/hmtMapViewer

# Orig tarball:
cd ..
echo "Creating orig tarball: mrpt-${MRPT_VERSION_STR}.tar.gz"
tar czf mrpt-${MRPT_VERSION_STR}.tar.gz mrpt-${MRPT_VERSION_STR}

# Create .zip file with DOS line endings
echo "Creating orig zip in DOS format: mrpt-${MRPT_VERSION_STR}.zip"
cd mrpt-${MRPT_VERSION_STR}
bash scripts/all_files_DOS_format.sh
cd ..
zip mrpt-${MRPT_VERSION_STR}.zip -q -r mrpt-${MRPT_VERSION_STR}/*

rm -fr mrpt-${MRPT_VERSION_STR}

exit 0

