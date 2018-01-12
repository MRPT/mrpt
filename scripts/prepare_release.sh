#!/bin/bash
# Export sources from a git tree and prepare it for a public release.
# JLBC, 2008-2018

set -e  # exit on error

# Checks
# --------------------------------
if [ -f version_prefix.txt ];
then
	if [ -z ${MRPT_VERSION_STR+x} ];
	then
		# MRPT_VERSION_STR is not set by caller: load it
		MRPT_VERSION_STR=`head -n 1 version_prefix.txt`
	fi
	MRPT_VERSION_MAJOR=${MRPT_VERSION_STR:0:1}
	MRPT_VERSION_MINOR=${MRPT_VERSION_STR:2:1}
	MRPT_VERSION_PATCH=${MRPT_VERSION_STR:4:1}
	MRPT_VER_MM="${MRPT_VERSION_MAJOR}.${MRPT_VERSION_MINOR}"
	MRPT_VER_MMP="${MRPT_VERSION_MAJOR}.${MRPT_VERSION_MINOR}.${MRPT_VERSION_PATCH}"
else
	echo "ERROR: Run this script from the MRPT root directory."
	exit 1
fi

MRPTSRC=`pwd`
OUT_RELEASES_DIR="$HOME/mrpt_release"

OUT_DIR=$OUT_RELEASES_DIR/mrpt-${MRPT_VERSION_STR}

echo "=========== Generating MRPT release ${MRPT_VER_MMP} =================="
echo "MRPT_VERSION_STR   : ${MRPT_VERSION_STR}"
echo "OUT_DIR            : ${OUT_DIR}"
echo "============================================================"
echo

# Prepare output directory:
rm -fR $OUT_RELEASES_DIR  || true
mkdir -p ${OUT_DIR}

# Export / copy sources to target dir:
if [ -d "$MRPTSRC/.git" ];
then
	echo "# Exporting git source tree to ${OUT_DIR}"
	git archive --format=tar HEAD | tar -x -C ${OUT_DIR}

	# Remove VCS control files:
	find ${OUT_DIR} -name '.gitignore' | xargs rm

	# Generate ./SOURCE_DATE_EPOCH with UNIX time_t
	SOURCE_DATE_EPOCH=$(git log -1 --pretty=%ct)
else
	echo "# Copying sources to ${OUT_DIR}"
	cp -R . ${OUT_DIR}

	# Generate ./SOURCE_DATE_EPOCH with UNIX time_t
	SOURCE_DATE_EPOCH=$(date +%s)
fi

# See https://reproducible-builds.org/specs/source-date-epoch/
echo $SOURCE_DATE_EPOCH > ${OUT_DIR}/SOURCE_DATE_EPOCH

# Compile guides now:
LST_GUIDES=`cat $MRPTSRC/doc/guide-list.txt`
echo "# Building LaTeX documents..."
for guide in $LST_GUIDES; do
	echo "#  * Building $guide..."
	make -C $MRPTSRC/doc/$guide/ > /tmp/mrpt_build_latex.log 2>&1
	cp $MRPTSRC/doc/$guide/$guide.pdf ${OUT_DIR}/doc/
done

cd ${OUT_DIR}

# Patch for extra External Projects in old Ubuntu PPAs:
TMP_EXTRA_OTHERLIBS="/tmp/mrpt_release_extra_otherlibs.zip"
if [ ! -z "$MRPT_RELEASE_EXTRA_OTHERLIBS_URL" ];
then
	if [ ! -f "$TMP_EXTRA_OTHERLIBS" ];
	then
		wget $MRPT_RELEASE_EXTRA_OTHERLIBS_URL -O $TMP_EXTRA_OTHERLIBS
	fi
	cp $TMP_EXTRA_OTHERLIBS $MRPT_RELEASE_EXTRA_OTHERLIBS_PATH
fi

# Dont include Debian files in releases:
rm -fR packaging

# Orig tarball:
cd ..
echo "# Creating orig tarball: mrpt-${MRPT_VERSION_STR}.tar.gz"
tar czf mrpt-${MRPT_VERSION_STR}.tar.gz mrpt-${MRPT_VERSION_STR}

# Create .zip file with DOS line endings
echo "# Creating orig zip in DOS format: mrpt-${MRPT_VERSION_STR}.zip"
cd mrpt-${MRPT_VERSION_STR}
bash scripts/all_files_change_format.sh todos
cd ..
zip mrpt-${MRPT_VERSION_STR}.zip -q -r mrpt-${MRPT_VERSION_STR}/*

rm -fr mrpt-${MRPT_VERSION_STR}

# GPG signature:
gpg --armor --detach-sign mrpt-${MRPT_VERSION_STR}.tar.gz
