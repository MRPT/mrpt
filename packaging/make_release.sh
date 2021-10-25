#!/bin/bash
#
# Export sources from a git tree (including SOME of the git submodules) and
# prepare it for a public release:
# - tar.gz, .zip: source code, with Unix / Windows line feeds.
# - gpg signature.
# - Creates SOURCE_DATE_EPOCH. Read https://reproducible-builds.org/docs/source-date-epoch/
# - Compiles the guides into .ps.gz. See list in MRPT/doc/guide-list.txt
#
# JLBC, 2008-2021

set -e  # exit on error

# Sets the bash variables:
# MRPT_VERSION_STR, MRPT_VER_MMP, MRPT_VERSION_{MAJOR,MINOR,PATCH}
# --------------------------------
if [ -f version_prefix.txt ];
then
	MRPT_VERSION_STR=$(head -n 1 version_prefix.txt)
	MRPT_VERSION_MAJOR=${MRPT_VERSION_STR:0:1}
	MRPT_VERSION_MINOR=${MRPT_VERSION_STR:2:1}
	MRPT_VERSION_PATCH=${MRPT_VERSION_STR:4:1}
	MRPT_VER_MMP="${MRPT_VERSION_MAJOR}.${MRPT_VERSION_MINOR}.${MRPT_VERSION_PATCH}"
	echo "MRPT version: ${MRPT_VER_MMP}"
else
	echo "Error: cannot find version_prefix.txt!! Invoke scripts from mrpt sources root directory."
	exit 1
fi

MRPTSRC=$(pwd)
OUT_RELEASES_DIR="$HOME/mrpt_release"

OUT_DIR=$OUT_RELEASES_DIR/mrpt-${MRPT_VERSION_STR}

echo "=========== Generating MRPT release ${MRPT_VER_MMP} =================="
echo "MRPT_VERSION_STR   : ${MRPT_VERSION_STR}"
echo "OUT_DIR            : ${OUT_DIR}"
echo "============================================================"
echo

# Prepare output directory:
rm -fR "$OUT_RELEASES_DIR" || true
mkdir -p "${OUT_DIR}"

# Export / copy sources to target dir:
if [ -d "$MRPTSRC/.git" ];
then
	echo "> Exporting git source tree to ${OUT_DIR}"
	git archive --format=tar HEAD | tar -x -C "${OUT_DIR}"

	# Include external submodules:
	EXTERNAL_MODS="nanogui nanogui/ext/nanovg googletest libfyaml rplidar_sdk ${MRPT_PKG_EXPORTED_SUBMODULES}"
	for MOD in $EXTERNAL_MODS;
	do
		echo "> Exporting git submodule: ${MRPTSRC}/3rdparty/$MOD"
		cd "${MRPTSRC}/3rdparty/$MOD"
		git archive --format=tar HEAD | tar -x -C "${OUT_DIR}/3rdparty/$MOD"
	done

	cd "${MRPTSRC}"

	# Remove VCS control files:
	find "${OUT_DIR}" -name '.git*' -delete

	# Generate ./SOURCE_DATE_EPOCH with UNIX time_t
	SOURCE_DATE_EPOCH=$(git log -1 --pretty=%ct)
else
	echo "> Copying sources to ${OUT_DIR}"
	cp -R . "${OUT_DIR}"

	# Generate ./SOURCE_DATE_EPOCH with UNIX time_t
	SOURCE_DATE_EPOCH=$(date +%s)
fi

# See https://reproducible-builds.org/specs/source-date-epoch/
echo "$SOURCE_DATE_EPOCH" > "${OUT_DIR}/SOURCE_DATE_EPOCH"

# Compile guides now:
LST_GUIDES=$(cat "$MRPTSRC/doc/guide-list.txt")
echo "> Building LaTeX documents..."
for guide in $LST_GUIDES; do
	echo ">  * Building $guide..."
	make -C "$MRPTSRC/doc/$guide/" > /tmp/mrpt_build_latex.log 2>&1
	cp "$MRPTSRC/doc/$guide/$guide.pdf" "${OUT_DIR}/doc/"
done

# Orig tarball:
cd "${OUT_DIR}/.."

echo "> Creating source tarball: mrpt-${MRPT_VERSION_STR}.tar.gz"
tar czf "mrpt-${MRPT_VERSION_STR}.tar.gz" "mrpt-${MRPT_VERSION_STR}"

# Create .zip file with DOS line endings
echo "> Creating source zip in DOS format: mrpt-${MRPT_VERSION_STR}.zip"

cd "${OUT_DIR}"
bash scripts/all_files_change_format.sh todos

cd "${OUT_DIR}/.."
zip "mrpt-${MRPT_VERSION_STR}.zip" -q -r "mrpt-${MRPT_VERSION_STR}/*"

rm -fr "mrpt-${MRPT_VERSION_STR}"

# GPG signature:
gpg --armor --detach-sign "mrpt-${MRPT_VERSION_STR}.tar.gz"


echo "=================================================================="
echo "                     Package files generated:"
echo "=================================================================="
find "$(pwd)"
echo "=================================================================="
echo "You should also do now: \"git clean -fd\" in the source code root"
