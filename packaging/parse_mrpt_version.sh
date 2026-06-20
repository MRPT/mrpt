#!/bin/bash
# Sets the bash variables:
# MRPT_VERSION_STR, MRPT_VER_MMP, MRPT_VER_MM, MRPT_VERSION_{MAJOR,MINOR,PATCH}
#
# The version is the single source of truth defined in the <version> tag of
# modules/mrpt_common/package.xml (the same file parsed by CMake).

# Resolve the repository root from this script's location (packaging/..),
# so it works regardless of the caller's current working directory:
_PARSE_VER_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
MRPT_ROOT_DIR="$( cd "${_PARSE_VER_DIR}/.." && pwd )"
MRPT_COMMON_PACKAGE_XML="${MRPT_ROOT_DIR}/modules/mrpt_common/package.xml"

if [ ! -f "${MRPT_COMMON_PACKAGE_XML}" ];
then
	echo "Error: cannot find ${MRPT_COMMON_PACKAGE_XML}!! Cannot determine MRPT version."
	exit 1
fi

if [ -z "${MRPT_VERSION_STR+x}" ];
then
	# MRPT_VERSION_STR is not set by caller: load it from package.xml
	MRPT_VERSION_STR=$(sed -n 's:.*<version>\([0-9.]*\)</version>.*:\1:p' "${MRPT_COMMON_PACKAGE_XML}" | head -n 1)
fi

MRPT_VERSION_MAJOR=$(echo "${MRPT_VERSION_STR}" | cut -d. -f1)
MRPT_VERSION_MINOR=$(echo "${MRPT_VERSION_STR}" | cut -d. -f2)
MRPT_VERSION_PATCH=$(echo "${MRPT_VERSION_STR}" | cut -d. -f3)
MRPT_VER_MMP="${MRPT_VERSION_MAJOR}.${MRPT_VERSION_MINOR}.${MRPT_VERSION_PATCH}"
MRPT_VER_MM="${MRPT_VERSION_MAJOR}.${MRPT_VERSION_MINOR}"
echo "MRPT version: ${MRPT_VER_MMP}"
