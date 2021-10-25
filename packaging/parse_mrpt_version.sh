#!/bin/bash
# Sets the bash variables:
# MRPT_VERSION_STR, MRPT_VER_MMP, MRPT_VER_MM, MRPT_VERSION_{MAJOR,MINOR,PATCH}

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
    echo "MRPT version: ${MRPT_VER_MMP}"
else
	echo "Error: cannot find version_prefix.txt!! Invoke scripts from mrpt sources root directory."
	exit 1
fi
