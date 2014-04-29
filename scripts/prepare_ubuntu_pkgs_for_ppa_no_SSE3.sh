#!/bin/bash
# See the main script for more info:

export MRPT_PKG_CUSTOM_CMAKE_PARAMS="\"-DDISABLE_SSE3=ON -DDISABLE_SSE4=ON\""

# (from: http://stackoverflow.com/questions/59895/can-a-bash-script-tell-what-directory-its-stored-in )
MYDIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

/bin/bash $MYDIR/prepare_ubuntu_pkgs_for_ppa.sh

export MRPT_PKG_CUSTOM_CMAKE_PARAMS=""
