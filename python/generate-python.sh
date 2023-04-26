#!/bin/bash
# Usage: ./generate-python.sh
#
# Based on https://github.com/RosettaCommons/binder
#

PYBIND11_VERSION=$(dpkg -s pybind11-dev | grep '^Version:' | cut -d " " -f2)
SYSTEM_PYBIND11_MM_VERSION=$(echo $PYBIND11_VERSION | cut -d. -f1).$(echo $PYBIND11_VERSION | cut -d. -f2)

PYBIND11_MM_VERSION=${PYBIND11_MM_VERSION:-$SYSTEM_PYBIND11_MM_VERSION}

echo "System PYBIND11_VERSION: $PYBIND11_VERSION (Used for wrapper: $PYBIND11_MM_VERSION)"

MODULE_NAME=mrpt

mkdir -p generated-sources-pybind

$HOME/code/binder/build/source/binder \
	--root-module=pymrpt \
	--prefix generated-sources-pybind/ \
	--bind pymrpt \
	-config ./python.conf \
	./all_wrapped_mrpt_headers.hpp \
	-- \
	-iwithsysroot/usr/include/c++/11/ \
	-iwithsysroot/usr/include/x86_64-linux-gnu/c++/11/ \
	-std=c++17 -DNDEBUG \
	-I$HOME/code/mrpt/build-Release/include/mrpt-configuration/ \
	-I$HOME/code/mrpt/build-Release/3rdparty/nanogui/ \
	-I/usr/include/eigen3 \
	-I$HOME/code/mrpt/3rdparty/nanoflann/include/ \
	-I$HOME/code/mrpt/3rdparty/nanogui/ext/nanovg/src/ \
	-I$HOME/code/mrpt/3rdparty/nanogui/include/ \
	-I$HOME/code/mrpt/libs/apps/include/ \
	-I$HOME/code/mrpt/libs/bayes/include/ \
	-I$HOME/code/mrpt/libs/comms/include/ \
	-I$HOME/code/mrpt/libs/config/include/ \
	-I$HOME/code/mrpt/libs/containers/include/ \
	-I$HOME/code/mrpt/libs/core/include/ \
	-I$HOME/code/mrpt/libs/expr/include/ \
	-I$HOME/code/mrpt/libs/gui/include/ \
	-I$HOME/code/mrpt/libs/hwdrivers/include/ \
	-I$HOME/code/mrpt/libs/img/include/ \
	-I$HOME/code/mrpt/libs/io/include/ \
	-I$HOME/code/mrpt/libs/kinematics/include/ \
	-I$HOME/code/mrpt/libs/maps/include/ \
	-I$HOME/code/mrpt/libs/math/include/ \
	-I$HOME/code/mrpt/libs/nav/include/ \
	-I$HOME/code/mrpt/libs/obs/include/ \
	-I$HOME/code/mrpt/libs/opengl/include/ \
	-I$HOME/code/mrpt/libs/poses/include/ \
	-I$HOME/code/mrpt/libs/random/include/ \
	-I$HOME/code/mrpt/libs/rtti/include/ \
	-I$HOME/code/mrpt/libs/serialization/include/ \
	-I$HOME/code/mrpt/libs/serialization/include/ \
	-I$HOME/code/mrpt/libs/slam/include/ \
	-I$HOME/code/mrpt/libs/graphs/include/ \
	-I$HOME/code/mrpt/libs/system/include/ \
	-I$HOME/code/mrpt/libs/tfest/include \
	-I$HOME/code/mrpt/libs/topography/include \
	-I$HOME/code/mrpt/libs/typemeta/include \
	-I$HOME/code/mrpt/libs/vision/include/ \

# Enforce formatting:
#find generated-sources-pybind -name "*.cpp" | xargs -I FIL clang-format-11 -i FIL
