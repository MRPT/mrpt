#!/bin/bash
# Usage: ./generate-python.sh
#
# Based on https://github.com/RosettaCommons/binder
#
# binder config: llvm-14
#


PYBIND11_VERSION=$(dpkg -s pybind11-dev | grep '^Version:' | cut -d " " -f2)
SYSTEM_PYBIND11_MM_VERSION=$(echo $PYBIND11_VERSION | cut -d. -f1).$(echo $PYBIND11_VERSION | cut -d. -f2)

PYBIND11_MM_VERSION=${PYBIND11_MM_VERSION:-$SYSTEM_PYBIND11_MM_VERSION}

echo "System PYBIND11_VERSION: $PYBIND11_VERSION (Used for wrapper: $PYBIND11_MM_VERSION)"

MODULE_NAME=mrpt
WRAP_OUT_DIR=src

mkdir -p $WRAP_OUT_DIR

$HOME/code/binder/build/source/binder \
	--root-module=pymrpt \
	--prefix $WRAP_OUT_DIR/ \
	--bind pymrpt \
	-config ./python.conf \
	./all_wrapped_mrpt_headers.hpp \
	-- \
	-iwithsysroot/usr/include/c++/11/ \
	-iwithsysroot/usr/include/x86_64-linux-gnu/c++/11/ \
	-std=c++17 -DNDEBUG \
	-DMRPT_BUILDING_PYTHON_WRAPPER \
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

# Workarounds to binder limitations:
# These are to ensure multiplatform portatbility of generated code
# (e.g. avoid build errors in armhf)
# -----------------------------------------------------------------------------
# Replace:
# struct std::chrono::time_point<class mrpt::Clock, struct std::chrono::duration<long, struct std::ratio<1, 10000000> > >
# mrpt::Clock::time_point
find $WRAP_OUT_DIR -name "*.cpp" | 	xargs -I FIL \
	sed -i -e 's/struct std::chrono::time_point<class mrpt::Clock, struct std::chrono::duration<long, struct std::ratio<1, 10000000> > >/mrpt::Clock::time_point/g' FIL
find $WRAP_OUT_DIR -name "*.cpp" | 	xargs -I FIL \
	sed -i -e 's/std::chrono::time_point<mrpt::Clock,std::chrono::duration<long, std::ratio<1, 10000000> >>/mrpt::Clock::time_point/g' FIL

find $WRAP_OUT_DIR -name "*.cpp" | 	xargs -I FIL \
	sed -i -e 's/std::chrono::duration<long, struct std::ratio<1, 10000000> >/std::chrono::duration<int64_t,struct std::ratio<1,10000000>>/g' FIL

find $WRAP_OUT_DIR -name "*.cpp" | 	xargs -I FIL \
	sed -i -e 's/std::chrono::duration<long,/std::chrono::duration<int64_t,/g' FIL

sed -i -e 's/unsigned long/size_t/g' $WRAP_OUT_DIR/std/array.cpp
sed -i -e 's/unsigned long/size_t/g' $WRAP_OUT_DIR/std/stl_multimap.cpp

# (long)
# (int64_t)
find $WRAP_OUT_DIR -name "*.cpp" | 	xargs -I FIL \
	sed -i -e 's/(long)/(int64_t)/g' FIL

# applying manual patches:
echo "Applying manual patches to pybind11 code..."
find . -name "patch-0*.diff" | sort | xargs -I FIL bash -c "echo \"Applying patch: FIL\" && git apply FIL --ignore-whitespace"

