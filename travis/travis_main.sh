#!/bin/bash
set -e   # Make sure any error makes the script to return an error code

if [ "$CC" == "gcc" ]; then
  export CC=gcc-7
  export CXX=g++-7
fi

if [ "$CC" == "clang" ]; then
  export CC=clang-6.0
  export CXX=clang++-6.0
fi

ORIG_MRPT_DIR=`pwd`
MRPT_DIR=/mrpt
BUILD_DIR=/build

CMAKE_C_FLAGS="-Wall -Wextra -Wabi"
CMAKE_CXX_FLAGS="-Wall -Wextra -Wabi"
EXTRA_CMAKE_ARGS="-DDISABLE_PCL=ON"  # PCL causes link errors (?!)

function install_lint_reqs()
{
  pip install -r travis/python_reqs.txt
}

# arguments: extra CMAKE flags to append to common ones
function do_generate_makefile()
{
  # prepare_build_dir
  # Make sure we dont have spurious files:
  cd $ORIG_MRPT_DIR
  git clean -fd || true
  rsync -av --exclude=.git $ORIG_MRPT_DIR /
  cd $MRPT_DIR

  rm -fr $BUILD_DIR || true
  mkdir -p $BUILD_DIR
  cd $BUILD_DIR

  if [ "$DEPS" == "minimal" ]; then
    DOWNLOAD_EIGEN_FLAG="-DEIGEN_USE_EMBEDDED_VERSION=ON"
  else
    DOWNLOAD_EIGEN_FLAG=""
  fi

  # configure cmake:
  VERBOSE=1 cmake $MRPT_DIR \
    -DCMAKE_BUILD_TYPE=${BUILD_TYPE} \
    $EXTRA_CMAKE_ARGS $DOWNLOAD_EIGEN_FLAG $@
}

function build ()
{
  if [ "$DEPS" == "minimal" ]; then
    DISABLE_PYTHON_BINDINGS=ON
  else
    DISABLE_PYTHON_BINDINGS=OFF
  fi

  #don't regenerate makefiles on stage 2
  if [ "$STAGE" != "2" ]; then
    do_generate_makefile \
      -DBUILD_EXAMPLES=$BUILD_EXAMPLES \
      -DBUILD_TESTING=FALSE \
      -DDISABLE_PYTHON_BINDINGS=$DISABLE_PYTHON_BINDINGS
  fi

  cd $BUILD_DIR

  if [ "$STAGE" == "1" ]; then
    make -j2 $BUILD_TARGET
  else
    make -j2
  fi

  cd $MRPT_DIR
}

command_exists () {
    type "$1" &> /dev/null ;
}

function test ()
{
  # gcc is too slow and we have a time limit in Travis CI:
  if [ "$CC" == "gcc" ] && [ "$TRAVIS_OS_NAME" == "osx" ]; then
    return
  fi

  do_generate_makefile -DBUILD_APPLICATIONS=FALSE -DENABLE_COVERAGE=On

  # Remove gdb use for coverage test reports.
  # Use `test_gdb` to show stack traces of failing unit tests.
#  if command_exists gdb ; then
#    make test_gdb
#  else
    make -j2 tests_build_all
    make -j2 test
    make -j2 gcov
#  fi
  bash <(curl -s https://codecov.io/bash) -X gcov

  cd $MRPT_DIR
}

case $TASK in
  build ) build;;
  test ) test;;
  lint ) install_lint_reqs;;
esac
