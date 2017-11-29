#!/bin/bash

set -e   # Make sure any error makes the script to return an error code
set -x   # echo

MRPT_DIR=`pwd`
BUILD_DIR=build

CMAKE_C_FLAGS="-Wall -Wextra -Wabi -O2"
CMAKE_CXX_FLAGS="-Wall -Wextra -Wabi -O2"

function build ()
{
  #env
  git clean -fr || true
  rm -fr $BUILD_DIR || true
  mkdir $BUILD_DIR && cd $BUILD_DIR
  
  # gcc is too slow and we have a time limit in Travis CI: exclude examples when building with gcc
  if [ "$CC" == "gcc" ]; then
    BUILD_EXAMPLES=FALSE
  else
    BUILD_EXAMPLES=TRUE
  fi

  cmake $MRPT_DIR -DBUILD_EXAMPLES=$BUILD_EXAMPLES -DBUILD_APPLICATIONS=TRUE -DBUILD_TESTING=FALSE
  make -j2

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

  git clean -fr || true
  rm -fr $BUILD_DIR || true
  mkdir $BUILD_DIR && cd $BUILD_DIR

  cmake $MRPT_DIR -DBUILD_APPLICATIONS=FALSE -DCMAKE_BUILD_TYPE=${BUILD_TYPE}
  # Remove gdb use for coverage test reports.
  # Use `test_gdb` to show stack traces of failing unit tests.
#  if command_exists gdb ; then
#    make test_gdb
#  else
    make test -j2
#  fi

  cd $MRPT_DIR
}

case $TASK in
  build ) build;;
  test ) test;;
esac
