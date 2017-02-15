#!/bin/sh

MRPT_DIR=`pwd`
BUILD_DIR=build

CMAKE_C_FLAGS="-Wall -Wextra -Wabi -O2"
CMAKE_CXX_FLAGS="-Wall -Wextra -Wabi -O2"

function build ()
{
  #env
  mkdir $BUILD_DIR && cd $BUILD_DIR

  # gcc is too slow and we have a time limit in Travis CI: exclude examples when building with gcc
  if [ "$CC" == "gcc" ]; then
    BUILD_EXAMPLES=FALSE
	BUILD_ARIA=FALSE
  else
    BUILD_EXAMPLES=TRUE
	BUILD_ARIA=ON
  fi

  cmake $MRPT_DIR -DBUILD_EXAMPLES=$BUILD_EXAMPLES -DBUILD_APPLICATIONS=TRUE -DBUILD_TESTING=FALSE -DBUILD_ARIA=$BUILD_ARIA
  make -j2
}

command_exists () {
    type "$1" &> /dev/null ;
}

function test ()
{
  mkdir $BUILD_DIR && cd $BUILD_DIR
  cmake $MRPT_DIR -DBUILD_APPLICATIONS=FALSE
  # Use `test_gdb` to show stack traces of failing unit tests.
  if command_exists gdb ; then
    make test_gdb
  else
    make test
  fi
}

function doc ()
{
  echo doc placeholder
}

case $TASK in
  build ) build;;
  test ) test;;
  doc ) doc;;
esac
