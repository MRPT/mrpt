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
  else
    BUILD_EXAMPLES=TRUE
  fi

  cmake $MRPT_DIR -DBUILD_EXAMPLES=$BUILD_EXAMPLES -DBUILD_APPLICATIONS=TRUE -DBUILD_TESTING=FALSE
  make -j2
}

function test ()
{
  mkdir $BUILD_DIR && cd $BUILD_DIR
  cmake $MRPT_DIR -DBUILD_APPLICATIONS=FALSE
  make test
}

function doc ()
{
  echo doc placeholder
}

case $TASK in
  build ) build;;
  test ) test;;
  testhwdrivers ) testhwdrivers;;
  doc ) doc;;
esac
