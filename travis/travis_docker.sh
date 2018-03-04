#!/bin/bash
set -e   # Make sure any error makes the script to return an error code


export DOCKER_IMAGE=mrpt/mrpt-build-env:$DIST
export STAGE1_OUTPUT_IMAGE=mrpt/mrpt-travis-archive:$DIST-$CC-$TRAVIS_COMMIT

if [ "$STAGE" == "2" ]; then
  export DOCKER_IMAGE=$STAGE1_OUTPUT_IMAGE
fi

docker run -v $HOME:$HOME $CODECOV_VARS -e STAGE -e BUILD_TYPE -e MAKEFLAGS -e CCACHE_SLOPPINESS -e TASK -e DEPS -e CI_SOURCE_PATH -e REPOSITORY_NAME -e HOME -e DISTRO -e CI_ROS_DISTRO -e CODECOV_TOKEN --name mrptbuild $DOCKER_IMAGE bash -c 'cd $CI_SOURCE_PATH; apt-get update -qq ; source travis/check_style.sh; source travis/travis_main.sh'

if [ "$STAGE" == "1" ]; then
  docker login -u="$DOCKER_USERNAME" -p="$DOCKER_PASSWORD"
  docker commit mrptbuild $STAGE1_OUTPUT_IMAGE
  docker push $STAGE1_OUTPUT_IMAGE
fi
