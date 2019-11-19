#!/bin/bash -e

# Start by getting aptitude,
# which has far better dependency handling than apt-get.
apt-get install aptitude

# Get several development tools and libraries.
aptitude -PvVR install \
    build-essential    \
    libjpeg-dev        \
    libpng-dev         \
    libtiff-dev        \
    liblapack-dev      \
    libv4l-dev         \
    mesa-common-dev    \
    libgl1-mesa-dev    \
    libglu1-mesa-dev   \
    mesa-utils         \
    freeglut3-dev      \
    ffmpeg             \
    libffms2-dev       \
    libavcodec-dev     \
    libavdevice-dev    \
    libavfilter-dev    \
    libavformat-dev    \
    libdc1394-22-dev   \

