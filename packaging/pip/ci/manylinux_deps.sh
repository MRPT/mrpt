#!/bin/bash
# Build dependencies of the MRPT wheel inside the manylinux_2_28 images
# (AlmaLinux 8), run once per image by cibuildwheel ("before-all").
# Libraries not installed here (assimp, octomap, tinyxml2, nanoflann,
# simpleini...) use the versions embedded in the MRPT sources. GLFW is the one
# embedded in nanogui, built into libmrpt_nanogui. Eigen and oneTBB are built
# by the superbuild (CMakeLists.txt), into the SDK.
set -euo pipefail

dnf install -y epel-release
dnf install -y \
  ccache \
  zlib-devel libzstd-devel lz4-devel jsoncpp-devel \
  mesa-libGL-devel mesa-libEGL-devel libglvnd-devel \
  libX11-devel libXrandr-devel libXinerama-devel libXcursor-devel libXi-devel \
  libXxf86vm-devel
