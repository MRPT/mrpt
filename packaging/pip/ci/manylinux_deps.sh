#!/bin/bash
# Build dependencies of the MRPT wheel inside the manylinux_2_28 images
# (AlmaLinux 8), run once per image by cibuildwheel ("before-all").
# Libraries not installed here (assimp, octomap, tinyxml2, nanoflann,
# simpleini...) use the versions embedded in the MRPT sources.
set -euo pipefail

dnf install -y epel-release
dnf install -y \
  ccache \
  zlib-devel libzstd-devel lz4-devel jsoncpp-devel \
  mesa-libGL-devel mesa-libEGL-devel libglvnd-devel \
  libX11-devel libXrandr-devel libXinerama-devel libXcursor-devel libXi-devel \
  libXxf86vm-devel

# GLFW, as a shared library bundled into the wheel by auditwheel. X11 backend
# only: it also works in Wayland sessions, via XWayland.
GLFW_VERSION=3.4
curl -fsSL "https://github.com/glfw/glfw/archive/refs/tags/${GLFW_VERSION}.tar.gz" | tar xz -C /tmp
cmake -S "/tmp/glfw-${GLFW_VERSION}" -B /tmp/glfw-build -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_INSTALL_PREFIX=/usr -DCMAKE_INSTALL_LIBDIR=lib64 -DBUILD_SHARED_LIBS=ON \
  -DGLFW_BUILD_WAYLAND=OFF -DGLFW_BUILD_EXAMPLES=OFF -DGLFW_BUILD_TESTS=OFF -DGLFW_BUILD_DOCS=OFF
cmake --build /tmp/glfw-build --parallel
cmake --install /tmp/glfw-build
rm -rf "/tmp/glfw-${GLFW_VERSION}" /tmp/glfw-build

# Eigen (header only): the distribution one is too old.
EIGEN_VERSION=3.4.0
curl -fsSL "https://gitlab.com/libeigen/eigen/-/archive/${EIGEN_VERSION}/eigen-${EIGEN_VERSION}.tar.gz" \
  | tar xz -C /tmp
cmake -S "/tmp/eigen-${EIGEN_VERSION}" -B /tmp/eigen-build \
  -DCMAKE_INSTALL_PREFIX=/usr/local -DBUILD_TESTING=OFF -DEIGEN_BUILD_DOC=OFF
cmake --install /tmp/eigen-build
rm -rf "/tmp/eigen-${EIGEN_VERSION}" /tmp/eigen-build
