# ------------------------------------------------------------------------------
#        Mobile Robot Programming Toolkit (MRPT)
#
# Copyright (c) 2005-2026, Jose Luis Blanco-Claraco, contributors (see Git history)
# All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
# ------------------------------------------------------------------------------

# Superbuild for the MRPT Python wheel, driven by scikit-build-core (see
# pyproject.toml). Each MRPT module is an independent CMake project, so they are
# built here in dependency order as ExternalProjects, all installed into one
# staging tree with the layout of the wheel's site-packages contents:
#
#   mrpt/<module>/        Python packages (__init__.py, .pyi stubs, _bindings)
#   mrpt/sdk/             C++ SDK: include/, lib/ (shared libs + CMake configs)
#
# Downstream wheels with C++ code build against mrpt/sdk (see mrpt/sdk/__init__.py).

cmake_minimum_required(VERSION 3.24)
project(mrpt_wheel LANGUAGES C CXX)

include(ExternalProject)

set(MRPT_MODULES_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../../modules")

# Modules shipped in the wheel. Their mrpt_* dependencies are added
# automatically. Hardware drivers are left out: they need device libraries
# that make little sense in a portable wheel.
set(MRPT_WHEEL_MODULES
  mrpt_bayes mrpt_comms mrpt_config mrpt_containers mrpt_core mrpt_expr
  mrpt_graphs mrpt_gui mrpt_img mrpt_io mrpt_kinematics mrpt_maps mrpt_math
  mrpt_nav mrpt_obs mrpt_opengl mrpt_poses mrpt_random mrpt_rtti
  mrpt_serialization mrpt_slam mrpt_system mrpt_tfest mrpt_topography
  mrpt_viz
  CACHE STRING "MRPT modules to include in the wheel")

# Extra CMake arguments for every module, e.g. "-DDISABLE_ASSIMP=ON".
set(MRPT_WHEEL_EXTRA_CMAKE_ARGS "" CACHE STRING "Extra CMake args for all modules")

# Optional GUI toolkits: the wheel only uses the GLFW-based (nanogui) windows.
list(APPEND MRPT_WHEEL_EXTRA_CMAKE_ARGS -DDISABLE_WXWIDGETS=ON -DMRPT_WITH_QT=OFF)
# GLFW embedded in nanogui, X11 backend only (it also works in Wayland
# sessions, via XWayland):
list(APPEND MRPT_WHEEL_EXTRA_CMAKE_ARGS -DGLFW_BUILD_WAYLAND=OFF)
# Built into libmrpt_obs: one less library to bundle.
list(APPEND MRPT_WHEEL_EXTRA_CMAKE_ARGS -DTINYXML2_USE_EMBEDDED_VERSION=ON)

find_package(Python COMPONENTS Interpreter Development.Module REQUIRED)
find_package(pybind11 CONFIG REQUIRED)

# mrpt_* dependencies of a module, as listed in its package.xml:
function(mrpt_wheel_get_deps MODULE OUT_VAR)
  file(READ "${MRPT_MODULES_DIR}/${MODULE}/package.xml" _xml)
  string(REGEX MATCHALL "<(build_)?depend>mrpt_[a-z_]+</" _matches "${_xml}")
  set(_deps "")
  foreach(_m ${_matches})
    string(REGEX REPLACE "<(build_)?depend>(mrpt_[a-z_]+)</" "\\2" _dep "${_m}")
    list(APPEND _deps ${_dep})
  endforeach()
  list(REMOVE_DUPLICATES _deps)
  set(${OUT_VAR} ${_deps} PARENT_SCOPE)
endfunction()

# Transitive closure of the requested modules:
set(_all_modules "")
set(_pending ${MRPT_WHEEL_MODULES} mrpt_common)
while(_pending)
  list(POP_FRONT _pending _mod)
  if (_mod IN_LIST _all_modules)
    continue()
  endif()
  if (NOT EXISTS "${MRPT_MODULES_DIR}/${_mod}/CMakeLists.txt")
    message(FATAL_ERROR "MRPT module not found: ${_mod}")
  endif()
  list(APPEND _all_modules ${_mod})
  mrpt_wheel_get_deps(${_mod} _deps)
  list(APPEND _pending ${_deps})
endwhile()

# Sort them so each module comes after its dependencies:
set(_sorted_modules "")
while(_all_modules)
  set(_progress FALSE)
  foreach(_mod ${_all_modules})
    mrpt_wheel_get_deps(${_mod} _deps)
    if (_sorted_modules)
      list(REMOVE_ITEM _deps ${_sorted_modules})
    endif()
    if (NOT _deps)
      list(APPEND _sorted_modules ${_mod})
      list(REMOVE_ITEM _all_modules ${_mod})
      set(_progress TRUE)
    endif()
  endforeach()
  if (NOT _progress)
    message(FATAL_ERROR "Circular dependencies among: ${_all_modules}")
  endif()
endwhile()
message(STATUS "MRPT modules in the wheel: ${_sorted_modules}")

set(MRPT_STAGE_DIR "${CMAKE_BINARY_DIR}/stage")
set(MRPT_SDK_DIR "${MRPT_STAGE_DIR}/mrpt/sdk")

# Shared libraries go to mrpt/sdk/lib/, the Python extensions to
# mrpt/<module>/: one RPATH that works for both locations.
if (APPLE)
  set(_rpath "@loader_path;@loader_path/../sdk/lib")
else()
  set(_rpath "\$ORIGIN;\$ORIGIN/../sdk/lib")
endif()

set(_cache_args
  -DCMAKE_BUILD_TYPE:STRING=Release
  -DCMAKE_INSTALL_PREFIX:PATH=${MRPT_SDK_DIR}
  -DCMAKE_INSTALL_LIBDIR:STRING=lib
  -DCMAKE_PREFIX_PATH:STRING=${MRPT_SDK_DIR}
  -DCMAKE_INSTALL_RPATH:STRING=${_rpath}
  -DMRPT_PYTHON_INSTALL_DIR:PATH=${MRPT_STAGE_DIR}
  -DMRPT_NO_SOVERSION:BOOL=ON
  -DBUILD_TESTING:BOOL=OFF
  -DPython_EXECUTABLE:FILEPATH=${Python_EXECUTABLE}
  -DPython3_EXECUTABLE:FILEPATH=${Python_EXECUTABLE}
  -Dpybind11_DIR:PATH=${pybind11_DIR}
  # Do not use libraries from the user package registry (~/.cmake/packages):
  # they would not be bundled into the wheel.
  -DCMAKE_FIND_USE_PACKAGE_REGISTRY:BOOL=OFF
  -DCMAKE_EXPORT_NO_PACKAGE_REGISTRY:BOOL=ON
)
# (CMAKE_MAKE_PROGRAM changes on each build: pip uses temporary environments)
foreach(_var
    CMAKE_MAKE_PROGRAM CMAKE_C_COMPILER CMAKE_CXX_COMPILER
    CMAKE_C_COMPILER_LAUNCHER CMAKE_CXX_COMPILER_LAUNCHER
    CMAKE_OSX_DEPLOYMENT_TARGET CMAKE_OSX_ARCHITECTURES
    CMAKE_TOOLCHAIN_FILE VCPKG_TARGET_TRIPLET)
  if (DEFINED ${_var} AND NOT "${${_var}}" STREQUAL "")
    list(APPEND _cache_args "-D${_var}:STRING=${${_var}}")
  endif()
endforeach()

# Eigen (header only), installed into the SDK: some public headers use it.
set(MRPT_WHEEL_EIGEN_VERSION 3.4.0)
ExternalProject_Add(ep_eigen
  URL "https://gitlab.com/libeigen/eigen/-/archive/${MRPT_WHEEL_EIGEN_VERSION}/eigen-${MRPT_WHEEL_EIGEN_VERSION}.tar.gz"
  URL_HASH SHA256=8586084f71f9bde545ee7fa6d00288b264a2b7ac3607b974e54d13e7162c1c72
  DOWNLOAD_EXTRACT_TIMESTAMP ON
  BINARY_DIR "${CMAKE_BINARY_DIR}/eigen"
  CMAKE_CACHE_ARGS ${_cache_args}
  CMAKE_ARGS
    -DBUILD_TESTING=OFF
    -DEIGEN_BUILD_DOC=OFF
    -DEIGEN_BUILD_PKGCONFIG=OFF
  USES_TERMINAL_CONFIGURE ON
  USES_TERMINAL_BUILD ON
  USES_TERMINAL_INSTALL ON
)

# oneTBB, installed into the SDK too (headers, library and CMake config), since
# MRPT public headers use it. Downstream wheels must use this one as well.
set(MRPT_WHEEL_TBB_VERSION 2022.3.0)
ExternalProject_Add(ep_tbb
  URL "https://github.com/uxlfoundation/oneTBB/archive/refs/tags/v${MRPT_WHEEL_TBB_VERSION}.tar.gz"
  URL_HASH SHA256=01598a46c1162c27253a0de0236f520fd8ee8166e9ebb84a4243574f88e6e50a
  DOWNLOAD_EXTRACT_TIMESTAMP ON
  BINARY_DIR "${CMAKE_BINARY_DIR}/tbb"
  CMAKE_CACHE_ARGS ${_cache_args}
  CMAKE_ARGS
    -DTBB_TEST=OFF
    -DTBB_STRICT=OFF
    -DTBBMALLOC_PROXY_BUILD=OFF
    -DTBB_DISABLE_HWLOC_AUTOMATIC_SEARCH=ON
  USES_TERMINAL_CONFIGURE ON
  USES_TERMINAL_BUILD ON
  USES_TERMINAL_INSTALL ON
)

foreach(_mod ${_sorted_modules})
  mrpt_wheel_get_deps(${_mod} _deps)
  set(_ep_deps ep_eigen ep_tbb)
  foreach(_dep ${_deps})
    list(APPEND _ep_deps ep_${_dep})
  endforeach()

  ExternalProject_Add(ep_${_mod}
    SOURCE_DIR "${MRPT_MODULES_DIR}/${_mod}"
    BINARY_DIR "${CMAKE_BINARY_DIR}/modules/${_mod}"
    CMAKE_CACHE_ARGS ${_cache_args}
    CMAKE_ARGS ${MRPT_WHEEL_EXTRA_CMAKE_ARGS}
    DEPENDS ${_ep_deps}
    # Always re-run the module build tool, so source edits are picked up:
    BUILD_ALWAYS ON
    # One module at a time with Ninja (console pool), each one in parallel:
    USES_TERMINAL_CONFIGURE ON
    USES_TERMINAL_BUILD ON
    USES_TERMINAL_INSTALL ON
  )
endforeach()

# Copy the staging tree into the wheel. Data, docs and programs are left out.
install(DIRECTORY "${MRPT_STAGE_DIR}/" DESTINATION .
  USE_SOURCE_PERMISSIONS
  PATTERN "mrpt/sdk/bin" EXCLUDE
  PATTERN "mrpt/sdk/share/doc" EXCLUDE
  PATTERN "mrpt/sdk/share/mrpt_data" EXCLUDE
  PATTERN "__pycache__" EXCLUDE
)

# Versions downstream C++ extensions must match:
file(READ "${MRPT_MODULES_DIR}/mrpt_common/package.xml" _xml)
string(REGEX MATCH "<version>([0-9.]+)</version>" _ "${_xml}")
file(WRITE "${CMAKE_BINARY_DIR}/_build_info.py"
  "MRPT_VERSION = \"${CMAKE_MATCH_1}\"\nPYBIND11_VERSION = \"${pybind11_VERSION}\"\n")
install(FILES
  "${CMAKE_CURRENT_SOURCE_DIR}/mrpt_sdk/__init__.py"
  "${CMAKE_BINARY_DIR}/_build_info.py"
  DESTINATION mrpt/sdk)
