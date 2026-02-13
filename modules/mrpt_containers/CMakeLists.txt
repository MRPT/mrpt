# ------------------------------------------------------------------------------
#        Mobile Robot Programming Toolkit (MRPT)
#
# Copyright (c) 2005-2026, Jose Luis Blanco-Claraco, contributors (see Git history)
# All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
# ------------------------------------------------------------------------------

cmake_minimum_required(VERSION 3.16)

# Tell CMake we'll use C++ for use in its tests/flags
project(mrpt_containers LANGUAGES C CXX)

# MRPT CMake scripts: "mrpt_xxx()"
find_package(mrpt_common REQUIRED)
find_package(mrpt_core REQUIRED)
find_package(mrpt_typemeta REQUIRED)

include(./cmake/find_libfyaml.cmake REQUIRED)    # Defines embedded version of libfyaml

# define lib:
set(LIB_SOURCES
  src/ts_hash_map.cpp
  src/yaml.cpp
  src/CDynamicGrid.cpp
  tests/ts_hash_map_unittest.cpp
  tests/vector_with_small_size_optimization_unittest.cpp
  tests/poly_ptr_unittest.cpp
  tests/bimap_unittest.cpp
  tests/CDynamicGrid_unittest.cpp
  tests/circularbuffer_unittest.cpp
  tests/yaml_unittest.cpp
  tests/visit_each_unittest.cpp
  tests/find_closest_unittest.cpp
)

set(LIB_PUBLIC_HEADERS
  include/mrpt/containers/vector_with_small_size_optimization.h
  include/mrpt/containers/internal_yaml_fwrds.h
  include/mrpt/containers/deepcopy_ptr.h
  include/mrpt/containers/ts_hash_map.h
  include/mrpt/containers/CDynamicGrid3D.h
  include/mrpt/containers/CThreadSafeQueue.h
  include/mrpt/containers/deepcopy_poly_ptr.h
  include/mrpt/containers/bimap.h
  include/mrpt/containers/yaml_frwd.h
  include/mrpt/containers/traits_map.h
  include/mrpt/containers/ci_less.h
  include/mrpt/containers/NonCopiableData.h
  include/mrpt/containers/CommentPosition.h
  include/mrpt/containers/map_as_vector.h
  include/mrpt/containers/copy_container_typecasting.h
  include/mrpt/containers/ValueCommentPair.h
  include/mrpt/containers/stl_containers_utils.h
  include/mrpt/containers/MT_buffer.h
  include/mrpt/containers/YamlEmitOptions.h
  include/mrpt/containers/visit_each.h
  include/mrpt/containers/list_searchable.h
  include/mrpt/containers/ContainerReadOnlyProxyAccessor.h
  include/mrpt/containers/yaml.h
  include/mrpt/containers/PerThreadDataHolder.h
  include/mrpt/containers/printf_vector.h
  include/mrpt/containers/CDynamicGrid.h
  include/mrpt/containers/circular_buffer.h
  include/mrpt/containers/find_closest.h
)

mrpt_add_library(
  TARGET ${PROJECT_NAME}
  SOURCES ${LIB_SOURCES} ${LIB_PUBLIC_HEADERS}
  PUBLIC_LINK_LIBRARIES
    mrpt::mrpt_core
    mrpt::mrpt_typemeta
#  PRIVATE_LINK_LIBRARIES
  CMAKE_DEPENDENCIES
    mrpt_core
    mrpt_typemeta
)

if (TARGET mrpt_libfyaml)
  target_link_libraries(${PROJECT_NAME} PRIVATE mrpt_libfyaml)
else()
  target_include_directories(${PROJECT_NAME} SYSTEM PRIVATE ${LIBFYAML_INCLUDES})
  target_link_libraries(${PROJECT_NAME} PRIVATE ${LIBFYAML_LIB})
endif()

if (DEFINED libfyaml_VERSION)
    string(LENGTH "${libfyaml_VERSION}" _len)
    if(_len LESS 5)
      set(libfyaml_VERSION "0.${libfyaml_VERSION}")
    endif()
    unset(_len)

    VERSION_TO_HEXADECIMAL(MRPT_LIBFYAML_VERSION_HEX ${libfyaml_VERSION})
    target_compile_definitions(${PROJECT_NAME} PRIVATE -DMRPT_LIBFYAML_VERSION=${MRPT_LIBFYAML_VERSION_HEX})
    message(STATUS "MRPT_LIBFYAML_VERSION_HEX  : ${MRPT_LIBFYAML_VERSION_HEX}")
endif()

# Python wrapper:
mrpt_add_python_module(containers
  python_bindings/${PROJECT_NAME}_py.cpp
)

