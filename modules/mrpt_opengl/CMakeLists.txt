# ------------------------------------------------------------------------------
#        Mobile Robot Programming Toolkit (MRPT)
#
# Copyright (c) 2005-2026, Jose Luis Blanco-Claraco, contributors (see Git history)
# All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
# ------------------------------------------------------------------------------

cmake_minimum_required(VERSION 3.16)
project(mrpt_opengl LANGUAGES C CXX)

# MRPT CMake scripts: "mrpt_xxx()"
find_package(mrpt_common REQUIRED)

find_package(mrpt_viz REQUIRED)
find_package(mrpt_poses REQUIRED)
find_package(mrpt_img REQUIRED)

# Lists of directories with source files:
#  See "DeclareMRPTLib.cmake" for explanations
# -------------------------------------------------
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")
include(./cmake/script_assimp.cmake REQUIRED)      # Check for system assimp lib (3D models)
include(./cmake/script_gl_glut.cmake REQUIRED)     # Check for the GL,GLUT libraries

# Assimp:
if (CMAKE_MRPT_HAS_ASSIMP)
	# ASSIMP_ROOT_DIR - the root directory where the installation can be found
	# ASSIMP_CXX_FLAGS - extra flags for compilation
	# ASSIMP_LINK_FLAGS - extra flags for linking
	# ASSIMP_INCLUDE_DIRS - include directories
	# ASSIMP_LIBRARY_DIRS - link directories
	# ASSIMP_LIBRARIES - libraries to link plugins with
	if (NOT ${ASSIMP_CXX_FLAGS} STREQUAL "")
		add_definitions("${ASSIMP_CXX_FLAGS}")
	endif()
	if (NOT "${ASSIMP_INCLUDE_DIRS}" STREQUAL "")
		include_directories("${ASSIMP_INCLUDE_DIRS}")
	endif()
	if(NOT "${ASSIMP_VERSION}" STREQUAL "")
		string(SUBSTRING "${ASSIMP_VERSION}" 0 1 MRPT_ASSIMP_VERSION_MAJOR)
		add_definitions(-DMRPT_ASSIMP_VERSION_MAJOR=${MRPT_ASSIMP_VERSION_MAJOR})
	endif()
endif ()

option(MRPT_OPENGL_PROFILER OFF)

# define lib:
set(LIB_SOURCES
  src/Buffer.cpp
  src/CompiledScene.cpp
  src/CompiledViewport.cpp
  src/Shader.cpp
  src/ShaderProgramManager.cpp
  src/Texture.cpp
  src/TRenderMatrices.cpp
  src/VertexArrayObject.cpp
)

set(LIB_PUBLIC_HEADERS
  include/mrpt/opengl/Buffer.h
  include/mrpt/opengl/CompiledScene.h
  include/mrpt/opengl/CompiledViewport.h
  include/mrpt/opengl/Shader.h
  include/mrpt/opengl/Texture.h
  include/mrpt/opengl/TRenderMatrices.h
  include/mrpt/opengl/VertexArrayObject.h
)

mrpt_add_library(
  TARGET ${PROJECT_NAME}
  SOURCES ${LIB_SOURCES} ${LIB_PUBLIC_HEADERS}
  PUBLIC_LINK_LIBRARIES
    mrpt::mrpt_viz
    mrpt::mrpt_poses
    mrpt::mrpt_img
  CMAKE_DEPENDENCIES
    mrpt_viz
    mrpt_poses
    mrpt_img
)

if(NOT BUILD_mrpt-opengl)
  return()
endif()

if (MRPT_OPENGL_PROFILER)
	target_compile_definitions(${PROJECT_NAME} PRIVATE MRPT_OPENGL_PROFILER)
endif()

# https://developer.apple.com/macos/whats-new/#deprecationofopenglandopencl
if (APPLE)
	target_compile_definitions(${PROJECT_NAME} PRIVATE GL_SILENCE_DEPRECATION)
endif()

# Link against gl, glu, glut:
target_link_libraries(${PROJECT_NAME} PRIVATE
	${MRPT_OPENGL_LIBS}
	)
if (TARGET mrpt_glew) # Basically, for Windows
	target_link_libraries(${PROJECT_NAME} PRIVATE mrpt_glew)
endif()

# Link against assimp:
if (CMAKE_MRPT_HAS_ASSIMP)
	if(TARGET "EP_assimp")
		add_dependencies(${PROJECT_NAME} EP_assimp)
	endif()
	target_link_libraries(${PROJECT_NAME} PRIVATE ${ASSIMP_LIBRARIES})
endif ()
if (MINGW)
	target_link_libraries(${PROJECT_NAME} PRIVATE opengl32 GlU32)
endif()

