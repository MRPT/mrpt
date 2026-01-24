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
include(./cmake/script_gl_glut.cmake REQUIRED)     # Check for the GL,GLUT libraries

option(MRPT_OPENGL_PROFILER OFF)

# define lib:
set(LIB_SOURCES
  src/RenderQueue.cpp
  src/gltext.cpp
  src/CompiledScene.cpp
  src/LinesProxy.cpp
  src/Shader.cpp
  src/TTriangle.cpp
  src/TexturedTrianglesProxy.cpp
  src/DefaultShaders.cpp
  src/TRenderMatrices.cpp
  src/CFBORender.cpp
  src/checkOpenGLErr.cpp
  src/Texture.cpp
  src/TrianglesProxy.cpp
  src/ShaderProgramManager.cpp
  src/CompiledViewport.cpp
  src/Buffer.cpp
  src/VertexArrayObject.cpp
  src/FrameBuffer.cpp
  src/RenderableProxy.cpp
  src/PointsProxy.cpp
  # tests ---
  tests/CFBORender_unittest.cpp
  tests/TRenderMatrices_unittest.cpp
)

set(LIB_PUBLIC_HEADERS
  include/mrpt/opengl/RenderableProxy.h
  include/mrpt/opengl/VertexArrayObject.h
  include/mrpt/opengl/CompiledScene.h
  include/mrpt/opengl/Shader.h
  include/mrpt/opengl/Texture.h
  include/mrpt/opengl/ShaderProgramManager.h
  include/mrpt/opengl/PointsProxy.h
  include/mrpt/opengl/registerAllClasses.h
  include/mrpt/opengl/CompiledViewport.h
  include/mrpt/opengl/TrianglesProxy.h
  include/mrpt/opengl/pointcloud_adapters.h
  include/mrpt/opengl/CFBORender.h
  include/mrpt/opengl/LinesProxy.h
  include/mrpt/opengl/TTriangle.h
  include/mrpt/opengl/TexturedTrianglesProxy.h
  include/mrpt/opengl/DefaultShaders.h
  include/mrpt/opengl/FrameBuffer.h
  include/mrpt/opengl/TRenderMatrices.h
  include/mrpt/opengl/opengl_api.h
  include/mrpt/opengl/OpenGLDepth2LinearLUTs.h
  include/mrpt/opengl/RenderQueue.h
  include/mrpt/opengl/Buffer.h
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

if ("$ENV{VERBOSE}")
  message(STATUS "MRPT_OPENGL_LIBS: ${MRPT_OPENGL_LIBS}")
endif()

if (TARGET mrpt_glew) # Basically, for Windows
  target_link_libraries(${PROJECT_NAME} PRIVATE mrpt_glew)
endif()

if (MINGW)
  target_link_libraries(${PROJECT_NAME} PRIVATE opengl32 GlU32)
endif()
