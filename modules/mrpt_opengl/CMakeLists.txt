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
find_package(Eigen3 REQUIRED)

# Lists of directories with source files:
#  See "DeclareMRPTLib.cmake" for explanations
# -------------------------------------------------
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")
include(./cmake/script_opengl.cmake REQUIRED)     # Check for OpenGL / EGL / GLES

option(MRPT_OPENGL_PROFILER OFF)

# define lib:
set(LIB_SOURCES
  src/RenderQueue.cpp
  src/gltext.cpp
  src/CompiledScene.cpp
  src/LinesProxy.cpp
  src/Shader.cpp
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
  src/SkyBoxProxy.cpp
)

set(LIB_UNIT_TEST_SOURCES
  tests/CFBORender_unittest.cpp
  tests/CFBORender_SolidPrimitives_unittest.cpp
  tests/CFBORender_LinePointPrimitives_unittest.cpp
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
  include/mrpt/opengl/TexturedTrianglesProxy.h
  include/mrpt/opengl/SkyBoxProxy.h
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
  UNIT_TEST_SOURCES ${LIB_UNIT_TEST_SOURCES}
  PUBLIC_LINK_LIBRARIES
    mrpt::mrpt_viz
    mrpt::mrpt_poses
    mrpt::mrpt_img
  PRIVATE_LINK_LIBRARIES
    Eigen3::Eigen
  CMAKE_DEPENDENCIES
    mrpt_viz
    mrpt_poses
    mrpt_img
  UNITTEST_LINK_LIBRARIES
    Eigen3::Eigen
)

mrpt_add_python_module(opengl python_bindings/${PROJECT_NAME}_py.cpp)

if (MRPT_OPENGL_PROFILER)
  target_compile_definitions(${PROJECT_NAME} PRIVATE MRPT_OPENGL_PROFILER)
endif()

# https://developer.apple.com/macos/whats-new/#deprecationofopenglandopencl
if (APPLE)
  target_compile_definitions(${PROJECT_NAME} PRIVATE GL_SILENCE_DEPRECATION)
endif()

# Link against OpenGL / EGL / GLES:
target_link_libraries(${PROJECT_NAME} PRIVATE
  ${MRPT_OPENGL_LIBS}
)

if ("$ENV{VERBOSE}")
  message(STATUS "MRPT_OPENGL_LIBS: ${MRPT_OPENGL_LIBS}")
endif()

if(WIN32 AND NOT TARGET mrpt_glew)
  # In colcon builds 3rdparty/ is excluded (COLCON_IGNORE); build bundled GLEW here.
  set(_MRPT_3RDPARTY "${CMAKE_CURRENT_SOURCE_DIR}/../../3rdparty")
  if(EXISTS "${_MRPT_3RDPARTY}/glew/src/glew.c")
    add_library(mrpt_glew STATIC "${_MRPT_3RDPARTY}/glew/src/glew.c")
    target_compile_definitions(mrpt_glew PUBLIC GLEW_STATIC)
    target_include_directories(mrpt_glew SYSTEM PUBLIC
      "${_MRPT_3RDPARTY}/glew/include/"
      "${_MRPT_3RDPARTY}/glext/include/")
    if(MSVC)
      target_compile_options(mrpt_glew PRIVATE /W0)
    endif()
    set_target_properties(mrpt_glew PROPERTIES FOLDER "3rd party")
  endif()
endif()

if (TARGET mrpt_glew) # Basically, for Windows
  target_link_libraries(${PROJECT_NAME} PRIVATE mrpt_glew)
  # Install GLEW and glext headers so downstream colcon packages (mrpt_imgui,
  # etc.) that include opengl_api.h can resolve GL/glew.h and glext/glext.h.
  if(DEFINED _MRPT_3RDPARTY)
    if(EXISTS "${_MRPT_3RDPARTY}/glew/include/GL")
      install(DIRECTORY "${_MRPT_3RDPARTY}/glew/include/GL"
        DESTINATION "${CMAKE_INSTALL_INCLUDEDIR}")
    endif()
    # opengl_api.h uses #include "glext/glext.h" (relative), so install
    # glext.h next to opengl_api.h in include/mrpt/opengl/glext/.
    if(EXISTS "${_MRPT_3RDPARTY}/glext/include/glext/glext.h")
      install(FILES "${_MRPT_3RDPARTY}/glext/include/glext/glext.h"
        DESTINATION "${CMAKE_INSTALL_INCLUDEDIR}/mrpt/opengl/glext")
    endif()
  endif()
endif()

if (MINGW)
  target_link_libraries(${PROJECT_NAME} PRIVATE opengl32 GlU32)
endif()
