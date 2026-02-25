
cmake_minimum_required(VERSION 3.16)
project(mrpt_imgui LANGUAGES C CXX)

# Ensure core MRPT packages used here are available to CMake
find_package(mrpt_common REQUIRED)
find_package(mrpt_opengl REQUIRED)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")
find_package(PkgConfig QUIET)
include(./cmake/script_gl_glut.cmake REQUIRED)     # Check for the GL,GLUT libraries

# define lib:
set(LIB_SOURCES
  src/CImGuiSceneView.cpp
)

set(LIB_PUBLIC_HEADERS
  include/mrpt/imgui/CImGuiSceneView.h
)

mrpt_add_library(
  TARGET ${PROJECT_NAME}
  SOURCES ${LIB_SOURCES} ${LIB_PUBLIC_HEADERS}
  PUBLIC_LINK_LIBRARIES
    mrpt::mrpt_opengl
  CMAKE_DEPENDENCIES
    mrpt_opengl
)

if(MRPT_OPENGL_LIBS)
  # Link against gl:
  target_link_libraries(${PROJECT_NAME} PRIVATE ${MRPT_OPENGL_LIBS})
endif()
