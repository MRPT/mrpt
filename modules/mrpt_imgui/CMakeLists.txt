
cmake_minimum_required(VERSION 3.16)
project(mrpt_imgui LANGUAGES C CXX)

# Ensure core MRPT packages used here are available to CMake
find_package(mrpt_common REQUIRED)
find_package(mrpt_opengl REQUIRED)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")
find_package(PkgConfig QUIET)
include(./cmake/script_opengl.cmake REQUIRED)     # Check for OpenGL / EGL / GLES

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

if(TARGET mrpt_glew)
  target_link_libraries(${PROJECT_NAME} PRIVATE mrpt_glew)
endif()
