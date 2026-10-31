
cmake_minimum_required(VERSION 3.16)
project(mrpt_imgui LANGUAGES C CXX)

# Ensure core MRPT packages used here are available to CMake
find_package(mrpt_common REQUIRED)
find_package(mrpt_opengl REQUIRED)
find_package(mrpt_imgui_vendor QUIET)
if(NOT mrpt_imgui_vendor_FOUND)
  # The vendor package is skipped when built without GLFW
  # (MRPT_IMGUI_VENDOR_ENABLE=OFF): so is this one.
  message(STATUS "mrpt_imgui: mrpt_imgui_vendor not found, skipping.")
  return()
endif()

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")
find_package(PkgConfig QUIET)
include(./cmake/script_opengl.cmake REQUIRED)     # Check for OpenGL / EGL / GLES

# Bundled GLEW (Windows colcon builds only; skipped when a "mrpt_glew"
# target already exists from a non-colcon top-level build). This must be
# defined before mrpt_add_library() so it can be added to this library's
# export set, since mrpt_imgui links it PUBLIC and its public header pulls in
# GLEW through mrpt/opengl/opengl_api.h.
if(WIN32 AND NOT TARGET mrpt_glew)
  # In colcon builds 3rdparty/ is excluded (COLCON_IGNORE); build bundled GLEW here.
  set(_MRPT_3RDPARTY "${CMAKE_CURRENT_SOURCE_DIR}/../../3rdparty")
  if(EXISTS "${_MRPT_3RDPARTY}/glew/src/glew.c")
    add_library(mrpt_glew STATIC "${_MRPT_3RDPARTY}/glew/src/glew.c")
    target_compile_definitions(mrpt_glew PUBLIC GLEW_STATIC)
    # PRIVATE: only glew.c itself needs these source-tree paths to compile;
    # consumers get GL/glew.h from mrpt_opengl's installed headers instead
    # (see opengl_api.h), so these paths must not leak into the export set.
    target_include_directories(mrpt_glew SYSTEM PRIVATE
      "${_MRPT_3RDPARTY}/glew/include/"
      "${_MRPT_3RDPARTY}/glext/include/")
    if(MSVC)
      target_compile_options(mrpt_glew PRIVATE /W0)
    endif()
    set_target_properties(mrpt_glew PROPERTIES FOLDER "3rd party")
  endif()
endif()

if(TARGET mrpt_glew)
  set(_MRPT_IMGUI_ADDITIONAL_EXPORT_TARGETS mrpt_glew)
else()
  set(_MRPT_IMGUI_ADDITIONAL_EXPORT_TARGETS "")
endif()

# define lib:
set(LIB_SOURCES
  src/CImGuiSceneView.cpp
  src/CImGuiAppShell.cpp
  src/CImGuiBackgroundScene.cpp
  src/CImGuiTimeline.cpp
  src/CImGuiImageView.cpp
  src/CImGuiPlotPanel.cpp
  src/CImGuiLogConsole.cpp
  src/CImGuiFileDialog.cpp
  src/SimpleFileDialog.cpp
  src/style.cpp
  src/widgets.cpp
  src/config_paths.cpp
)

set(LIB_PUBLIC_HEADERS
  include/mrpt/imgui/AsyncTask.h
  include/mrpt/imgui/CImGuiAppShell.h
  include/mrpt/imgui/CImGuiBackgroundScene.h
  include/mrpt/imgui/CImGuiFileDialog.h
  include/mrpt/imgui/CImGuiImageView.h
  include/mrpt/imgui/CImGuiLogConsole.h
  include/mrpt/imgui/CImGuiPlotPanel.h
  include/mrpt/imgui/CImGuiSceneView.h
  include/mrpt/imgui/CImGuiTimeline.h
  include/mrpt/imgui/SimpleFileDialog.h
  include/mrpt/imgui/config_paths.h
  include/mrpt/imgui/icons.h
  include/mrpt/imgui/style.h
  include/mrpt/imgui/widgets.h
)

set(LIB_UNIT_TEST_SOURCES
  tests/AsyncTask_unittest.cpp
  tests/CImGuiAppShell_unittest.cpp
  tests/CImGuiFileDialog_unittest.cpp
  tests/CImGuiImageView_unittest.cpp
  tests/CImGuiLogConsole_unittest.cpp
  tests/CImGuiPlotPanel_unittest.cpp
  tests/CImGuiTimeline_unittest.cpp
  tests/config_paths_unittest.cpp
  tests/style_unittest.cpp
  tests/widgets_unittest.cpp
)

mrpt_add_library(
  TARGET ${PROJECT_NAME}
  SOURCES ${LIB_SOURCES} ${LIB_PUBLIC_HEADERS}
  UNIT_TEST_SOURCES ${LIB_UNIT_TEST_SOURCES}
  PUBLIC_LINK_LIBRARIES
    mrpt::mrpt_opengl
    mrpt::mrpt_imgui_vendor
  CMAKE_DEPENDENCIES
    mrpt_opengl
    mrpt_imgui_vendor
  ADDITIONAL_EXPORT_TARGETS
    ${_MRPT_IMGUI_ADDITIONAL_EXPORT_TARGETS}
)

if(MRPT_OPENGL_LIBS)
  # Link against gl:
  target_link_libraries(${PROJECT_NAME} PRIVATE ${MRPT_OPENGL_LIBS})
endif()

if(TARGET mrpt_glew)
  target_link_libraries(${PROJECT_NAME} PUBLIC mrpt_glew)
endif()
