#                    _
#                   | |    Mobile Robot Programming Toolkit (MRPT)
#  _ __ ___  _ __ _ | |_
# | '_ ` _ \| '__| '_ \ __|         https://www.mrpt.org/
# | | | | | | |  | |_) | |_
# |_| |_| |_|_|  | .__/ \__|    https://github.com/MRPT/mrpt/
#                | |
#                |_|
#
# Copyright (c) 2005-2026, Individual contributors, see AUTHORS file
# See: https://www.mrpt.org/Authors - All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause

cmake_minimum_required(VERSION 3.16)
project(mrpt_imgui_vendor LANGUAGES CXX)

find_package(mrpt_common REQUIRED)

# ---------------------------------------------------------------------------
# Options
# ---------------------------------------------------------------------------
# Static by default: Dear ImGui has no ABI-stability discipline and no upstream
# SOVERSION, so a shared library whose ABI changes between patch releases would
# silently break already-installed consumers. All consumers ship in the same
# release, hence there is nothing to deduplicate.
option(MRPT_IMGUI_VENDOR_SHARED
  "Build the vendored ImGui/ImPlot library as a shared library" OFF)

# Off by default: the two demo translation units are the bulk of the archive and
# nothing ships against them. Turn on while developing UI code to get
# ImGui::ShowDemoWindow() / ImPlot::ShowDemoWindow() as a widget reference.
option(MRPT_IMGUI_VENDOR_WITH_DEMO
  "Include imgui_demo.cpp and implot_demo.cpp (ImGui::ShowDemoWindow())" OFF)

# ---------------------------------------------------------------------------
# Submodule sanity checks
# ---------------------------------------------------------------------------
set(IMGUI_DIR "${CMAKE_CURRENT_SOURCE_DIR}/3rdparty/imgui")
set(IMPLOT_DIR "${CMAKE_CURRENT_SOURCE_DIR}/3rdparty/implot")
set(PFD_DIR "${CMAKE_CURRENT_SOURCE_DIR}/3rdparty/portable-file-dialogs")
set(ICONFONTS_DIR "${CMAKE_CURRENT_SOURCE_DIR}/3rdparty/IconFontCppHeaders")

foreach(_probe
    "${IMGUI_DIR}/imgui.h"
    "${IMPLOT_DIR}/implot.h"
    "${PFD_DIR}/portable-file-dialogs.h"
    "${ICONFONTS_DIR}/IconsMaterialSymbols.h")
  if(NOT EXISTS "${_probe}")
    message(FATAL_ERROR
      "Missing vendored source: ${_probe}\n"
      "Run:  git submodule update --init --recursive")
  endif()
endforeach()
unset(_probe)

# ---------------------------------------------------------------------------
# Dependencies
# ---------------------------------------------------------------------------
# This package is the base of MRPT's ImGui GUI stack, so a missing GLFW is a
# hard error rather than a silent skip: quietly dropping it would make the GUI
# applications disappear from the build without any diagnostic. Builds that
# genuinely want no GUI opt out explicitly.
option(MRPT_IMGUI_VENDOR_ENABLE "Build the vendored ImGui stack (needs GLFW)" ON)

if(NOT MRPT_IMGUI_VENDOR_ENABLE)
  message(STATUS "mrpt_imgui_vendor: disabled via MRPT_IMGUI_VENDOR_ENABLE=OFF.")
  return()
endif()

find_package(glfw3 3.3 QUIET)
if(NOT glfw3_FOUND)
  message(FATAL_ERROR
    "mrpt_imgui_vendor requires GLFW >= 3.3, which was not found.\n"
    "  Debian/Ubuntu:  sudo apt install libglfw3-dev\n"
    "  macOS:          brew install glfw\n"
    "  Windows:        vcpkg install glfw3 --triplet x64-windows\n"
    "Configure with -DMRPT_IMGUI_VENDOR_ENABLE=OFF to build without the "
    "ImGui GUI stack instead.")
endif()

find_package(OpenGL REQUIRED)

# ---------------------------------------------------------------------------
# Library
# ---------------------------------------------------------------------------
set(LIB_SOURCES
  ${IMGUI_DIR}/imgui.cpp
  ${IMGUI_DIR}/imgui_draw.cpp
  ${IMGUI_DIR}/imgui_tables.cpp
  ${IMGUI_DIR}/imgui_widgets.cpp
  ${IMGUI_DIR}/misc/cpp/imgui_stdlib.cpp
  ${IMGUI_DIR}/backends/imgui_impl_glfw.cpp
  ${IMGUI_DIR}/backends/imgui_impl_opengl3.cpp
  ${IMPLOT_DIR}/implot.cpp
  ${IMPLOT_DIR}/implot_items.cpp
  src/embedded_icon_font.cpp
)

if(MRPT_IMGUI_VENDOR_WITH_DEMO)
  list(APPEND LIB_SOURCES
    ${IMGUI_DIR}/imgui_demo.cpp
    ${IMPLOT_DIR}/implot_demo.cpp
  )
endif()

set(LIB_PUBLIC_HEADERS
  include/mrpt/imgui_vendor/icon_font.h
)

# mrpt_add_library() creates the target via a bare add_library(), so the library
# type follows BUILD_SHARED_LIBS. Steer it for this project only.
set(BUILD_SHARED_LIBS ${MRPT_IMGUI_VENDOR_SHARED})

# Consumers of the *installed* package cannot see build-tree ALIAS targets, so
# the upstream-style names are recreated in the generated -config.cmake. An
# INTERFACE IMPORTED target may reference mrpt::mrpt_imgui_vendor before the
# -targets.cmake include: the link is only resolved at generate time.
set(EXTRA_CONFIG_CMDS "\
if(NOT TARGET imgui::imgui)
  add_library(imgui::imgui INTERFACE IMPORTED)
  set_target_properties(imgui::imgui PROPERTIES
    INTERFACE_LINK_LIBRARIES mrpt::mrpt_imgui_vendor)
endif()
if(NOT TARGET implot::implot)
  add_library(implot::implot INTERFACE IMPORTED)
  set_target_properties(implot::implot PROPERTIES
    INTERFACE_LINK_LIBRARIES mrpt::mrpt_imgui_vendor)
endif()")

mrpt_add_library(
  TARGET ${PROJECT_NAME}
  SOURCES ${LIB_SOURCES} ${LIB_PUBLIC_HEADERS}
  PUBLIC_LINK_LIBRARIES
    glfw
    OpenGL::GL
  CMAKE_DEPENDENCIES
    glfw3
    OpenGL
)

# Always PIC: a static vendor library gets linked into MRPT shared libraries.
set_target_properties(${PROJECT_NAME} PROPERTIES POSITION_INDEPENDENT_CODE TRUE)

# Upstream headers are included unqualified (`#include "imgui.h"`), so the
# directory holding them must itself be on the interface include path. SYSTEM
# keeps third-party warnings out of consumers' builds.
target_include_directories(${PROJECT_NAME} SYSTEM PUBLIC
  $<BUILD_INTERFACE:${IMGUI_DIR}>
  $<BUILD_INTERFACE:${IMGUI_DIR}/backends>
  $<BUILD_INTERFACE:${IMGUI_DIR}/misc/cpp>
  $<BUILD_INTERFACE:${IMPLOT_DIR}>
  $<BUILD_INTERFACE:${PFD_DIR}>
  $<BUILD_INTERFACE:${ICONFONTS_DIR}>
  $<INSTALL_INTERFACE:include/mrpt/imgui_vendor>
)

# Docking branch feature set. No GL-loader macro on purpose: imgui_impl_opengl3
# falls back to its bundled imgui_impl_opengl3_loader.h, so neither GLEW nor
# GLAD nor gl3w is needed.
target_compile_definitions(${PROJECT_NAME} PUBLIC IMGUI_ENABLE_DOCKING)

if(MRPT_IMGUI_VENDOR_WITH_DEMO)
  target_compile_definitions(${PROJECT_NAME} PUBLIC MRPT_IMGUI_VENDOR_HAS_DEMO)
endif()

# MRPT's warning level must not apply to upstream code.
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  target_compile_options(${PROJECT_NAME} PRIVATE -w)
elseif(MSVC)
  target_compile_options(${PROJECT_NAME} PRIVATE /W0)
endif()

set_target_properties(${PROJECT_NAME} PROPERTIES FOLDER "3rd party")

# Aliases so that code written against upstream conventions stays portable.
add_library(imgui::imgui ALIAS ${PROJECT_NAME})
add_library(implot::implot ALIAS ${PROJECT_NAME})

# ---------------------------------------------------------------------------
# Install the vendored headers under a dedicated prefix, so that flat names
# like imgui.h never land in a shared include/ directory.
# (mrpt_add_library already handles this package's own include/ tree.)
# ---------------------------------------------------------------------------
set(_hdr_dest ${CMAKE_INSTALL_INCLUDEDIR}/mrpt/imgui_vendor)

set(_backend_hdrs
  ${IMGUI_DIR}/backends/imgui_impl_glfw.h
  ${IMGUI_DIR}/backends/imgui_impl_opengl3.h
  ${IMGUI_DIR}/backends/imgui_impl_opengl3_loader.h
)

install(FILES
    ${IMGUI_DIR}/imgui.h
    ${IMGUI_DIR}/imconfig.h
    ${IMGUI_DIR}/imgui_internal.h
    ${IMGUI_DIR}/imstb_rectpack.h
    ${IMGUI_DIR}/imstb_textedit.h
    ${IMGUI_DIR}/imstb_truetype.h
    ${IMGUI_DIR}/misc/cpp/imgui_stdlib.h
    ${_backend_hdrs}
    ${IMPLOT_DIR}/implot.h
    ${IMPLOT_DIR}/implot_internal.h
    ${PFD_DIR}/portable-file-dialogs.h
    ${ICONFONTS_DIR}/IconsMaterialSymbols.h
  DESTINATION ${_hdr_dest}
)

# Upstream's own convention puts both the root and backends/ on the include
# path, so code in the wild writes either "imgui_impl_glfw.h" or
# "backends/imgui_impl_glfw.h". Install the sub-path spellings too, so that
# neither form has to be edited when switching to this package.
install(FILES ${_backend_hdrs} DESTINATION ${_hdr_dest}/backends)
install(FILES ${IMGUI_DIR}/misc/cpp/imgui_stdlib.h
        DESTINATION ${_hdr_dest}/misc/cpp)

unset(_backend_hdrs)

# Upstream license texts, next to the wrapper's own.
install(DIRECTORY LICENSES
  DESTINATION ${CMAKE_INSTALL_DATADIR}/${PROJECT_NAME}
)

unset(_hdr_dest)
