# FindEGL.cmake — locate the EGL library and headers
# Self-contained: uses only standard CMake modules (no KDE ECM required).
#
# Defines:
#   EGL_FOUND            — TRUE if EGL is usable
#   EGL_INCLUDE_DIRS     — header search paths
#   EGL_LIBRARIES        — libraries to link
#   EGL_VERSION          — API version read from EGL/egl.h
#   EGL::EGL             — imported target (preferred)
#
# SPDX-License-Identifier: BSD-3-Clause

include(FindPackageHandleStandardArgs)
include(CheckCXXSourceCompiles)
include(CMakePushCheckState)

# 1. Try pkg-config first — it knows distro install prefixes.
find_package(PkgConfig QUIET)
if(PKG_CONFIG_FOUND)
  pkg_check_modules(PKG_EGL QUIET egl)
endif()

# 2. Locate the header.
find_path(EGL_INCLUDE_DIR
  NAMES EGL/egl.h
  HINTS ${PKG_EGL_INCLUDE_DIRS}
)

# 3. Locate the library (not needed on Emscripten — it's built-in).
if(NOT EMSCRIPTEN)
  find_library(EGL_LIBRARY
    NAMES EGL libEGL
    HINTS ${PKG_EGL_LIBRARY_DIRS}
  )
endif()

# 4. Parse the API version from the header.
if(EGL_INCLUDE_DIR AND EXISTS "${EGL_INCLUDE_DIR}/EGL/egl.h")
  file(READ "${EGL_INCLUDE_DIR}/EGL/egl.h" _egl_header)
  string(REGEX MATCHALL "[ \t]EGL_VERSION_[0-9_]+" _egl_ver_lines "${_egl_header}")
  unset(_egl_header)
  foreach(_line ${_egl_ver_lines})
    string(REGEX REPLACE "[ \t]EGL_VERSION_([0-9_]+)" "\\1" _cand "${_line}")
    string(REPLACE "_" "." _cand "${_cand}")
    if(NOT DEFINED EGL_VERSION OR EGL_VERSION VERSION_LESS _cand)
      set(EGL_VERSION "${_cand}")
    endif()
  endforeach()
endif()

# 5. Verify the header actually compiles (catches mismatched sysroots, etc.).
cmake_push_check_state(RESET)
if(EGL_LIBRARY)
  list(APPEND CMAKE_REQUIRED_LIBRARIES "${EGL_LIBRARY}")
endif()
if(EGL_INCLUDE_DIR)
  list(APPEND CMAKE_REQUIRED_INCLUDES "${EGL_INCLUDE_DIR}")
endif()
check_cxx_source_compiles("
#include <EGL/egl.h>
int main() {
  EGLint x = 0; EGLDisplay dpy = 0; EGLContext ctx = 0;
  eglDestroyContext(dpy, ctx);
}" _EGL_COMPILES)
cmake_pop_check_state()

# 6. Decide what REQUIRED_VARS to check.
set(_egl_required EGL_INCLUDE_DIR _EGL_COMPILES)
if(NOT EMSCRIPTEN)
  list(APPEND _egl_required EGL_LIBRARY)
endif()

find_package_handle_standard_args(EGL
  FOUND_VAR    EGL_FOUND
  REQUIRED_VARS ${_egl_required}
  VERSION_VAR  EGL_VERSION
)

# 7. Create the imported target.
if(EGL_FOUND AND NOT TARGET EGL::EGL)
  if(EMSCRIPTEN)
    add_library(EGL::EGL INTERFACE IMPORTED)
  else()
    add_library(EGL::EGL UNKNOWN IMPORTED)
    set_target_properties(EGL::EGL PROPERTIES
      IMPORTED_LOCATION             "${EGL_LIBRARY}"
      INTERFACE_COMPILE_OPTIONS     "${PKG_EGL_CFLAGS_OTHER}"
      INTERFACE_INCLUDE_DIRECTORIES "${EGL_INCLUDE_DIR}"
    )
  endif()
endif()

mark_as_advanced(EGL_INCLUDE_DIR EGL_LIBRARY _EGL_COMPILES)

# Compatibility aliases
set(EGL_LIBRARIES    ${EGL_LIBRARY})
set(EGL_INCLUDE_DIRS ${EGL_INCLUDE_DIR})
set(EGL_VERSION_STRING ${EGL_VERSION})
