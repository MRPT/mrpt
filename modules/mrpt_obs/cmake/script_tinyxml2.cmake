# Check for the TINYXML2 library
# ===================================================
set(CMAKE_MRPT_HAS_TINYXML2 0)
set(CMAKE_MRPT_HAS_TINYXML2_SYSTEM 0)

# This option will be available only on Linux, hence it's declared here:
option(DISABLE_TINYXML2 "Do not use the tinyxml2 library" 0)
mark_as_advanced(DISABLE_TINYXML2)

if (DISABLE_TINYXML2)
  return()
endif()

set(_use_embeded_default "OFF")
if (WIN32)
  set(_use_embeded_default "ON")
endif()

set(TINYXML2_USE_EMBEDDED_VERSION ${_use_embeded_default} CACHE BOOL "Download tinyxml2 and use it instead of system version")
mark_as_advanced(TINYXML2_USE_EMBEDDED_VERSION)

unset(_use_embeded_default)

# Try the system tinyxml2 first (works on macOS via brew, and Linux via pkg)
if(NOT TINYXML2_USE_EMBEDDED_VERSION)
  find_package(tinyxml2 QUIET)
  if(tinyxml2_FOUND)
    set(CMAKE_MRPT_HAS_TINYXML2 1)
    set(CMAKE_MRPT_HAS_TINYXML2_SYSTEM 1)
    if(NOT TARGET imp_tinyxml2)
      add_library(imp_tinyxml2 INTERFACE IMPORTED)
      set_target_properties(imp_tinyxml2
        PROPERTIES
        INTERFACE_LINK_LIBRARIES "tinyxml2::tinyxml2"
      )
    endif()
  endif()
endif()

if (TINYXML2_USE_EMBEDDED_VERSION AND NOT CMAKE_MRPT_HAS_TINYXML2)
  # download on the fly:
  set(TINYXML2_VERSION_TO_DOWNLOAD "7.1.0" CACHE STRING "Download from this GitHub tag")
  # Use CMAKE_CURRENT_BINARY_DIR so this works in standalone/colcon builds
  set(_tinyxml2_download_dir "${CMAKE_CURRENT_BINARY_DIR}/3rdparty/tinyxml2")

  # A failed download must not leave an empty file behind: it would compile
  # fine, and only fail later, at link time.
  foreach(_file tinyxml2.h tinyxml2.cpp)
    set(_dest "${_tinyxml2_download_dir}/${_file}")
    set(_size 0)
    if (EXISTS "${_dest}")
      file(SIZE "${_dest}" _size)
    endif()
    set(_attempt 0)
    while(_size EQUAL 0 AND _attempt LESS 3)
      math(EXPR _attempt "${_attempt} + 1")
      file(DOWNLOAD
        https://github.com/leethomason/tinyxml2/raw/${TINYXML2_VERSION_TO_DOWNLOAD}/${_file}
        "${_dest}" STATUS _status)
      list(GET _status 0 _status_code)
      if (_status_code EQUAL 0)
        file(SIZE "${_dest}" _size)
      else()
        message(STATUS "tinyxml2: downloading ${_file} failed (attempt ${_attempt}): ${_status}")
        file(REMOVE "${_dest}")
      endif()
    endwhile()
    if (_size EQUAL 0)
      message(FATAL_ERROR "tinyxml2: could not download ${_file}. Set TINYXML2_USE_EMBEDDED_VERSION=OFF to use a system tinyxml2, or DISABLE_TINYXML2=ON.")
    endif()
  endforeach()
  unset(_dest)
  unset(_size)
  unset(_attempt)
  unset(_status)
  unset(_status_code)

  set(CMAKE_MRPT_HAS_TINYXML2 1)
  set(CMAKE_MRPT_HAS_TINYXML2_SYSTEM 0)
  unset(_tinyxml2_download_dir)
endif()

if(UNIX AND NOT CMAKE_MRPT_HAS_TINYXML2)
  find_path(TINYXML2_INCLUDE_DIR tinyxml2.h)
  mark_as_advanced(TINYXML2_INCLUDE_DIR)

  find_library(TINYXML2_LIBRARY NAMES tinyxml2)
  mark_as_advanced(TINYXML2_LIBRARY)

  if(TINYXML2_INCLUDE_DIR AND TINYXML2_LIBRARY)
    set(CMAKE_MRPT_HAS_TINYXML2 1)
    set(CMAKE_MRPT_HAS_TINYXML2_SYSTEM 1)

    add_library(imp_tinyxml2 INTERFACE IMPORTED)
    set_target_properties(imp_tinyxml2
      PROPERTIES
      INTERFACE_INCLUDE_DIRECTORIES "${TINYXML2_INCLUDE_DIR}"
      INTERFACE_LINK_LIBRARIES "${TINYXML2_LIBRARY}"
      )
  endif()
endif()

if(${CMAKE_MRPT_HAS_TINYXML2} AND "$ENV{VERBOSE}")
  message(STATUS "libtinyxml2 configuration:")
  message(STATUS "  TINYXML2_INCLUDE_DIR: ${TINYXML2_INCLUDE_DIR}")
  message(STATUS "  TINYXML2_LIBRARY: ${TINYXML2_LIBRARY}")
endif()
