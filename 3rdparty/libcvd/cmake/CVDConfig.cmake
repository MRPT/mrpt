#.rst:
# CVDConfig
# --------
#
# Find the native CVD includes and library.
#
# Result Variables
# ^^^^^^^^^^^^^^^^
#
# This module defines the following variables:
#
# ::
#
#   CVD_INCLUDE_DIRS   - where to find cvd/*.h.
#   CVD_LIBRARIES      - List of libraries when using CVD.
#   CVD_FOUND          - True if CVD found.

find_path(CVD_INCLUDE_DIR NAMES cvd/config.h PATH_SUFFIXES include)
find_library(CVD_LIBRARY_RELEASE NAMES cvd PATH_SUFFIXES lib)
find_library(CVD_LIBRARY_DEBUG NAMES cvd_debug PATH_SUFFIXES lib)

select_library_configurations(CVD)
mark_as_advanced(CVD_LIBRARY_RELEASE CVD_LIBRARY_DEBUG)

FIND_PACKAGE_HANDLE_STANDARD_ARGS(CVD REQUIRED_VARS CVD_LIBRARY CVD_INCLUDE_DIR)

if(CVD_FOUND)
    set(CVD_INCLUDE_DIRS ${CVD_INCLUDE_DIR})
    set(CVD_LIBRARIES ${CVD_LIBRARY})
endif()

mark_as_advanced(CVD_LIBRARY CVD_INCLUDE_DIR )
