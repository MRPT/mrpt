#.rst:
# FindTooN
# --------
#
# Find the native TooN includes.
#
# Result Variables
# ^^^^^^^^^^^^^^^^
#
# This module defines the following variables:
#
# ::
#
#   TooN_INCLUDE_DIRS   - where to find the TooN includes.
#   TooN_FOUND          - True if TooN found.

find_path(TooN_INCLUDE_DIR NAMES TooN/TooN.h PATH_SUFFIXES include)

FIND_PACKAGE_HANDLE_STANDARD_ARGS(TooN REQUIRED_VARS TooN_INCLUDE_DIR)

if(TooN_FOUND)
    set(TooN_INCLUDE_DIRS ${TooN_INCLUDE_DIR})
endif()

mark_as_advanced(TooN_INCLUDE_DIR )
