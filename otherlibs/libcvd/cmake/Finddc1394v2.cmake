#.rst:
# Finddc1394
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

find_path(dc1394v2_INCLUDE_DIR NAMES dc1394/control.h PATH_SUFFIXES include)

find_library(dc1394v2_LIBRARY NAMES dc1394 PATH_SUFFIXES lib)
find_library(raw1394_LIBRARY NAMES raw1394 PATH_SUFFIXES lib)
FIND_PACKAGE_HANDLE_STANDARD_ARGS(dc1394 REQUIRED_VARS dc1394v2_INCLUDE_DIR dc1394v2_LIBRARY raw1394_LIBRARY)

if(dc1394_FOUND)
    set(dc1394_INCLUDE_DIRS ${dc1394_INCLUDE_DIR})
    set(dc1394_LIBRARIES ${dc1394v2_LIBRARY} ${raw1394_LIBRARY})
endif()

mark_as_advanced(dc1394_INCLUDE_DIR dc1394v2_LIBRARY)
