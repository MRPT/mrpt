# ----------------------------------------------------------------------------
#  Update the library version header file
#    FILE_TO_PARSE="SRC/include/mrpt/MRPT_version.h.in"
#    TARGET_FILE  ="MRPT_version.h"
# ----------------------------------------------------------------------------
SET(CMAKE_MRPT_COMPLETE_NAME "MRPT ${CMAKE_MRPT_VERSION_NUMBER_MAJOR}.${CMAKE_MRPT_VERSION_NUMBER_MINOR}.${CMAKE_MRPT_VERSION_NUMBER_PATCH}")
# Build a three digits version code, eg. 0.5.1 -> 051,  1.2.0 -> 120
SET(CMAKE_MRPT_VERSION_CODE "0x${CMAKE_MRPT_VERSION_NUMBER_MAJOR}${CMAKE_MRPT_VERSION_NUMBER_MINOR}${CMAKE_MRPT_VERSION_NUMBER_PATCH}")

CONFIGURE_FILE("${CMAKE_SOURCE_DIR}/parse-files/version.h.in" "${MRPT_CONFIG_FILE_INCLUDE_DIR}/mrpt/version.h")
