# Check for libdc1394-2 (Only in Linux, there is no
#  Windows version yet...)
# ===================================================
set(CMAKE_MRPT_HAS_LIBDC1394_2 0)
set(CMAKE_MRPT_HAS_LIBDC1394_2_SYSTEM 0)

# DISABLE_LIBDC1394
# ---------------------
option(DISABLE_LIBDC1394 "Force not using libdc1394" "OFF")
mark_as_advanced(DISABLE_LIBDC1394)
if(NOT DISABLE_LIBDC1394)


# Invoke pkg-config for getting the configuration:
if(PKG_CONFIG_FOUND)
	PKG_CHECK_MODULES(LIBDC1394_2 QUIET libdc1394-2)
	if (LIBDC1394_2_FOUND)
		set(CMAKE_MRPT_HAS_LIBDC1394_2 1)
		set(CMAKE_MRPT_HAS_LIBDC1394_2_SYSTEM 1)
		#message(STATUS "LIBDC1394_2_LIBRARIES    : ${LIBDC1394_2_LIBRARIES}")
		#message(STATUS "LIBDC1394_2_INCLUDE_DIRS : ${LIBDC1394_2_INCLUDE_DIRS}")
		#message(STATUS "LIBDC1394_2_LIBRARY_DIRS : ${LIBDC1394_2_LIBRARY_DIRS}")
		#message(STATUS "LIBDC1394_2_LDFLAGS      : ${LIBDC1394_2_LDFLAGS}")
	else(LIBDC1394_2_FOUND)
		set(CMAKE_MRPT_HAS_LIBDC1394_2 0)
	endif (LIBDC1394_2_FOUND)
endif(PKG_CONFIG_FOUND)

endif(NOT DISABLE_LIBDC1394)

