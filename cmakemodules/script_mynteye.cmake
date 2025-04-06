# Support for MYNT EYE depth cameras
# ==========================================================================================
set(CMAKE_MRPT_HAS_MYNTEYE_D 0)  # Set default value

find_package(mynteyed QUIET)
mark_as_advanced(mynteyed)

if (NOT mynteyed_FOUND)
	return()
endif()

set(CMAKE_MRPT_HAS_MYNTEYE_D 1)
