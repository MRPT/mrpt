# Support for INTERSENSE stereo camera
# ==========================================================================================
set(CMAKE_MRPT_HAS_INTERSENSE 0)

set( MRPT_HAS_INTERSENSE OFF CACHE BOOL "Build with support for Intersense libraries?")

if( MRPT_HAS_INTERSENSE )
	if(UNIX)
		message("Sorry! INTERSENSE sensor is supported only for Windows yet. Set MRPT_HAS_INTERSENSE to OFF")
	else()
		# Set to 1, next check for missing things and set to 0 on any error & report message:
		set(CMAKE_MRPT_HAS_INTERSENSE 1)
	endif()
endif()
