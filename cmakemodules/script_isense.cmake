# Support for INTERSENSE stereo camera
# ==========================================================================================
set(CMAKE_MRPT_HAS_INTERSENSE 0)  # Set default value (it cannot be empty!)

set( MRPT_HAS_INTERSENSE OFF CACHE BOOL "Build with support for Intersense libraries?")

if( MRPT_HAS_INTERSENSE )
	if(UNIX)
		message("Sorry! INTERSENSE sensor is supported only for Windows yet. Set MRPT_HAS_INTERSENSE to OFF")
	else(UNIX)
		# Set to 1, next check for missing things and set to 0 on any error & report message:
		set(CMAKE_MRPT_HAS_INTERSENSE 1)
	endif(UNIX)
endif(MRPT_HAS_INTERSENSE)
