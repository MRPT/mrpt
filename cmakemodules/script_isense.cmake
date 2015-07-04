# Support for INTERSENSE stereo camera
# ==========================================================================================
SET(CMAKE_MRPT_HAS_INTERSENSE 0)  # Set default value (it cannot be empty!)

SET( MRPT_HAS_INTERSENSE OFF CACHE BOOL "Build with support for Intersense libraries?")

IF( MRPT_HAS_INTERSENSE )
	IF(UNIX)
		MESSAGE("Sorry! INTERSENSE sensor is supported only for Windows yet. Set MRPT_HAS_INTERSENSE to OFF")
	ELSE(UNIX)
		# Set to 1, next check for missing things and set to 0 on any error & report message:
		SET(CMAKE_MRPT_HAS_INTERSENSE 1)
	ENDIF(UNIX)
ENDIF(MRPT_HAS_INTERSENSE)
