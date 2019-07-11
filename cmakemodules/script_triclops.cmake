# Check for FLIR/PointGreyResearch Triclops library
# --------------------------------------------------

set(CMAKE_MRPT_HAS_TRICLOPS 0)

# Leave at the user's choice to disable it:
option(DISABLE_PGR_TRICLOPS "Forces NOT using Flir FlyCapture2 libs, even if they are found by CMake" "OFF")
mark_as_advanced(DISABLE_PGR_TRICLOPS)

if(NOT DISABLE_PGR_TRICLOPS)
	# Look for libs & headers:
	find_library(PGR_TRICLOPS_LIB_FILE_RELEASE
		NAMES triclops4
		PATHS
			"/usr/local/"
			"/usr/lib/"
			# TODO: Add typical Windows path
		DOC "Full path of library file 'libtriclops'" )

	find_library(PGR_FC2BRIDGE_LIB_FILE_RELEASE
		NAMES flycapture2bridge4
		PATHS
			"/usr/local/"
			"/usr/lib/"
			# TODO: Add typical Windows path
		DOC "Full path of library file 'libflycapture2bridge'" )


	find_path(PGR_TRICLOPS_INCLUDE_DIR
		NAMES triclops.h
		PATHS
			"/usr/local/"
			"/usr/include/triclops4"
			"/usr/"
			# TODO: Add typical Windows path
		DOC "Path to [PATH]/triclops.h"
	)

	# The variable that actually ends up in config.h
	if (PGR_TRICLOPS_INCLUDE_DIR AND
			PGR_TRICLOPS_LIB_FILE_RELEASE AND
			PGR_FC2BRIDGE_LIB_FILE_RELEASE
		)
		set(CMAKE_MRPT_HAS_TRICLOPS 1)
	endif ()
endif()
