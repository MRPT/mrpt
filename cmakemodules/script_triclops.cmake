# Check for PointGreyResearch Triclops library
# -------------------------------------------------

SET(MRPT_HAS_PGR_TRICLOPS OFF CACHE BOOL "Build with support for Fly Capture 2 (PointGreyResearch Base C API")
SET(CMAKE_MRPT_HAS_TRICLOPS 0)

# Leave at the user's choice to disable it:
OPTION(DISABLE_PGR_TRICLOPS "Forces NOT using PGR FlyCapture2 libs, even if they are found by CMake" "OFF")
MARK_AS_ADVANCED(DISABLE_PGR_TRICLOPS)

IF(NOT DISABLE_PGR_TRICLOPS)	
	IF(MRPT_HAS_PGR_TRICLOPS)
		# Look for libs & headers:
		find_library(PGR_TRICLOPS_LIB_FILE_RELEASE
			NAMES triclops
			PATHS 
				"/usr/local/"
				"/usr/lib/"
				# TODO: Add typical Windows path
			DOC "Full path of library file 'libtriclops' or (TODO: WINDOWS)" )

		find_library(PGR_FC2BRIDGE_LIB_FILE_RELEASE
			NAMES flycapture2bridge
			PATHS 
				"/usr/local/"
				"/usr/lib/"
				# TODO: Add typical Windows path
			DOC "Full path of library file 'libflycapture2bridge' or (TODO: WINDOWS)" )

		find_library(PGR_PNMUTILS_LIB_FILE_RELEASE
			NAMES pnmutils
			PATHS 
				"/usr/local/"
				"/usr/lib/"
				# TODO: Add typical Windows path
			DOC "Full path of library file 'libpnmutils' or (TODO: WINDOWS)" )
		
	
		find_path(PGR_TRICLOPS_INCLUDE_DIR
			NAMES triclops.h
			PATHS 
				"/usr/local/"
				"/usr/include/triclops"
				"/usr/"
				# TODO: Add typical Windows path
			DOC "Path to [PATH]/triclops.h"
		)
		
		# The variable that actually ends up in config.h
		IF (PGR_TRICLOPS_INCLUDE_DIR AND PGR_TRICLOPS_LIB_FILE_RELEASE AND PGR_FC2BRIDGE_LIB_FILE_RELEASE AND PGR_PNMUTILS_LIB_FILE_RELEASE)
			SET(CMAKE_MRPT_HAS_TRICLOPS 1) 
		ELSE(PGR_TRICLOPS_INCLUDE_DIR AND PGR_TRICLOPS_LIB_FILE_RELEASE AND PGR_FC2BRIDGE_LIB_FILE_RELEASE AND PGR_PNMUTILS_LIB_FILE_RELEASE)
			MESSAGE("Error: Correct PGR_TRICLOPS_LIB_FILE_RELEASE and PGR_TRICLOPS_INCLUDE_DIR, or uncheck MRPT_HAS_PGR_TRICLOPS")
		ENDIF (PGR_TRICLOPS_INCLUDE_DIR AND PGR_TRICLOPS_LIB_FILE_RELEASE AND PGR_FC2BRIDGE_LIB_FILE_RELEASE AND PGR_PNMUTILS_LIB_FILE_RELEASE)
	ENDIF(MRPT_HAS_PGR_TRICLOPS)
ENDIF(NOT DISABLE_PGR_TRICLOPS)	
