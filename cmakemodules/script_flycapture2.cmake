# Check for PointGreyResearch FlyCapture2 library
# -------------------------------------------------

SET(MRPT_HAS_PGR_FLYCAPTURE2 OFF CACHE BOOL "Build with support for Fly Capture 2 (PointGreyResearch Base C API")
SET(CMAKE_MRPT_HAS_FLYCAPTURE2 0)

# Leave at the user's choice to disable it:
OPTION(DISABLE_PGR_FLYCAPTURE2 "Forces NOT using PGR FlyCapture2 libs, even if they are found by CMake" "OFF")
MARK_AS_ADVANCED(DISABLE_PGR_FLYCAPTURE2)

IF(NOT DISABLE_PGR_FLYCAPTURE2)	
	IF(MRPT_HAS_PGR_FLYCAPTURE2)
		# Look for libs & headers:
		find_library(PGR_FLYCAPTURE2_LIB_FILE_RELEASE
			NAMES FlyCapture2 
			PATHS 
				"/usr/local/"
				"$ENV{ProgramFiles}/Point Grey Research/FlyCapture2/lib64"
				"$ENV{ProgramFiles}/Point Grey Research/FlyCapture2/lib"
				"$ENV{ProgramFiles(x86)}/Point Grey Research/FlyCapture2/lib"
			DOC "Full path of library file 'libflycapture2' or 'FlyCapture2.lib'" )
		
		find_library(PGR_FLYCAPTURE2_LIB_FILE_DEBUG
			NAMES FlyCapture2d 
			PATHS 
				"/usr/local/"
				"$ENV{ProgramFiles}/Point Grey Research/FlyCapture2/lib64"
				"$ENV{ProgramFiles}/Point Grey Research/FlyCapture2/lib"
				"$ENV{ProgramFiles(x86)}/Point Grey Research/FlyCapture2/lib"
			DOC "Full path of library file 'libflycapture2d' or 'FlyCapture2d.lib'" )
		
		find_path(PGR_FLYCAPTURE2_INCLUDE_DIR
			NAMES FlyCapture2.h
			PATHS 
				"/usr/local/"
				"/usr/"
				"$ENV{ProgramFiles}/Point Grey Research/FlyCapture2/include"
				"$ENV{ProgramFiles(x86)}/Point Grey Research/FlyCapture2/include"
				"$ENV{ProgramW6432}/Point Grey Research/FlyCapture2/include"
			DOC "Path to [PATH]/FlyCapture2.h"
		)
		
		# The variable that actually ends up in config.h
		IF (PGR_FLYCAPTURE2_INCLUDE_DIR AND PGR_FLYCAPTURE2_LIB_FILE_RELEASE AND PGR_FLYCAPTURE2_LIB_FILE_DEBUG)
			SET(CMAKE_MRPT_HAS_FLYCAPTURE2 1) 
		ELSE(PGR_FLYCAPTURE2_INCLUDE_DIR AND PGR_FLYCAPTURE2_LIB_FILE_RELEASE AND PGR_FLYCAPTURE2_LIB_FILE_DEBUG)
			MESSAGE("Error: Correct PGR_FLYCAPTURE2_LIB_FILE_* and PGR_FLYCAPTURE2_INCLUDE_DIR, or uncheck MRPT_HAS_PGR_FLYCAPTURE2")
		ENDIF (PGR_FLYCAPTURE2_INCLUDE_DIR AND PGR_FLYCAPTURE2_LIB_FILE_RELEASE AND PGR_FLYCAPTURE2_LIB_FILE_DEBUG)
	ENDIF(MRPT_HAS_PGR_FLYCAPTURE2)
ENDIF(NOT DISABLE_PGR_FLYCAPTURE2)	
