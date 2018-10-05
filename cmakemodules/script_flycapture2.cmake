# Check for PointGreyResearch FlyCapture2 library
# -------------------------------------------------

set(MRPT_HAS_PGR_FLYCAPTURE2 OFF CACHE BOOL "Build with support for Fly Capture 2 (PointGreyResearch Base C API")
set(CMAKE_MRPT_HAS_FLYCAPTURE2 0)

# Leave at the user's choice to disable it:
option(DISABLE_PGR_FLYCAPTURE2 "Forces NOT using PGR FlyCapture2 libs, even if they are found by CMake" "OFF")
mark_as_advanced(DISABLE_PGR_FLYCAPTURE2)

if(NOT DISABLE_PGR_FLYCAPTURE2)	
	if(MRPT_HAS_PGR_FLYCAPTURE2)
		# Look for libs & headers:
		find_library(PGR_FLYCAPTURE2_LIB_FILE_RELEASE
			NAMES flycapture FlyCapture2
			PATHS 
				"/usr/local/"
				"/usr/lib/"
				"$ENV{ProgramFiles}/Point Grey Research/FlyCapture2/lib64"
				"$ENV{ProgramFiles}/Point Grey Research/FlyCapture2/lib"
				"$ENV{ProgramFiles(x86)}/Point Grey Research/FlyCapture2/lib"
			DOC "Full path of library file 'libflycapture2' or 'FlyCapture2.lib'" )
		
	
		find_path(PGR_FLYCAPTURE2_INCLUDE_DIR
			NAMES FlyCapture2.h
			PATHS 
				"/usr/local/"
				"/usr/include/flycapture"
				"/usr/"
				"$ENV{ProgramFiles}/Point Grey Research/FlyCapture2/include"
				"$ENV{ProgramFiles(x86)}/Point Grey Research/FlyCapture2/include"
				"$ENV{ProgramW6432}/Point Grey Research/FlyCapture2/include"
			DOC "Path to [PATH]/FlyCapture2.h"
		)
		
		# The variable that actually ends up in config.h
		if (PGR_FLYCAPTURE2_INCLUDE_DIR AND PGR_FLYCAPTURE2_LIB_FILE_RELEASE)
			set(CMAKE_MRPT_HAS_FLYCAPTURE2 1) 
		else(PGR_FLYCAPTURE2_INCLUDE_DIR AND PGR_FLYCAPTURE2_LIB_FILE_RELEASE)
			message("Error: Correct PGR_FLYCAPTURE2_LIB_FILE_RELEASE and PGR_FLYCAPTURE2_INCLUDE_DIR, or uncheck MRPT_HAS_PGR_FLYCAPTURE2")
		endif (PGR_FLYCAPTURE2_INCLUDE_DIR AND PGR_FLYCAPTURE2_LIB_FILE_RELEASE)
	endif(MRPT_HAS_PGR_FLYCAPTURE2)
endif(NOT DISABLE_PGR_FLYCAPTURE2)	
