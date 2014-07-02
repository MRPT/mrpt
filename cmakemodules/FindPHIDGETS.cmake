## FindPHIDGETS: find the Phidget library
## Copyright (C) 2014 Jose-Luis Blanco-Claraco
##
## Returns these variables (with the standard semantics in CMake modules):
##
## - PHIDGETS_FOUND
## - PHIDGETS_INCLUDE_DIR
## - PHIDGETS_LIBRARIES
##


# Look for libs & headers:
find_library(PHIDGETS_LIBRARIES
	NAMES phidget21 
	PATHS 
		"/usr/"
		"/usr/local/"
		"$ENV{ProgramFiles}/Phidgets/lib"
	DOC "Full path of library file 'phidget21.so' or 'phidget21.lib'" )

# Help find the .h files:
IF (EXISTS ${PHIDGETS_LIBRARIES})
	get_filename_component(AUX_PARENT_DIR1 ${PHIDGETS_LIBRARIES} PATH)
	get_filename_component(AUX_PARENT_DIR2 ${AUX_PARENT_DIR1} PATH)
ENDIF (EXISTS ${PHIDGETS_LIBRARIES})
	
find_path(PHIDGETS_INCLUDE_DIR
	NAMES phidget21.h
	PATHS 
		"/usr/local/include"
		"/usr/include/"
		"$ENV{ProgramFiles}/Phidgets/include"
		"${AUX_PARENT_DIR1}" "${AUX_PARENT_DIR2}"
	DOC "Path to [PATH]/phidget21.h"
)

set(PHIDGETS_FOUND "NO")
if(PHIDGETS_LIBRARIES AND PHIDGETS_INCLUDE_DIR)
	set(PHIDGETS_FOUND "Yes")
	IF ($ENV{VERBOSE})
		MESSAGE(STATUS "PHIDGETS_LIBRARIES: ${PHIDGETS_LIBRARIES}")
		MESSAGE(STATUS "PHIDGETS_INCLUDE_DIR: ${PHIDGETS_INCLUDE_DIR}")
	ENDIF ($ENV{VERBOSE})

	
	MARK_AS_ADVANCED(PHIDGETS_INCLUDE_DIR)
	MARK_AS_ADVANCED(PHIDGETS_LIBRARIES)
endif(PHIDGETS_LIBRARIES AND PHIDGETS_INCLUDE_DIR)
