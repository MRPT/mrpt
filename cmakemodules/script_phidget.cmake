# Support for phidget Interface Kit with proximity sensor device :
# =================================================================

option(DISABLE_PHIDGETS "Forces NOT using PHIDGETSlibs, even if they are found by CMake" "OFF")
mark_as_advanced(DISABLE_PHIDGETS)
set(CMAKE_MRPT_HAS_PHIDGET 0)
set(CMAKE_MRPT_HAS_PHIDGET_SYSTEM 0)

if(NOT DISABLE_PHIDGETS)	
	find_package(PHIDGETS QUIET)
	## Defines:
	## - PHIDGETS_FOUND
	## - PHIDGETS_INCLUDE_DIR
	## - PHIDGETS_LIBRARIES

	if(PHIDGETS_FOUND)
		set(CMAKE_MRPT_HAS_PHIDGET 1)
		set(CMAKE_MRPT_HAS_PHIDGET_SYSTEM 1)
	endif(PHIDGETS_FOUND)
endif()
