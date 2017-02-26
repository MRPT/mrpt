# Support for phidget Interface Kit with proximity sensor device :
# =================================================================

OPTION(DISABLE_PHIDGETS "Forces NOT using PHIDGETSlibs, even if they are found by CMake" "OFF")
MARK_AS_ADVANCED(DISABLE_PHIDGETS)
SET(CMAKE_MRPT_HAS_PHIDGET 0)
SET(CMAKE_MRPT_HAS_PHIDGET_SYSTEM 0)

IF(NOT DISABLE_PHIDGETS)	
	find_package(PHIDGETS QUIET)
	## Defines:
	## - PHIDGETS_FOUND
	## - PHIDGETS_INCLUDE_DIR
	## - PHIDGETS_LIBRARIES

	IF(PHIDGETS_FOUND)
		SET(CMAKE_MRPT_HAS_PHIDGET 1)
		SET(CMAKE_MRPT_HAS_PHIDGET_SYSTEM 1)
	ENDIF(PHIDGETS_FOUND)
ENDIF()
