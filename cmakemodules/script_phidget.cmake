# Support for phidget Interface Kit with proximity sensor device :
# =================================================================

find_package(PHIDGETS QUIET)
## Defines:
## - PHIDGETS_FOUND
## - PHIDGETS_INCLUDE_DIR
## - PHIDGETS_LIBRARIES

SET(CMAKE_MRPT_HAS_PHIDGET 0)
SET(CMAKE_MRPT_HAS_PHIDGET_SYSTEM 0)

IF(PHIDGETS_FOUND)
	SET(CMAKE_MRPT_HAS_PHIDGET 1)
	SET(CMAKE_MRPT_HAS_PHIDGET_SYSTEM 1)
ENDIF(PHIDGETS_FOUND)

