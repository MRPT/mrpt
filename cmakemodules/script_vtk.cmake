# Check forVTK
# ===================================================
SET(CMAKE_MRPT_HAS_VTK 0)

OPTION(DISABLE_VTK "Force not using VTK libs" "OFF")
MARK_AS_ADVANCED(DISABLE_VTK)
IF(NOT DISABLE_VTK)

	FIND_PACKAGE(VTK QUIET)
	IF (VTK_FOUND)
		SET(CMAKE_MRPT_HAS_VTK 1)
	ENDIF()
ENDIF()
