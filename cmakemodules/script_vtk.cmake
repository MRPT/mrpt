# Check forVTK
# ===================================================
SET(CMAKE_MRPT_HAS_VTK 0)

OPTION(DISABLE_VTK "Force not using VTK libs" "OFF")
MARK_AS_ADVANCED(DISABLE_VTK)
IF(NOT DISABLE_VTK)
	FIND_PACKAGE(VTK)
	IF (VTK_FOUND)
		# vtkStructuredGrid added after 5.10.0rc2 or above
		# constraint on the version - putting this in FIND_PACKAGE doesn't work.
		IF (${VTK_MAJOR_VERSION} LESS 5 OR (${VTK_MAJOR_VERSION} EQUAL 5 AND VTK_MINOR_VERSION LESS 10))
			MESSAGE(FATAL_ERROR "Current VTK version (${VTK_MAJOR_VERSION}.${VTK_MINOR_VERSION}.${VTK_BUILD_VERSION}) is less than version 5.10 (the required minimum). Exiting.")
		ENDIF()

		SET(CMAKE_MRPT_HAS_VTK 1)
	ENDIF()
ENDIF()
