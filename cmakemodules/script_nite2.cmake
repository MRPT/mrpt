# Support for NITE2 Library (only for windows -- by now --)
# ==========================================================================================
SET(CMAKE_MRPT_HAS_NITE2 0)
IF(UNIX)
	SET( MRPT_HAS_NITE2 OFF CACHE INTERNAL "Has NITE2 Windows libraries?" FORCE)
ELSE(UNIX)
	SET( MRPT_HAS_NITE2 OFF CACHE BOOL "Has NITE2 Windows libraries?")

	IF(MRPT_HAS_NITE2)
		SET( NITE2_ROOT_DIR "" CACHE PATH "Path to NITE2 library root directory")

		IF(UNIX)
			MESSAGE("Sorry! NITE2 is supported only for Windows yet. Set MRPT_HAS_NITE2 to OFF")
			SET(CMAKE_MRPT_HAS_NITE2 0)
		ELSE(UNIX)
			# Set to 1, next check for missing things and set to 0 on any error & report message:
			SET(CMAKE_MRPT_HAS_NITE2 1)

			IF(NOT EXISTS ${NITE2_ROOT_DIR})
				SET(CMAKE_MRPT_HAS_NITE2 0)
				MESSAGE("The directory 'NITE2_ROOT_DIR' does not exists. Turn off NITE2 support or provide the correct path.")
			ENDIF(NOT EXISTS ${NITE2_ROOT_DIR})

			IF(NOT EXISTS ${NITE2_ROOT_DIR}/Include/NiTE.h)
				SET(CMAKE_MRPT_HAS_NITE2 0)
				MESSAGE("The directory 'NITE2_ROOT_DIR' does not contain Include/NiTE.h. Turn off NITE2 support or provide the correct path.")
			ENDIF(NOT EXISTS ${NITE2_ROOT_DIR}/Include/NiTE.h)

			IF(NOT EXISTS ${NITE2_ROOT_DIR}/Lib/NiTE2.lib)
				SET(CMAKE_MRPT_HAS_NITE2 0)
				MESSAGE("The directory 'NITE2_ROOT_DIR' does not contain Lib/NiTE2.lib. Turn off NITE2 support or provide the correct path.")
			ENDIF(NOT EXISTS ${NITE2_ROOT_DIR}/Lib/NiTE2.lib)

		ENDIF(UNIX)

	ENDIF(MRPT_HAS_NITE2)
ENDIF(UNIX)

IF(CMAKE_MRPT_HAS_NITE2)
	INCLUDE_DIRECTORIES("${NITE2_ROOT_DIR}/Include")
	LINK_DIRECTORIES("${NITE2_ROOT_DIR}/Lib")
ENDIF(CMAKE_MRPT_HAS_NITE2)
