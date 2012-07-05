# Support for Videre Design stereo camera:
# ===================================================
SET( MRPT_HAS_SVS OFF CACHE BOOL "Add support for STOC Stereo Camera?")

IF(MRPT_HAS_SVS)
	SET( SVS_ROOT_DIR "" CACHE PATH "Path to SVS library root directory")


	IF(UNIX)
		# Set to 1, next check for missing things and set to 0 on any error & report message:
		SET(CMAKE_MRPT_HAS_SVS 1)

		IF(NOT EXISTS ${SVS_ROOT_DIR})
			SET(CMAKE_MRPT_HAS_SVS 0)
			MESSAGE("The directory 'SVS_ROOT_DIR' does not exists. Turn off SVS support or provide the correct path.")
		ENDIF(NOT EXISTS ${SVS_ROOT_DIR})

		IF(NOT EXISTS ${SVS_ROOT_DIR}/src/svsclass.h)
			SET(CMAKE_MRPT_HAS_SVS 0)
			MESSAGE("The directory 'SVS_ROOT_DIR' does not contain src/svsclass.h. Turn off SVS support or provide the correct path.")
		ENDIF(NOT EXISTS ${SVS_ROOT_DIR}/src/svsclass.h)

		IF(NOT EXISTS ${SVS_ROOT_DIR}/src/dcs.h)
			SET(CMAKE_MRPT_HAS_SVS 0)
			MESSAGE("The directory 'SVS_ROOT_DIR' does not contain src/dcs.h. Turn off SVS support or provide the correct path.")
		ENDIF(NOT EXISTS ${SVS_ROOT_DIR}/src/dcs.h)
	ELSE(UNIX)


		MESSAGE("Sorry! STOC camera is supported only for LINUX yet. Set MRPT_HAS_SVS to OFF")
		SET(CMAKE_MRPT_HAS_SVS 0)
	ENDIF(UNIX)
ENDIF(MRPT_HAS_SVS)

IF(CMAKE_MRPT_HAS_SVS)
	INCLUDE_DIRECTORIES("${SVS_ROOT_DIR}/src")
	LINK_DIRECTORIES("${SVS_ROOT_DIR}/bin")
ENDIF(CMAKE_MRPT_HAS_SVS)

# This can only be a system lib:
SET(CMAKE_MRPT_HAS_SVS_SYSTEM 0)
IF(CMAKE_MRPT_HAS_SVS)
	SET(CMAKE_MRPT_HAS_SVS_SYSTEM 1)
ENDIF(CMAKE_MRPT_HAS_SVS)

