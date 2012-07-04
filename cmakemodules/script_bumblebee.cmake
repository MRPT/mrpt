# Support for Bumblebee stereo camera (actually: PGR Digiclops/Triclops Windows libraries)
# ==========================================================================================
IF(UNIX)
	SET( MRPT_HAS_BUMBLEBEE OFF CACHE INTERNAL "Has PGR Digiclops/Triclops Windows libraries?" FORCE)
ELSE(UNIX)
	SET( MRPT_HAS_BUMBLEBEE OFF CACHE BOOL "Has PGR Digiclops/Triclops Windows libraries?")

	IF(MRPT_HAS_BUMBLEBEE)
		SET( BUMBLEBEE_PGR_FLYCAPTURE_ROOT_DIR "" CACHE PATH "Path to PGR FlyCapture library root directory")
		SET( BUMBLEBEE_TRICLOPS_ROOT_DIR "" CACHE PATH "Path to triclops library root directory")

		IF(UNIX)
			MESSAGE("Sorry! Bumblebee camera is supported only for Windows yet. Set MRPT_HAS_BUMBLEBEE to OFF")
			SET(CMAKE_MRPT_HAS_BUMBLEBEE 0)
		ELSE(UNIX)
			# Set to 1, next check for missing things and set to 0 on any error & report message:
			SET(CMAKE_MRPT_HAS_BUMBLEBEE 1)

			IF(NOT EXISTS ${BUMBLEBEE_PGR_FLYCAPTURE_ROOT_DIR})
				SET(CMAKE_MRPT_HAS_BUMBLEBEE 0)
				MESSAGE("The directory 'BUMBLEBEE_PGR_FLYCAPTURE_ROOT_DIR' does not exists. Turn off BUMBLEBEE support or provide the correct path.")
			ENDIF(NOT EXISTS ${BUMBLEBEE_PGR_FLYCAPTURE_ROOT_DIR})

			IF(NOT EXISTS ${BUMBLEBEE_PGR_FLYCAPTURE_ROOT_DIR}/include/PGRFlyCapture.h)
				SET(CMAKE_MRPT_HAS_BUMBLEBEE 0)
				MESSAGE("The directory 'BUMBLEBEE_PGR_FLYCAPTURE_ROOT_DIR' does not contain include/PGRFlyCapture.h. Turn off BUMBLEBEE support or provide the correct path.")
			ENDIF(NOT EXISTS ${BUMBLEBEE_PGR_FLYCAPTURE_ROOT_DIR}/include/PGRFlyCapture.h)

			IF(NOT EXISTS ${BUMBLEBEE_PGR_FLYCAPTURE_ROOT_DIR}/lib/PGRFlyCapture.lib)
				SET(CMAKE_MRPT_HAS_BUMBLEBEE 0)
				MESSAGE("The directory 'BUMBLEBEE_PGR_FLYCAPTURE_ROOT_DIR' does not contain lib/PGRFlyCapture.LIB. Turn off BUMBLEBEE support or provide the correct path.")
			ENDIF(NOT EXISTS ${BUMBLEBEE_PGR_FLYCAPTURE_ROOT_DIR}/lib/PGRFlyCapture.lib)

			# ----

			IF(NOT EXISTS ${BUMBLEBEE_TRICLOPS_ROOT_DIR})
				SET(CMAKE_MRPT_HAS_BUMBLEBEE 0)
				MESSAGE("The directory 'BUMBLEBEE_TRICLOPS_ROOT_DIR' does not exists. Turn off BUMBLEBEE support or provide the correct path.")
			ENDIF(NOT EXISTS ${BUMBLEBEE_TRICLOPS_ROOT_DIR})

			IF(NOT EXISTS ${BUMBLEBEE_TRICLOPS_ROOT_DIR}/include/triclops.h)
				SET(CMAKE_MRPT_HAS_BUMBLEBEE 0)
				MESSAGE("The directory 'BUMBLEBEE_TRICLOPS_ROOT_DIR' does not contain include/triclops.h. Turn off BUMBLEBEE support or provide the correct path.")
			ENDIF(NOT EXISTS ${BUMBLEBEE_TRICLOPS_ROOT_DIR}/include/triclops.h)

			IF(NOT EXISTS ${BUMBLEBEE_TRICLOPS_ROOT_DIR}/lib/triclops.lib)
				SET(CMAKE_MRPT_HAS_BUMBLEBEE 0)
				MESSAGE("The directory 'BUMBLEBEE_TRICLOPS_ROOT_DIR' does not contain lib/triclops.lib. Turn off BUMBLEBEE support or provide the correct path.")
			ENDIF(NOT EXISTS ${BUMBLEBEE_TRICLOPS_ROOT_DIR}/lib/triclops.lib)
		ENDIF(UNIX)

	ENDIF(MRPT_HAS_BUMBLEBEE)
ENDIF(UNIX)

IF(CMAKE_MRPT_HAS_BUMBLEBEE)
	INCLUDE_DIRECTORIES("${BUMBLEBEE_PGR_FLYCAPTURE_ROOT_DIR}/include")
	INCLUDE_DIRECTORIES("${BUMBLEBEE_TRICLOPS_ROOT_DIR}/include")

	LINK_DIRECTORIES("${BUMBLEBEE_PGR_FLYCAPTURE_ROOT_DIR}/lib")
	LINK_DIRECTORIES("${BUMBLEBEE_TRICLOPS_ROOT_DIR}/lib")
ENDIF(CMAKE_MRPT_HAS_BUMBLEBEE)
