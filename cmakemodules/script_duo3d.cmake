# Support for DUO3D stereo camera
# ==========================================================================================
IF(UNIX)
	SET( MRPT_HAS_DUO3D OFF CACHE INTERNAL "Has DUO3D libraries?" FORCE)
ELSE(UNIX)
	SET( MRPT_HAS_DUO3D OFF CACHE BOOL "Has DUO3D libraries?")

	IF( MRPT_HAS_DUO3D )
		SET( DUO3D_ROOT_DIR "" CACHE PATH "Path to DUO3D library root directory")

		IF(UNIX)
			MESSAGE("Sorry! DUO3D camera is supported only for Windows yet. Set MRPT_HAS_DUO3D to OFF")
			SET(CMAKE_MRPT_HAS_DUO3D 0)
		ELSE(UNIX)
			# Set to 1, next check for missing things and set to 0 on any error & report message:
			SET(CMAKE_MRPT_HAS_DUO3D 1)

			IF(NOT EXISTS ${DUO3D_ROOT_DIR})
				SET(CMAKE_MRPT_HAS_DUO3D 0)
				MESSAGE("The directory 'DUO3D_ROOT_DIR' does not exists. Turn off DUO3D support or provide the correct path.")
			ENDIF(NOT EXISTS ${DUO3D_ROOT_DIR})

			IF(NOT EXISTS ${DUO3D_ROOT_DIR}/include/DUOLib.h)
				SET(CMAKE_MRPT_HAS_DUO3D 0)
				MESSAGE("The directory 'DUO3D_ROOT_DIR' does not contain include/DUOLib.h. Turn off DUO3D support or provide the correct path.")
			ENDIF(NOT EXISTS ${DUO3D_ROOT_DIR}/include/DUOLib.h)

			IF(NOT EXISTS ${DUO3D_ROOT_DIR}/lib/DUOLib.lib)
				SET(CMAKE_MRPT_HAS_DUO3D 0)
				MESSAGE("The directory 'DUO3D_ROOT_DIR' does not contain lib/DUOLib.lib. Turn off DUO3D support or provide the correct path.")
			ENDIF(NOT EXISTS ${DUO3D_ROOT_DIR}/lib/DUOLib.lib)
		ENDIF(UNIX)

	ENDIF(MRPT_HAS_DUO3D)
ENDIF(UNIX)

