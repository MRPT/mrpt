# Support for DUO3D stereo camera
# ==========================================================================================
SET(CMAKE_MRPT_HAS_DUO3D 0)  # Set default value (it cannot be empty!)

SET( MRPT_HAS_DUO3D OFF CACHE BOOL "Build with support for DUO3D libraries?")

IF( MRPT_HAS_DUO3D )
	IF(UNIX)
		MESSAGE("Sorry! DUO3D camera is supported only for Windows yet. Set MRPT_HAS_DUO3D to OFF")
	ELSE(UNIX)
	
			SET( DUO3D_INCLUDE_DIR "" CACHE PATH "Path to DUO3D include directory" )
			SET( DUO3D_LIB_DIR "" CACHE PATH "Path to DUO3D library directory" )

			# Set to 1, next check for missing things and set to 0 on any error & report message:
			SET(CMAKE_MRPT_HAS_DUO3D 1)

			# IF(NOT EXISTS ${DUO3D_ROOT_DIR})
				# SET(CMAKE_MRPT_HAS_DUO3D 0)
				# MESSAGE("The directory 'DUO3D_ROOT_DIR' does not exists. Turn off DUO3D support or provide the correct path.")
			# ENDIF(NOT EXISTS ${DUO3D_ROOT_DIR})

			IF(NOT EXISTS ${DUO3D_INCLUDE_DIR}/DUOLib.h)
				SET(CMAKE_MRPT_HAS_DUO3D 0)
				MESSAGE("The directory 'DUO3D_INCLUDE_DIR' does not contain DUOLib.h. Turn off DUO3D support or provide the correct path.")
			ENDIF(NOT EXISTS ${DUO3D_INCLUDE_DIR}/DUOLib.h)

			IF(NOT EXISTS ${DUO3D_LIB_DIR}/DUOLib.lib)
				SET(CMAKE_MRPT_HAS_DUO3D 0)
				MESSAGE("The directory 'DUO3D_LIB_DIR' does not contain DUOLib.lib. Turn off DUO3D support or provide the correct path.")
			ENDIF(NOT EXISTS ${DUO3D_LIB_DIR}/DUOLib.lib)
	ENDIF(UNIX)
			
ENDIF(MRPT_HAS_DUO3D)
