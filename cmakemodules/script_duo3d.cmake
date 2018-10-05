# Support for DUO3D stereo camera
# ==========================================================================================
set(CMAKE_MRPT_HAS_DUO3D 0)  # Set default value (it cannot be empty!)

set( MRPT_HAS_DUO3D OFF CACHE BOOL "Build with support for DUO3D libraries?")

if( MRPT_HAS_DUO3D )
	if(UNIX)
		message("Sorry! DUO3D camera is supported only for Windows yet. Set MRPT_HAS_DUO3D to OFF")
	else(UNIX)
	
			set( DUO3D_INCLUDE_DIR "" CACHE PATH "Path to DUO3D include directory" )
			set( DUO3D_LIB_DIR "" CACHE PATH "Path to DUO3D library directory" )

			# Set to 1, next check for missing things and set to 0 on any error & report message:
			set(CMAKE_MRPT_HAS_DUO3D 1)

			# if(NOT EXISTS ${DUO3D_ROOT_DIR})
				# set(CMAKE_MRPT_HAS_DUO3D 0)
				# message("The directory 'DUO3D_ROOT_DIR' does not exists. Turn off DUO3D support or provide the correct path.")
			# endif(NOT EXISTS ${DUO3D_ROOT_DIR})

			if(NOT EXISTS ${DUO3D_INCLUDE_DIR}/DUOLib.h)
				set(CMAKE_MRPT_HAS_DUO3D 0)
				message("The directory 'DUO3D_INCLUDE_DIR' does not contain DUOLib.h. Turn off DUO3D support or provide the correct path.")
			endif(NOT EXISTS ${DUO3D_INCLUDE_DIR}/DUOLib.h)

			if(NOT EXISTS ${DUO3D_LIB_DIR}/DUOLib.lib)
				set(CMAKE_MRPT_HAS_DUO3D 0)
				message("The directory 'DUO3D_LIB_DIR' does not contain DUOLib.lib. Turn off DUO3D support or provide the correct path.")
			endif(NOT EXISTS ${DUO3D_LIB_DIR}/DUOLib.lib)
	endif(UNIX)
			
endif(MRPT_HAS_DUO3D)
