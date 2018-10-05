# Support for NITE2 Library (only for windows -- by now --)
# ==========================================================================================
set(CMAKE_MRPT_HAS_NITE2 0)
if(UNIX)
	set( MRPT_HAS_NITE2 OFF CACHE INTERNAL "Has NITE2 Windows libraries?" FORCE)
else(UNIX)
	set( MRPT_HAS_NITE2 OFF CACHE BOOL "Has NITE2 Windows libraries?")

	if(MRPT_HAS_NITE2)
		set( NITE2_ROOT_DIR "" CACHE PATH "Path to NITE2 library root directory")

		if(UNIX)
			message("Sorry! NITE2 is supported only for Windows yet. Set MRPT_HAS_NITE2 to OFF")
			set(CMAKE_MRPT_HAS_NITE2 0)
		else(UNIX)
			# Set to 1, next check for missing things and set to 0 on any error & report message:
			set(CMAKE_MRPT_HAS_NITE2 1)

			if(NOT EXISTS ${NITE2_ROOT_DIR})
				set(CMAKE_MRPT_HAS_NITE2 0)
				message("The directory 'NITE2_ROOT_DIR' does not exists. Turn off NITE2 support or provide the correct path.")
			endif(NOT EXISTS ${NITE2_ROOT_DIR})

			if(NOT EXISTS ${NITE2_ROOT_DIR}/Include/NiTE.h)
				set(CMAKE_MRPT_HAS_NITE2 0)
				message("The directory 'NITE2_ROOT_DIR' does not contain Include/NiTE.h. Turn off NITE2 support or provide the correct path.")
			endif(NOT EXISTS ${NITE2_ROOT_DIR}/Include/NiTE.h)

			if(NOT EXISTS ${NITE2_ROOT_DIR}/Lib/NiTE2.lib)
				set(CMAKE_MRPT_HAS_NITE2 0)
				message("The directory 'NITE2_ROOT_DIR' does not contain Lib/NiTE2.lib. Turn off NITE2 support or provide the correct path.")
			endif(NOT EXISTS ${NITE2_ROOT_DIR}/Lib/NiTE2.lib)

		endif(UNIX)

	endif(MRPT_HAS_NITE2)
endif(UNIX)

if(CMAKE_MRPT_HAS_NITE2)
	include_directories("${NITE2_ROOT_DIR}/Include")
	link_directories("${NITE2_ROOT_DIR}/Lib")
endif(CMAKE_MRPT_HAS_NITE2)
