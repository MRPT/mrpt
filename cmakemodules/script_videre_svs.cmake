# Support for Videre Design stereo camera:
# ===================================================
set( MRPT_HAS_SVS OFF CACHE BOOL "Add support for STOC Stereo Camera?")

if(MRPT_HAS_SVS)
	set( SVS_ROOT_DIR "" CACHE PATH "Path to SVS library root directory")


	if(UNIX)
		# Set to 1, next check for missing things and set to 0 on any error & report message:
		set(CMAKE_MRPT_HAS_SVS 1)

		if(NOT EXISTS ${SVS_ROOT_DIR})
			set(CMAKE_MRPT_HAS_SVS 0)
			message("The directory 'SVS_ROOT_DIR' does not exists. Turn off SVS support or provide the correct path.")
		endif(NOT EXISTS ${SVS_ROOT_DIR})

		if(NOT EXISTS ${SVS_ROOT_DIR}/src/svsclass.h)
			set(CMAKE_MRPT_HAS_SVS 0)
			message("The directory 'SVS_ROOT_DIR' does not contain src/svsclass.h. Turn off SVS support or provide the correct path.")
		endif(NOT EXISTS ${SVS_ROOT_DIR}/src/svsclass.h)

		if(NOT EXISTS ${SVS_ROOT_DIR}/src/dcs.h)
			set(CMAKE_MRPT_HAS_SVS 0)
			message("The directory 'SVS_ROOT_DIR' does not contain src/dcs.h. Turn off SVS support or provide the correct path.")
		endif(NOT EXISTS ${SVS_ROOT_DIR}/src/dcs.h)
	else(UNIX)


		message("Sorry! STOC camera is supported only for LINUX yet. Set MRPT_HAS_SVS to OFF")
		set(CMAKE_MRPT_HAS_SVS 0)
	endif(UNIX)
endif(MRPT_HAS_SVS)

if(CMAKE_MRPT_HAS_SVS)
	include_directories("${SVS_ROOT_DIR}/src")
	link_directories("${SVS_ROOT_DIR}/bin")
endif(CMAKE_MRPT_HAS_SVS)

# This can only be a system lib:
set(CMAKE_MRPT_HAS_SVS_SYSTEM 0)
if(CMAKE_MRPT_HAS_SVS)
	set(CMAKE_MRPT_HAS_SVS_SYSTEM 1)
endif(CMAKE_MRPT_HAS_SVS)

