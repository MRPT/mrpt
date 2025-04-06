# Check for the OpenCV libraries:
#  pkg-config if available (Linux), otherwise CMake module
# =========================================================
set(CMAKE_MRPT_HAS_OPENCV 0)

set(MRPT_OPENCV_VERSION 0.0.0)
set(MRPT_OPENCV_VERSION_HEX "0x000")

# Define the interface library even if we don't have opencv in the system,
# or its use is disabled in mrpt, to simplify specifying the list of dependencies
# in all other libs / apps:
add_library(imp_opencv INTERFACE IMPORTED)

option(DISABLE_OPENCV "Disable the OpenCV library" "OFF")
mark_as_advanced(DISABLE_OPENCV)
if(DISABLE_OPENCV)
	return()
endif()

# 1st option: Try to find OpenCV config file (NO_MODULE: Don't find a module, but OpenCVConfig.cmake):
if(NOT CMAKE_MRPT_HAS_OPENCV)
	find_package(OpenCV QUIET COMPONENTS core imgcodecs calib3d) # imgproc  )
	if(OpenCV_FOUND)
		set(MRPT_OPENCV_VERSION ${OpenCV_VERSION})
		set(CMAKE_MRPT_HAS_OPENCV 1)

	endif()
endif()

# Opencv version as Hex. number:
VERSION_TO_HEXADECIMAL(OPENCV_VERSION_HEX ${MRPT_OPENCV_VERSION})
set(MRPT_OPENCV_VERSION_HEX "${OPENCV_VERSION_HEX}")

# OpenCV (all compilers):
if(CMAKE_MRPT_HAS_OPENCV)
	if($ENV{VERBOSE})
		message(STATUS "OpenCV: ${OpenCV_VERSION}")
		message(STATUS "        OpenCV_LIBS:         ${OpenCV_LIBS}")
		message(STATUS "        OpenCV_INCLUDE_DIRS: ${OpenCV_INCLUDE_DIRS}")
	endif()

	set_target_properties(imp_opencv
		PROPERTIES
		INTERFACE_INCLUDE_DIRECTORIES "${OpenCV_INCLUDE_DIRS}"
		INTERFACE_LINK_LIBRARIES "${OpenCV_LIBS}"
		)

	set(CMAKE_MRPT_HAS_OPENCV_SYSTEM 1)
endif()


# -- install DLLs for MRPT binary packages --
if(DEFINED ENV{OPENCV_DLLS_TO_INSTALL_DIRS})
	message(STATUS "Collecting OpenCV DLLs to install from directories: $ENV{OPENCV_DLLS_TO_INSTALL_DIRS}")

	foreach(DIR $ENV{OPENCV_DLLS_TO_INSTALL_DIRS})
		file(TO_CMAKE_PATH "${DIR}" DIR) # fix backslashes
		message(STATUS " Processing: ${DIR}")
		file(GLOB_RECURSE EXTRA_DLLS "${DIR}/*.dll")
		message(STATUS " Found DLLs: ${EXTRA_DLLS}")
		foreach(F ${EXTRA_DLLS})
			install(FILES "${F}" DESTINATION bin)
		endforeach()
	endforeach()
endif()
