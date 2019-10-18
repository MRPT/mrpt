# Check for the OpenCV libraries:
#  pkg-config if available (Linux), otherwise CMake module
# =========================================================
set(CMAKE_MRPT_HAS_OPENCV 0)

set(MRPT_OPENCV_VERSION 0.0.0)
set(MRPT_OPENCV_VERSION_HEX "0x000")
set(MRPT_OPENCV_SRC_DIR "") # used by MRPT exported targets

set(OpenCV_IGNORE_PKGCONFIG OFF CACHE BOOL "Forces using OpenCVConfig.cmake to find OpenCV")
mark_as_advanced(OpenCV_IGNORE_PKGCONFIG)

option(DISABLE_OPENCV "Disable the OpenCV library" "OFF")
mark_as_advanced(DISABLE_OPENCV)
if(DISABLE_OPENCV)
	return()
endif()


# 1st option: Try to find OpenCV config file (NO_MODULE: Don't find a module, but OpenCVConfig.cmake):
if(NOT CMAKE_MRPT_HAS_OPENCV)
	find_package(OpenCV QUIET NO_MODULE)
	if(OpenCV_FOUND)
		set(MRPT_OPENCV_VERSION ${OpenCV_VERSION})
		set(OpenCV_LIBRARIES ${OpenCV_LIBS})
		set(OPENCV_LIBDIR ${OpenCV_LIB_DIR})
		if (NOT "${BASEDIR}" STREQUAL "")
			set(MRPT_OPENCV_SRC_DIR "${BASEDIR}")
		endif (NOT "${BASEDIR}" STREQUAL "")
		if($ENV{VERBOSE})
			message(STATUS "OpenCV ${OpenCV_VERSION} found through OpenCVConfig.cmake")
		endif($ENV{VERBOSE})

		set(CMAKE_MRPT_HAS_OPENCV 1)
	endif(OpenCV_FOUND)
endif(NOT CMAKE_MRPT_HAS_OPENCV)

# Opencv version as Hex. number:
VERSION_TO_HEXADECIMAL(MRPT_OPENCV_VERSION_HEX ${MRPT_OPENCV_VERSION})

# OpenCV (all compilers):
if(CMAKE_MRPT_HAS_OPENCV)
	# Important: we can't link against opencv_ts, apparently it leads to crashes
	# when also linking to gtest (???)
	list(REMOVE_ITEM OpenCV_LIBRARIES opencv_ts)

	if($ENV{VERBOSE})
		message(STATUS "OpenCV:")
		message(STATUS "        OpenCV_LIBRARIES:   ${OpenCV_LIBRARIES}")
		message(STATUS "        OpenCV_INCLUDE_DIRS: ${OpenCV_INCLUDE_DIRS}")
	endif($ENV{VERBOSE})

	add_library(imp_opencv INTERFACE IMPORTED)
	set_target_properties(imp_opencv
		PROPERTIES
		INTERFACE_INCLUDE_DIRECTORIES "${OpenCV_INCLUDE_DIRS}"
		INTERFACE_LINK_LIBRARIES "${OpenCV_LIBRARIES}"
		)

	# TODO: When all opencv versions in non-EOL Ubuntu distros use exported targets
	# simplify all this:
	set(OpenCV_LIBRARIES imp_opencv)

	set(CMAKE_MRPT_HAS_OPENCV_SYSTEM 1)
endif()

# -- install DLLs for MRPT binary packages --
if(WIN32)
	if (EXISTS "${OpenCV_DIR}/bin/Release")
		file(GLOB_RECURSE EXTRA_DLLS "${OpenCV_DIR}/bin/*.dll") # This includes debug & release DLLs
		foreach(F ${EXTRA_DLLS})
			install(FILES "${F}" DESTINATION bin)
		endforeach()
	endif()
endif()
