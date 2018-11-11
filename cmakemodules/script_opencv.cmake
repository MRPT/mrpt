# Check for the OpenCV libraries:
#  pkg-config if available (Linux), otherwise CMake module
# =========================================================
set(CMAKE_MRPT_HAS_OPENCV 0)
set(MRPT_OPENCV_VERSION 0.0.0)
set(MRPT_OPENCV_VERSION_HEX "0x000")
set(MRPT_OPENCV_SRC_DIR "")
set(OpenCV_IGNORE_PKGCONFIG OFF CACHE BOOL "Forces using OpenCVConfig.cmake to find OpenCV")
mark_as_advanced(OpenCV_IGNORE_PKGCONFIG)

# Use CMAKE module if opencv's not been detected yet:
if(NOT CMAKE_MRPT_HAS_OPENCV)
	# 1st: Try to find OpenCV config file (NO_MODULE: Don't find a module, but OpenCVConfig.cmake):
	find_package(OpenCV  QUIET NO_MODULE)
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

# 2nd: Invoke pkg-config for getting the configuration:
if(NOT CMAKE_MRPT_HAS_OPENCV AND PKG_CONFIG_FOUND AND NOT OpenCV_IGNORE_PKGCONFIG)
	PKG_CHECK_MODULES(OPENCV QUIET opencv)
	if(OPENCV_FOUND)
		set(CMAKE_MRPT_HAS_OPENCV 1)
		set(MRPT_OPENCV_VERSION ${OPENCV_VERSION})

		if ("${OPENCV_LIBDIR}")
			link_directories(${OPENCV_LIBDIR})
		endif ("${OPENCV_LIBDIR}")
		set(OpenCV_LIBRARIES ${OPENCV_LIBRARIES})

		if($ENV{VERBOSE})
			message(STATUS " opencv include: ${OPENCV_INCLUDE_DIRS} (Version: ${OPENCV_VERSION})")
		endif($ENV{VERBOSE})
	endif(OPENCV_FOUND)
endif(NOT CMAKE_MRPT_HAS_OPENCV AND PKG_CONFIG_FOUND AND NOT OpenCV_IGNORE_PKGCONFIG)


if(NOT CMAKE_MRPT_HAS_OPENCV)
	# 3rd: OK, let's use the module:
	find_package(OpenCV)
	if(OpenCV_FOUND)
		# MRPT_OPENCV_VERSION
		if($ENV{VERBOSE})
			message(STATUS "OPENCV_EXE_LINKER_FLAGS: ${OpenCV_EXE_LINKER_FLAGS}")
			message(STATUS "OPENCV_INCLUDE_DIR: ${OpenCV_INCLUDE_DIR}")
			message(STATUS "OpenCV_LIBRARIES: ${OpenCV_LIBRARIES}")
		endif($ENV{VERBOSE})

		file(GLOB_RECURSE CV_VER_H "${OpenCV_CXCORE_INCLUDE_DIR}/cvver.h")
		file(READ "${CV_VER_H}" STR_CV_VERSION)

		# Extract the CV version from the cvver.h file, lines "#define CV_MAJOR_VERSION  XX", etc...

		#string(REGEX MATCHALL "[0-9]+.[0-9]+.[0-9]+" MRPT_OPENCV_VERSION "${STR_CV_VERSION}")
		string(REGEX MATCH "CV_MAJOR_VERSION[ ]+[0-9]+" CMAKE_OPENCV_VERSION_NUMBER_MAJOR "${STR_CV_VERSION}")
		string(REGEX MATCH "[0-9]+" CMAKE_OPENCV_VERSION_NUMBER_MAJOR "${CMAKE_OPENCV_VERSION_NUMBER_MAJOR}")

		string(REGEX MATCH "CV_MINOR_VERSION[ ]+[0-9]+" CMAKE_OPENCV_VERSION_NUMBER_MINOR "${STR_CV_VERSION}")
		string(REGEX MATCH "[0-9]+" CMAKE_OPENCV_VERSION_NUMBER_MINOR "${CMAKE_OPENCV_VERSION_NUMBER_MINOR}")

		string(REGEX MATCH "CV_SUBMINOR_VERSION[ ]+[0-9]+" CMAKE_OPENCV_VERSION_NUMBER_PATCH "${STR_CV_VERSION}")
		string(REGEX MATCH "[0-9]+" CMAKE_OPENCV_VERSION_NUMBER_PATCH "${CMAKE_OPENCV_VERSION_NUMBER_PATCH}")

		set(MRPT_OPENCV_VERSION "${CMAKE_OPENCV_VERSION_NUMBER_MAJOR}.${CMAKE_OPENCV_VERSION_NUMBER_MINOR}.${CMAKE_OPENCV_VERSION_NUMBER_PATCH}")

		if($ENV{VERBOSE})
			message(STATUS "OpenCV version detected: ${MRPT_OPENCV_VERSION}")
		endif($ENV{VERBOSE})

		set(CMAKE_MRPT_HAS_OPENCV 1)
	endif(OpenCV_FOUND)
endif(NOT CMAKE_MRPT_HAS_OPENCV)


# Opencv version as Hex. number:
VERSION_TO_HEXADECIMAL(MRPT_OPENCV_VERSION_HEX ${MRPT_OPENCV_VERSION})

# DISABLE_OPENCV
# ---------------------
option(DISABLE_OPENCV "Disable the OpenCV library" "OFF")
mark_as_advanced(DISABLE_OPENCV)
if(DISABLE_OPENCV)
	set(CMAKE_MRPT_HAS_OPENCV 0)
endif(DISABLE_OPENCV)


# OpenCV (all compilers):
if(CMAKE_MRPT_HAS_OPENCV)
	# Important: we can't link against opencv_ts, apparently it leads to crashes
	# when also linking to gtest (???)
	list(REMOVE_ITEM OpenCV_LIBRARIES opencv_ts)

	if($ENV{VERBOSE})
		message(STATUS "OpenCV:")
		message(STATUS "        OpenCV_LIBRARIES:   ${OpenCV_LIBRARIES}")
		message(STATUS "        OpenCV_INCLUDE_DIR: ${OpenCV_INCLUDE_DIR} ${OpenCV_INCLUDE_DIRS}")
	endif($ENV{VERBOSE})

	include_directories(${OpenCV_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIR})
	#ADD_DIRECTORIES_AS_ISYSTEM(OpenCV_INCLUDE_DIRS)

	set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OPENCV_EXE_LINKER_FLAGS}")

	set(CMAKE_MRPT_HAS_OPENCV_SYSTEM 1)

endif(CMAKE_MRPT_HAS_OPENCV)

# -- install DLLs --
if(WIN32)
	if (EXISTS "${OpenCV_DIR}/bin/Release")
		file(GLOB_RECURSE EXTRA_DLLS "${OpenCV_DIR}/bin/*.dll") # This includes debug & release DLLs
		foreach(F ${EXTRA_DLLS})
			install(FILES "${F}" DESTINATION bin)
		endforeach()
	endif()
endif()
