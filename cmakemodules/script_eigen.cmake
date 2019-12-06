# By default: Use system version

set(EIGEN_USE_EMBEDDED_VERSION OFF CACHE BOOL "Download Eigen3 and use it instead of system version")
if (EIGEN_USE_EMBEDDED_VERSION)
	# Include embedded version headers:
	include(ExternalProject)
	# download Eigen from gitlab
	ExternalProject_Add(EP_eigen3
	  URL               "https://gitlab.com/libeigen/eigen/-/archive/3.3.7/eigen-3.3.7.tar.bz2"
	  URL_MD5           "b9e98a200d2455f06db9c661c5610496"
	  SOURCE_DIR        "${MRPT_BINARY_DIR}/3rdparty/eigen3/"
	  CONFIGURE_COMMAND ""
	  BUILD_COMMAND     ""
	  INSTALL_COMMAND   ""
	  TEST_COMMAND      ""
	)

	set(EIGEN_EMBEDDED_INCLUDE_DIR "${MRPT_BINARY_DIR}/3rdparty/eigen3/" CACHE PATH "Eigen path for embedded use" FORCE)
	mark_as_advanced(EIGEN_EMBEDDED_INCLUDE_DIR)

	set(MRPT_EIGEN_INCLUDE_DIR "${EIGEN_EMBEDDED_INCLUDE_DIR}")  # only to find out version
	# define interface lib for Eigen3:
	add_library(Eigen INTERFACE)
	add_dependencies(Eigen EP_eigen3)
	export(
		TARGETS Eigen
		FILE "${MRPT_BINARY_DIR}/EP_eigen3-config.cmake"
	)
	install(TARGETS Eigen EXPORT Eigen-targets)
	install(
		EXPORT Eigen-targets
		DESTINATION ${this_lib_dev_INSTALL_PREFIX}share/mrpt
	)
	target_include_directories(Eigen
		SYSTEM  # omit warnings for these hdrs
		INTERFACE
	  $<BUILD_INTERFACE:${EIGEN_EMBEDDED_INCLUDE_DIR}>
	  $<BUILD_INTERFACE:${EIGEN_EMBEDDED_INCLUDE_DIR}/unsupported>
	)
	add_library(Eigen3::Eigen ALIAS Eigen)

else()
	find_package(Eigen3 QUIET NO_MODULE)

	if(Eigen3_FOUND)
		# Use system version
		set(MRPT_EIGEN_INCLUDE_DIR "${EIGEN3_INCLUDE_DIR}") # only to find out version

		# For Ubuntu 16.04, the Eigen version does not support imported target,
		# so create it ourselves:
		if (NOT TARGET Eigen3::Eigen)
			add_library(Eigen3::Eigen INTERFACE IMPORTED GLOBAL)
			set_target_properties(Eigen3::Eigen PROPERTIES
				INTERFACE_INCLUDE_DIRECTORIES
					$<BUILD_INTERFACE:${EIGEN3_INCLUDE_DIR}>
	 		)
		endif()
	else()
		message(FATAL_ERROR "eigen3 is required to build MRPT! Either install it and set EIGEN3_DIR or enable the variable EIGEN_USE_EMBEDDED_VERSION to automatically download it now.")
	endif()
endif()

# Create variables just for the final summary of the configuration (see bottom of this file):
set(CMAKE_MRPT_HAS_EIGEN 1)        # Always, it's a fundamental dep.!

# Create numeric (0/1) variable EIGEN_USE_EMBEDDED_VERSION_BOOL for the .cmake.in file:
if(EIGEN_USE_EMBEDDED_VERSION)
	set(EIGEN_USE_EMBEDDED_VERSION_BOOL 1)
	set(CMAKE_MRPT_HAS_EIGEN_SYSTEM 0)
else()
	set(EIGEN_USE_EMBEDDED_VERSION_BOOL 0)
	set(CMAKE_MRPT_HAS_EIGEN_SYSTEM 1)
endif()

# Detect Eigen version (just to show it in the CMake config summary)
set(EIGEN_VER_H "${MRPT_EIGEN_INCLUDE_DIR}/Eigen/src/Core/util/Macros.h")
if (EXISTS ${EIGEN_VER_H})
	file(READ "${EIGEN_VER_H}" STR_EIGEN_VERSION)

	# Extract the Eigen version from the Macros.h file, lines "#define EIGEN_WORLD_VERSION  XX", etc...

	string(REGEX MATCH "EIGEN_WORLD_VERSION[ ]+[0-9]+" CMAKE_EIGEN_VERSION_NUMBER_MAJOR "${STR_EIGEN_VERSION}")
	string(REGEX MATCH "[0-9]+" CMAKE_EIGEN_VERSION_NUMBER_MAJOR "${CMAKE_EIGEN_VERSION_NUMBER_MAJOR}")

	string(REGEX MATCH "EIGEN_MAJOR_VERSION[ ]+[0-9]+" CMAKE_EIGEN_VERSION_NUMBER_MINOR "${STR_EIGEN_VERSION}")
	string(REGEX MATCH "[0-9]+" CMAKE_EIGEN_VERSION_NUMBER_MINOR "${CMAKE_EIGEN_VERSION_NUMBER_MINOR}")

	string(REGEX MATCH "EIGEN_MINOR_VERSION[ ]+[0-9]+" CMAKE_EIGEN_VERSION_NUMBER_PATCH "${STR_EIGEN_VERSION}")
	string(REGEX MATCH "[0-9]+" CMAKE_EIGEN_VERSION_NUMBER_PATCH "${CMAKE_EIGEN_VERSION_NUMBER_PATCH}")

	set(MRPT_EIGEN_VERSION "${CMAKE_EIGEN_VERSION_NUMBER_MAJOR}.${CMAKE_EIGEN_VERSION_NUMBER_MINOR}.${CMAKE_EIGEN_VERSION_NUMBER_PATCH}")

	if($ENV{VERBOSE})
		message(STATUS "Eigen version detected: ${MRPT_EIGEN_VERSION}")
	endif()

	if(${MRPT_EIGEN_VERSION} VERSION_LESS "3.3")
		message(ERROR "Eigen ${MRPT_EIGEN_VERSION} detected, required >=3.3. Select EIGEN_USE_EMBEDDED_VERSION=ON.")
	endif()
endif ()

# -- Install --
# Using embedded version of libraries that need public headers?
if(EIGEN_USE_EMBEDDED_VERSION AND NOT IS_DEBIAN_DBG_PKG)
	install(
		DIRECTORY
			"${MRPT_BINARY_DIR}/3rdparty/eigen3/Eigen"
			"${MRPT_BINARY_DIR}/3rdparty/eigen3/unsupported"
		DESTINATION libs/math/include/ )
endif()
