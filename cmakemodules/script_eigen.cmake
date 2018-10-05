# Eigen library plugins:
# ===================================================
set(EIGEN_MATRIXBASE_PLUGIN "<mrpt/math/eigen_plugins.h>" CACHE STRING "Eigen plugin header")
set(EIGEN_MATRIXBASE_PLUGIN_POST_IMPL "<mrpt/math/eigen_plugins_impl.h>" CACHE STRING "Eigen plugin implementation header")

mark_as_advanced(EIGEN_MATRIXBASE_PLUGIN)
mark_as_advanced(EIGEN_MATRIXBASE_PLUGIN_POST_IMPL)

# By default: Use system version
find_package(Eigen3 QUIET NO_MODULE)

set(EIGEN_USE_EMBEDDED_VERSION OFF CACHE BOOL "Download Eigen3 and use it instead of system version")
if (EIGEN_USE_EMBEDDED_VERSION)
	# Include embedded version headers:
	include(ExternalProject)
	# download Eigen from bitbucket
	ExternalProject_Add(EP_eigen3
	  URL               "https://bitbucket.org/eigen/eigen/get/3.3.4.tar.bz2"
	  URL_MD5           "a7aab9f758249b86c93221ad417fbe18"
	  SOURCE_DIR        "${MRPT_BINARY_DIR}/otherlibs/eigen3/"
	  CONFIGURE_COMMAND ""
	  BUILD_COMMAND     ""
	  INSTALL_COMMAND   ""
	  TEST_COMMAND      ""
	)

	set(EIGEN_EMBEDDED_INCLUDE_DIR "${MRPT_BINARY_DIR}/otherlibs/eigen3/" CACHE PATH "Eigen path for embedded use" FORCE)
	mark_as_advanced(EIGEN_EMBEDDED_INCLUDE_DIR)

	set(MRPT_EIGEN_INCLUDE_DIR "${EIGEN_EMBEDDED_INCLUDE_DIR}")
elseif(Eigen3_FOUND)
	# Use system version
	set(MRPT_EIGEN_INCLUDE_DIR "${EIGEN3_INCLUDE_DIR}")
else()
	message(FATAL_ERROR "eigen3 is required to build MRPT! Either install it and set EIGEN3_DIR or enable the variable EIGEN_USE_EMBEDDED_VERSION to automatically download it now.")
endif()

include_directories("${MRPT_EIGEN_INCLUDE_DIR}")
if(EXISTS "${MRPT_EIGEN_INCLUDE_DIR}/unsupported/")
	include_directories("${MRPT_EIGEN_INCLUDE_DIR}/unsupported/")
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

# Add directories as "-isystem" to avoid warnings with :
set(AUX_EIGEN_INCL_DIR ${MRPT_EIGEN_INCLUDE_DIR})
ADD_DIRECTORIES_AS_ISYSTEM(AUX_EIGEN_INCL_DIR)

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
	endif($ENV{VERBOSE})
endif (EXISTS ${EIGEN_VER_H})
