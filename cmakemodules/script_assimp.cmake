# Check for system libassimp:
#  http://assimp.sourceforge.net/
# ===================================================
set(CMAKE_MRPT_HAS_ASSIMP 0)
set(CMAKE_MRPT_HAS_ASSIMP_SYSTEM 0)

set(ASSIMP_FOUND_VIA_CMAKE 0)

set(EMBEDDED_ASSIMP_DIR "${MRPT_BINARY_DIR}/otherlibs/assimp")

# 1st) Try to locate the pkg via pkg-config:
find_package(PkgConfig QUIET)
if(PKG_CONFIG_FOUND)
	PKG_CHECK_MODULES(ASSIMP QUIET assimp)
	if (ASSIMP_FOUND)
		if ($ENV{VERBOSE})
			message(STATUS "Assimp: Found via pkg-config")
			message(STATUS " ASSIMP_LIBRARIES=${ASSIMP_LIBRARIES}")
			message(STATUS " ASSIMP_INCLUDE_DIRS=${ASSIMP_INCLUDE_DIRS}")
		endif ($ENV{VERBOSE})

		set(CMAKE_MRPT_HAS_ASSIMP 1)
		set(CMAKE_MRPT_HAS_ASSIMP_SYSTEM 1)

		set(ASSIMP_CXX_FLAGS ${ASSIMP_CFLAGS})
	endif (ASSIMP_FOUND)
endif(PKG_CONFIG_FOUND)

if (NOT ASSIMP_FOUND)
	set(BUILD_ASSIMP ON CACHE BOOL "Build an embedded version of Assimp (3D models importer)")
	if (BUILD_ASSIMP)

		# Use embedded version:
		# --------------------------
		# Include embedded version headers:
		include(ExternalProject)
		# download from GH
		ExternalProject_Add(EP_assimp
		  URL               "https://github.com/assimp/assimp/archive/v4.1.0.tar.gz"
		  URL_MD5           "83b53a10c38d964bd1e69da0606e2727"
		  SOURCE_DIR        "${MRPT_BINARY_DIR}/otherlibs/assimp/"
		  CMAKE_ARGS 
			-DASSIMP_BUILD_ASSIMP_TOOLS=OFF
			-DASSIMP_BUILD_SAMPLES=OFF
			-DASSIMP_BUILD_STATIC_LIB=ON
			-DASSIMP_BUILD_TESTS=OFF
			-DASSIMP_LIBRARY_SUFFIX=-mrpt
			-DCMAKE_LIBRARY_OUTPUT_PATH=${MRPT_BINARY_DIR}/lib
			-DLIBRARY_OUTPUT_PATH=${MRPT_BINARY_DIR}/lib
			-DCMAKE_RUNTIME_OUTPUT_DIRECTORY=${MRPT_BINARY_DIR}/bin
			-DRUNTIME_OUTPUT_DIRECTORY=${MRPT_BINARY_DIR}/bin
			-DCMAKE_DEBUG_POSTFIX=${CMAKE_DEBUG_POSTFIX}
		  INSTALL_COMMAND   ""
		  TEST_COMMAND      ""
		)

		# 2nd attempt: via cmake
		set(ASSIMP_DIR "${EMBEDDED_ASSIMP_DIR}" CACHE PATH "Path to ASSIMP CMake config file" FORCE)
		find_package(ASSIMP QUIET)

		set(ASSIMP_FOUND_VIA_CMAKE 1)

		set(CMAKE_MRPT_HAS_ASSIMP 1)
		set(CMAKE_MRPT_HAS_ASSIMP_SYSTEM 0)
	endif (BUILD_ASSIMP)
endif()

if (ASSIMP_FOUND_VIA_CMAKE)
	# override wrong target libs in -config.cmake file:
	set(ASSIMP_LIBRARIES "")

	if(MSVC12)
		set(ASSIMP_MSVC_VERSION "vc120")
	elseif(MSVC14)
		set(ASSIMP_MSVC_VERSION "vc140")
	endif(MSVC12)

	if(MSVC12 OR MSVC14)
		set(ASSIMP_CUSTOM_LIB_NAME "assimp-mrpt-${ASSIMP_MSVC_VERSION}-mt")
	else()
		set(ASSIMP_CUSTOM_LIB_NAME "assimp")
		set(ASSIMP_LIBRARY_DIRS "${MRPT_BINARY_DIR}/lib")
	endif()

	list(APPEND ASSIMP_LIBRARIES optimized "${ASSIMP_CUSTOM_LIB_NAME}" debug "${ASSIMP_CUSTOM_LIB_NAME}${CMAKE_DEBUG_POSTFIX}")

	# override wrong include dirs:
	set(ASSIMP_INCLUDE_DIRS
		"${MRPT_BINARY_DIR}/otherlibs/assimp/include/"
		"${MRPT_BINARY_DIR}/EP_assimp-prefix/src/EP_assimp-build/include/"
	)

	# Install assimp DLLs (for binary packages)
	if(WIN32)
		file(GLOB_RECURSE EXTRA_DLLS "${MRPT_BINARY_DIR}/bin/Release/assimp-*.dll" "${MRPT_BINARY_DIR}/bin/Debug/assimp-*.dll")
		foreach(F ${EXTRA_DLLS})
			install(FILES "${F}" DESTINATION bin)
		endforeach(F)
	endif()
endif (ASSIMP_FOUND_VIA_CMAKE)

# ASSIMP_ROOT_DIR - the root directory where the installation can be found
# ASSIMP_CXX_FLAGS - extra flags for compilation
# ASSIMP_LINK_FLAGS - extra flags for linking
# ASSIMP_INCLUDE_DIRS - include directories
# ASSIMP_LIBRARY_DIRS - link directories
# ASSIMP_LIBRARIES - libraries to link plugins with
if (CMAKE_MRPT_HAS_ASSIMP)
	if (NOT "${ASSIMP_LIBRARY_DIRS}" STREQUAL "")
		link_directories("${ASSIMP_LIBRARY_DIRS}")
	endif()

	mark_as_advanced(ASSIMP_DIR)

	if ($ENV{VERBOSE})
		message(STATUS "Assimp:")
		message(STATUS " ASSIMP_INCLUDE_DIRS: ${ASSIMP_INCLUDE_DIRS}")
		message(STATUS " ASSIMP_CXX_FLAGS: ${ASSIMP_CXX_FLAGS}")
		message(STATUS " ASSIMP_LINK_FLAGS: ${ASSIMP_LINK_FLAGS}")
		message(STATUS " ASSIMP_LIBRARIES: ${ASSIMP_LIBRARIES}")
		message(STATUS " ASSIMP_LIBRARY_DIRS: ${ASSIMP_LIBRARY_DIRS}")
		message(STATUS " ASSIMP_VERSION: ${ASSIMP_VERSION}")
	endif ($ENV{VERBOSE})
endif (CMAKE_MRPT_HAS_ASSIMP)
