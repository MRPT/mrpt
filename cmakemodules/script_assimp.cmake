# Check for system libassimp:
#  http://assimp.sourceforge.net/
# ===================================================
SET(CMAKE_MRPT_HAS_ASSIMP 0)
SET(CMAKE_MRPT_HAS_ASSIMP_SYSTEM 0)

SET(ASSIMP_FOUND_VIA_CMAKE 0)

SET(EMBEDDED_ASSIMP_DIR "${MRPT_BINARY_DIR}/otherlibs/assimp")

# 1st) Try to locate the pkg via pkg-config:
find_package(PkgConfig QUIET)
IF(PKG_CONFIG_FOUND)
	PKG_CHECK_MODULES(ASSIMP ${_QUIET} assimp)
	IF (ASSIMP_FOUND)
		IF ($ENV{VERBOSE})
			MESSAGE(STATUS "Assimp: Found via pkg-config")
			MESSAGE(STATUS " ASSIMP_LIBRARIES=${ASSIMP_LIBRARIES}")
			MESSAGE(STATUS " ASSIMP_INCLUDE_DIRS=${ASSIMP_INCLUDE_DIRS}")
		ENDIF ($ENV{VERBOSE})

		SET(CMAKE_MRPT_HAS_ASSIMP 1)
		SET(CMAKE_MRPT_HAS_ASSIMP_SYSTEM 1)

		SET(ASSIMP_CXX_FLAGS ${ASSIMP_CFLAGS})
	ENDIF (ASSIMP_FOUND)
ENDIF(PKG_CONFIG_FOUND)

IF (NOT ASSIMP_FOUND)
	SET(BUILD_ASSIMP ON CACHE BOOL "Build an embedded version of Assimp (3D models importer)")
	IF (BUILD_ASSIMP)

		# Use embedded version:
		# --------------------------
		# Include embedded version headers:
		include(ExternalProject)
		# download from GH
		ExternalProject_Add(EP_assimp
		  URL               "https://github.com/assimp/assimp/archive/v4.0.1.tar.gz"
		  URL_MD5           "23a6301c728a413aafbfa1cca19ba91f"
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
			-DCMAKE_DEBUG_POSTFIX=d
		  INSTALL_COMMAND   ""
		  TEST_COMMAND      ""
		)

		# 2nd attempt: via cmake
		SET(ASSIMP_DIR "${EMBEDDED_ASSIMP_DIR}" CACHE PATH "Path to ASSIMP CMake config file" FORCE)
		FIND_PACKAGE(ASSIMP QUIET)

		SET(ASSIMP_FOUND_VIA_CMAKE 1)

		SET(CMAKE_MRPT_HAS_ASSIMP 1)
		SET(CMAKE_MRPT_HAS_ASSIMP_SYSTEM 0)

	ENDIF (BUILD_ASSIMP)
ENDIF()

IF (ASSIMP_FOUND_VIA_CMAKE)
	# override wrong target libs in -config.cmake file:
	set(ASSIMP_LIBRARIES "")

	if(MSVC12)
		SET(ASSIMP_MSVC_VERSION "vc120")
	elseif(MSVC14)
		SET(ASSIMP_MSVC_VERSION "vc140")
	ENDIF(MSVC12)

	if(MSVC12 OR MSVC14)
		SET(ASSIMP_CUSTOM_LIB_NAME "assimp-mrpt-${ASSIMP_MSVC_VERSION}-mt")
	else()
		SET(ASSIMP_CUSTOM_LIB_NAME "assimp")
		SET(ASSIMP_LIBRARY_DIRS "${MRPT_BINARY_DIR}/lib")
	endif()

	LIST(APPEND ASSIMP_LIBRARIES optimized "${ASSIMP_CUSTOM_LIB_NAME}" debug "${ASSIMP_CUSTOM_LIB_NAME}d")

	# override wrong include dirs:
	SET(ASSIMP_INCLUDE_DIRS
		"${MRPT_BINARY_DIR}/otherlibs/assimp/include/"
		"${MRPT_BINARY_DIR}/EP_assimp-prefix/src/EP_assimp-build/include/"
	)

	# Install assimp DLLs (for binary packages)
	IF(WIN32)
		FILE(GLOB_RECURSE EXTRA_DLLS "${MRPT_BINARY_DIR}/bin/Release/assimp-*.dll" "${MRPT_BINARY_DIR}/bin/Debug/assimp-*.dll")
		FOREACH(F ${EXTRA_DLLS})
			INSTALL(FILES "${F}" DESTINATION bin)
		ENDFOREACH(F)
	ENDIF()
ENDIF (ASSIMP_FOUND_VIA_CMAKE)

# ASSIMP_ROOT_DIR - the root directory where the installation can be found
# ASSIMP_CXX_FLAGS - extra flags for compilation
# ASSIMP_LINK_FLAGS - extra flags for linking
# ASSIMP_INCLUDE_DIRS - include directories
# ASSIMP_LIBRARY_DIRS - link directories
# ASSIMP_LIBRARIES - libraries to link plugins with
IF (CMAKE_MRPT_HAS_ASSIMP)
	if (NOT "${ASSIMP_LIBRARY_DIRS}" STREQUAL "")
		LINK_DIRECTORIES("${ASSIMP_LIBRARY_DIRS}")
	endif()

	MARK_AS_ADVANCED(ASSIMP_DIR)

	IF ($ENV{VERBOSE})
		MESSAGE(STATUS "Assimp:")
		MESSAGE(STATUS " ASSIMP_INCLUDE_DIRS: ${ASSIMP_INCLUDE_DIRS}")
		MESSAGE(STATUS " ASSIMP_CXX_FLAGS: ${ASSIMP_CXX_FLAGS}")
		MESSAGE(STATUS " ASSIMP_LINK_FLAGS: ${ASSIMP_LINK_FLAGS}")
		MESSAGE(STATUS " ASSIMP_LIBRARIES: ${ASSIMP_LIBRARIES}")
		MESSAGE(STATUS " ASSIMP_LIBRARY_DIRS: ${ASSIMP_LIBRARY_DIRS}")
		MESSAGE(STATUS " ASSIMP_VERSION: ${ASSIMP_VERSION}")
	ENDIF ($ENV{VERBOSE})
ENDIF (CMAKE_MRPT_HAS_ASSIMP)
