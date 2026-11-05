# Check for system libassimp:
#  http://assimp.sourceforge.net/
# ===================================================
set(CMAKE_MRPT_HAS_ASSIMP 0)
set(CMAKE_MRPT_HAS_ASSIMP_SYSTEM 0)

option(DISABLE_ASSIMP "Force not using ASSIMP library" "OFF")
mark_as_advanced(DISABLE_ASSIMP)
if(DISABLE_ASSIMP)
	return()
endif()

# 1st) Try to locate the pkg via pkg-config:
find_package(PkgConfig QUIET)
if(PKG_CONFIG_FOUND)
	PKG_CHECK_MODULES(ASSIMP QUIET assimp)
	if(ASSIMP_FOUND)
		if($ENV{VERBOSE})
			message(STATUS "Assimp: Found via pkg-config")
			message(STATUS " ASSIMP_LIBRARIES=${ASSIMP_LIBRARIES}")
			message(STATUS " ASSIMP_INCLUDE_DIRS=${ASSIMP_INCLUDE_DIRS}")
		endif()

		set(CMAKE_MRPT_HAS_ASSIMP 1)
		set(CMAKE_MRPT_HAS_ASSIMP_SYSTEM 1)

		set(ASSIMP_CXX_FLAGS ${ASSIMP_CFLAGS})
	endif()
endif()

# 2nd) An installed CMake package (e.g. from vcpkg):
if (NOT ASSIMP_FOUND)
	find_package(assimp CONFIG QUIET)
	if (assimp_FOUND AND TARGET assimp::assimp)
		set(ASSIMP_FOUND 1)
		set(ASSIMP_LIBRARIES assimp::assimp)
		set(ASSIMP_INCLUDE_DIRS "")
		set(ASSIMP_CXX_FLAGS "")
		set(ASSIMP_VERSION "${assimp_VERSION}")

		set(CMAKE_MRPT_HAS_ASSIMP 1)
		set(CMAKE_MRPT_HAS_ASSIMP_SYSTEM 1)
	endif()
endif()

# 3rd) Embedded version: a static library, linked into mrpt_viz. It uses the
# system zlib. (Not for MSVC: install the Assimp CMake package instead.)
if (NOT ASSIMP_FOUND AND NOT MSVC)
	set(MRPT_BUILD_ASSIMP ON CACHE BOOL "Build an embedded version of Assimp (3D models importer)")
	find_package(ZLIB QUIET)
	if (MRPT_BUILD_ASSIMP AND NOT ZLIB_FOUND)
		message(STATUS "Assimp: not found, and zlib is missing to build the embedded version.")
	endif()
	if (MRPT_BUILD_ASSIMP AND ZLIB_FOUND)
		include(ExternalProject)
		set(ASSIMP_VERSION "5.3.1")
		set(_assimp_prefix "${mrpt_viz_BINARY_DIR}/3rdparty/assimp-install")
		set(_assimp_lib "${_assimp_prefix}/lib/${CMAKE_STATIC_LIBRARY_PREFIX}assimp$<$<CONFIG:Debug>:d>${CMAKE_STATIC_LIBRARY_SUFFIX}")
		ExternalProject_Add(EP_assimp
		  URL               "https://github.com/assimp/assimp/archive/v${ASSIMP_VERSION}.tar.gz"
		  URL_MD5           "835357be3c1e70120fe2d5a52542ce55"
		  SOURCE_DIR        "${mrpt_viz_BINARY_DIR}/3rdparty/assimp"
		  CMAKE_ARGS
			-DBUILD_SHARED_LIBS=OFF
			-DCMAKE_POSITION_INDEPENDENT_CODE=ON
			-DASSIMP_BUILD_ASSIMP_TOOLS=OFF
			-DASSIMP_BUILD_SAMPLES=OFF
			-DASSIMP_BUILD_TESTS=OFF
			-DASSIMP_BUILD_ZLIB=OFF
			-DASSIMP_BUILD_MINIZIP=ON
			-DASSIMP_NO_EXPORT=ON
			-DASSIMP_WARNINGS_AS_ERRORS=OFF
			-DASSIMP_INSTALL_PDB=OFF
			-DCMAKE_INSTALL_PREFIX=${_assimp_prefix}
			-DCMAKE_INSTALL_LIBDIR=lib
			-DCMAKE_BUILD_TYPE=${CMAKE_BUILD_TYPE}
			-DCMAKE_C_COMPILER=${CMAKE_C_COMPILER}
			-DCMAKE_CXX_COMPILER=${CMAKE_CXX_COMPILER}
			-DCMAKE_C_COMPILER_LAUNCHER=${CMAKE_C_COMPILER_LAUNCHER}
			-DCMAKE_CXX_COMPILER_LAUNCHER=${CMAKE_CXX_COMPILER_LAUNCHER}
			-DCMAKE_OSX_DEPLOYMENT_TARGET=${CMAKE_OSX_DEPLOYMENT_TARGET}
			-DCMAKE_TOOLCHAIN_FILE=${CMAKE_TOOLCHAIN_FILE}
		  BUILD_BYPRODUCTS  "${_assimp_lib}"
		  TEST_COMMAND      ""
		)

		set(ASSIMP_LIBRARIES "${_assimp_lib}" ZLIB::ZLIB)
		set(ASSIMP_INCLUDE_DIRS "${_assimp_prefix}/include")
		set(ASSIMP_CXX_FLAGS "")
		unset(_assimp_prefix)
		unset(_assimp_lib)

		set(CMAKE_MRPT_HAS_ASSIMP 1)
		set(CMAKE_MRPT_HAS_ASSIMP_SYSTEM 0)
	endif()
endif()

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
	endif ()
endif ()
