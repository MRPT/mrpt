# Check for liboctomap
set(CMAKE_MRPT_HAS_OCTOMAP 0)
set(CMAKE_MRPT_HAS_OCTOMAP_SYSTEM 0)

set(EMBEDDED_OCTOMAP_DIR "${MRPT_BINARY_DIR}/otherlibs/octomap")

# Try to locate the pkg via CMake:
find_package(OCTOMAP QUIET)
if (OCTOMAP_FOUND)
	if ($ENV{VERBOSE})
		message(STATUS "liboctomap: Found via pkg-config")
		message(STATUS " OCTOMAP_LIBRARIES=${OCTOMAP_LIBRARIES}")
		message(STATUS " OCTOMAP_INCLUDE_DIRS=${OCTOMAP_INCLUDE_DIRS}")
	endif()

	set(CMAKE_MRPT_HAS_OCTOMAP 1)
	set(CMAKE_MRPT_HAS_OCTOMAP_SYSTEM 1)
endif()

if (NOT OCTOMAP_FOUND)
	set(BUILD_OCTOMAP ON CACHE BOOL "Build an embedded version of Octomap")
	if (BUILD_OCTOMAP)
		# Use embedded version:
		# --------------------------
		if (MSVC)
			set(LIB_EXT "lib")
			set(LIB_PREFIX "")
		else()
			set(LIB_EXT "a")
			set(LIB_PREFIX "lib")
		endif()
		set(CMD_CMAKE_POSTFIX "-DCMAKE_DEBUG_POSTFIX=${CMAKE_DEBUG_POSTFIX}")

		# Include embedded version headers:
		include(ExternalProject)

		# download from GH or use embedded ZIPed version (used only for old Ubuntu PPAs):
		if (EXISTS "${MRPT_SOURCE_DIR}/otherlibs/octomap.zip")
			set(OCTOMAP_EP_URL "${MRPT_SOURCE_DIR}/otherlibs/octomap.zip")
		else()
			set(OCTOMAP_EP_URL "https://github.com/MRPT/octomap/archive/devel.zip")
		endif()

		ExternalProject_Add(EP_octomap
		  URL               "${OCTOMAP_EP_URL}" #TO-DO: Switch back to original repo after next stable release.
		  SOURCE_DIR        "${MRPT_BINARY_DIR}/otherlibs/octomap/"
		  CMAKE_ARGS
			-DBUILD_TESTING=OFF
			-DBUILD_DYNAMICETD3D_SUBPROJECT=OFF
			-DBUILD_OCTOVIS_SUBPROJECT=OFF
			${CMD_CMAKE_POSTFIX}
		  BUILD_COMMAND
			${CMAKE_COMMAND} --build ${MRPT_BINARY_DIR}/EP_octomap-prefix/src/EP_octomap-build --config $<CONFIG> --target octomap-static
			COMMAND ${CMAKE_COMMAND} -E copy ${MRPT_BINARY_DIR}/otherlibs/octomap/lib/${LIB_PREFIX}octomap$<$<CONFIG:Debug>:${CMAKE_DEBUG_POSTFIX}>.${LIB_EXT} ${MRPT_BINARY_DIR}/lib/
			COMMAND ${CMAKE_COMMAND} -E copy ${MRPT_BINARY_DIR}/otherlibs/octomap/lib/${LIB_PREFIX}octomath$<$<CONFIG:Debug>:${CMAKE_DEBUG_POSTFIX}>.${LIB_EXT} ${MRPT_BINARY_DIR}/lib/
		  INSTALL_COMMAND   ""
		  TEST_COMMAND      ""
		)

		set(CMAKE_MRPT_HAS_OCTOMAP 1)
		set(CMAKE_MRPT_HAS_OCTOMAP_SYSTEM 0)

		set(OCTOMAP_LIBRARIES "")

		list(APPEND OCTOMAP_LIBRARIES
			${MRPT_BINARY_DIR}/lib/${LIB_PREFIX}octomath$<$<CONFIG:Debug>:${CMAKE_DEBUG_POSTFIX}>.${LIB_EXT}
			${MRPT_BINARY_DIR}/lib/${LIB_PREFIX}octomap$<$<CONFIG:Debug>:${CMAKE_DEBUG_POSTFIX}>.${LIB_EXT}
			)
		set(OCTOMAP_INCLUDE_DIRS
			"${MRPT_BINARY_DIR}/otherlibs/octomap/octomap/include/"
		)
	endif()
endif()


if (CMAKE_MRPT_HAS_OCTOMAP)
	mark_as_advanced(OCTOMAP_DIR)
	include_directories("${OCTOMAP_INCLUDE_DIRS}")

	if ($ENV{VERBOSE})
		message(STATUS "octomap:")
		message(STATUS " OCTOMAP_INCLUDE_DIRS: ${OCTOMAP_INCLUDE_DIRS}")
		message(STATUS " OCTOMAP_CXX_FLAGS: ${OCTOMAP_CXX_FLAGS}")
		message(STATUS " OCTOMAP_LINK_FLAGS: ${OCTOMAP_LINK_FLAGS}")
		message(STATUS " OCTOMAP_LIBRARIES: ${OCTOMAP_LIBRARIES}")
		message(STATUS " OCTOMAP_LIBRARY_DIRS: ${OCTOMAP_LIBRARY_DIRS}")
		message(STATUS " OCTOMAP_VERSION: ${OCTOMAP_VERSION}")
	endif ($ENV{VERBOSE})
endif ()
