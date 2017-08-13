# Check for liboctomap
SET(CMAKE_MRPT_HAS_OCTOMAP 0)
SET(CMAKE_MRPT_HAS_OCTOMAP_SYSTEM 0)

SET(EMBEDDED_OCTOMAP_DIR "${MRPT_BINARY_DIR}/otherlibs/octomap")

# Try to locate the pkg via CMake:
FIND_PACKAGE(OCTOMAP QUIET)
IF (OCTOMAP_FOUND)
	IF ($ENV{VERBOSE})
		MESSAGE(STATUS "liboctomap: Found via pkg-config")
		MESSAGE(STATUS " OCTOMAP_LIBRARIES=${OCTOMAP_LIBRARIES}")
		MESSAGE(STATUS " OCTOMAP_INCLUDE_DIRS=${OCTOMAP_INCLUDE_DIRS}")
	ENDIF()

	SET(CMAKE_MRPT_HAS_OCTOMAP 1)
	SET(CMAKE_MRPT_HAS_OCTOMAP_SYSTEM 1)
ENDIF()

IF (NOT OCTOMAP_FOUND)
	SET(BUILD_OCTOMAP ON CACHE BOOL "Build an embedded version of Octomap")
	IF (BUILD_OCTOMAP)
		# Use embedded version:
		# --------------------------
		if (MSVC)
			SET(LIB_EXT "lib")
			SET(LIB_PREFIX "")
			SET(CMD_CMAKE_POSTFIX "-DCMAKE_DEBUG_POSTFIX=d")
		else()
			SET(LIB_EXT "a")
			SET(LIB_PREFIX "lib")
			SET(CMD_CMAKE_POSTFIX "")
		endif()

		# Include embedded version headers:
		include(ExternalProject)
		# download from GH
		ExternalProject_Add(EP_octomap
		  URL               "https://github.com/jlblancoc/octomap/archive/devel.zip"
		  #URL_MD5           "ea292f0595d7408e36ab7e545ceda013"
		  SOURCE_DIR        "${MRPT_BINARY_DIR}/otherlibs/octomap/"
		  CMAKE_ARGS
			-DBUILD_TESTING=OFF
			-DBUILD_DYNAMICETD3D_SUBPROJECT=OFF
			-DBUILD_OCTOVIS_SUBPROJECT=OFF
			${CMD_CMAKE_POSTFIX}
		  BUILD_COMMAND
			${CMAKE_COMMAND} --build ${MRPT_BINARY_DIR}/EP_octomap-prefix/src/EP_octomap-build --config $<CONFIG> --target octomap-static
			COMMAND ${CMAKE_COMMAND} -E copy ${MRPT_BINARY_DIR}/otherlibs/octomap/lib/${LIB_PREFIX}octomap$<$<CONFIG:Debug>:d>.${LIB_EXT} ${MRPT_BINARY_DIR}/lib/
			COMMAND ${CMAKE_COMMAND} -E copy ${MRPT_BINARY_DIR}/otherlibs/octomap/lib/${LIB_PREFIX}octomath$<$<CONFIG:Debug>:d>.${LIB_EXT} ${MRPT_BINARY_DIR}/lib/
		  INSTALL_COMMAND   ""
		  TEST_COMMAND      ""
		)

		SET(CMAKE_MRPT_HAS_OCTOMAP 1)
		SET(CMAKE_MRPT_HAS_OCTOMAP_SYSTEM 0)

		set(OCTOMAP_LIBRARIES "")

		LIST(APPEND OCTOMAP_LIBRARIES
			${MRPT_BINARY_DIR}/lib/${LIB_PREFIX}octomath$<$<CONFIG:Debug>:d>.${LIB_EXT}
			${MRPT_BINARY_DIR}/lib/${LIB_PREFIX}octomap$<$<CONFIG:Debug>:d>.${LIB_EXT}
			)
		SET(OCTOMAP_INCLUDE_DIRS
			"${MRPT_BINARY_DIR}/otherlibs/octomap/octomap/include/"
		)
		INCLUDE_DIRECTORIES("${OCTOMAP_INCLUDE_DIRS}")

	ENDIF()
ENDIF()


IF (CMAKE_MRPT_HAS_OCTOMAP)
	MARK_AS_ADVANCED(OCTOMAP_DIR)

	IF ($ENV{VERBOSE})
		MESSAGE(STATUS "octomap:")
		MESSAGE(STATUS " OCTOMAP_INCLUDE_DIRS: ${OCTOMAP_INCLUDE_DIRS}")
		MESSAGE(STATUS " OCTOMAP_CXX_FLAGS: ${OCTOMAP_CXX_FLAGS}")
		MESSAGE(STATUS " OCTOMAP_LINK_FLAGS: ${OCTOMAP_LINK_FLAGS}")
		MESSAGE(STATUS " OCTOMAP_LIBRARIES: ${OCTOMAP_LIBRARIES}")
		MESSAGE(STATUS " OCTOMAP_LIBRARY_DIRS: ${OCTOMAP_LIBRARY_DIRS}")
		MESSAGE(STATUS " OCTOMAP_VERSION: ${OCTOMAP_VERSION}")
	ENDIF ($ENV{VERBOSE})
ENDIF ()
