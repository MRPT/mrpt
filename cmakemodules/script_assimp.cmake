# Check for system libassimp:
#  http://assimp.sourceforge.net/
# ===================================================


SET(CMAKE_MRPT_HAS_ASSIMP 0)
SET(CMAKE_MRPT_HAS_ASSIMP_SYSTEM 0)

SET(EMBEDDED_ASSIMP_DIR "${MRPT_BINARY_DIR}/otherlibs/assimp")

# 1st) Try to locate it via CMake (installed in the system or precompiled somewhere)
FIND_PACKAGE(ASSIMP QUIET) # Quiet in a 1st try...

IF (NOT ASSIMP_FOUND OR "${ASSIMP_DIR}" STREQUAL "${EMBEDDED_ASSIMP_DIR}")

	SET(BUILD_ASSIMP ON CACHE BOOL "Build an embedded version of Assimp (3D models importer)")
	IF (BUILD_ASSIMP)

		# Use embedded version:
		# --------------------------
		# Tune cmake vars for assimp build for mrpt:
		SET (ASSIMP_BUILD_ASSIMP_TOOLS OFF CACHE BOOL "If the supplementary tools for Assimp are built in addition to the library." FORCE)	
		SET (ASSIMP_BUILD_SAMPLES OFF CACHE BOOL "If the official samples are built as well (needs Glut)." FORCE)
		SET (ASSIMP_BUILD_STATIC_LIB ON CACHE BOOL "Build Assimp static." FORCE)
		SET (ASSIMP_BUILD_TESTS OFF CACHE BOOL "." FORCE)
		set(ASSIMP_LIBRARY_SUFFIX "-mrpt" CACHE STRING "Suffix to append to library names" FORCE)
		
		add_subdirectory("${MRPT_SOURCE_DIR}/otherlibs/assimp/")
		if(ENABLE_SOLUTION_FOLDERS)
			set_target_properties(assimp PROPERTIES FOLDER "3rd party")
		else(ENABLE_SOLUTION_FOLDERS)
			SET_TARGET_PROPERTIES(assimp  PROPERTIES PROJECT_LABEL "(3rdparty) assimp")
		endif(ENABLE_SOLUTION_FOLDERS)
		MARK_AS_ADVANCED(
			ASSIMP_BUILD_ASSIMP_TOOLS
			ASSIMP_BUILD_SAMPLES
			ASSIMP_BUILD_TESTS
			AMD64
			ASM686
			ASSIMP_BIN_INSTALL_DIR
			ASSIMP_BUILD_STATIC_LIB 
			ASSIMP_DEBUG_POSTFIX
			ASSIMP_ENABLE_BOOST_WORKAROUND
			ASSIMP_NO_EXPORT
			ASSIMP_INCLUDE_INSTALL_DIR
			ASSIMP_LIB_INSTALL_DIR
			ASSIMP_BIN_INSTALL_DIR
			ASSIMP_INSTALL_PDB
			ASSIMP_OPT_BUILD_PACKAGES
			ASSIMP_PACKAGE_VERSION
			ASSIMP_LIBRARY_SUFFIX
			)
			
		# 2nd attempt: Fatal error if not found:
		SET(ASSIMP_DIR "${EMBEDDED_ASSIMP_DIR}" CACHE PATH "Path to ASSIMP CMake config file" FORCE)
		FIND_PACKAGE(ASSIMP REQUIRED)

		# override wrong target libs in -config.cmake file:
		set(ASSIMP_LIBRARIES "")
		LIST(APPEND ASSIMP_LIBRARIES optimized "assimp-mrpt" debug "assimp-mrptd")
		
		# Override binary output dir:
		SET_TARGET_PROPERTIES(assimp PROPERTIES 
			ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/lib/"
			RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin/"
			)
			
		
		
		SET(CMAKE_MRPT_HAS_ASSIMP 1)
		SET(CMAKE_MRPT_HAS_ASSIMP_SYSTEM 0)
	ENDIF (BUILD_ASSIMP)
	
ELSE(NOT ASSIMP_FOUND OR "${ASSIMP_DIR}" STREQUAL "${EMBEDDED_ASSIMP_DIR}")
	# Detected in the system:
	SET(CMAKE_MRPT_HAS_ASSIMP 1)
	SET(CMAKE_MRPT_HAS_ASSIMP_SYSTEM 0)
	
	MARK_AS_ADVANCED(ASSIMP_DIR)
	
ENDIF(NOT ASSIMP_FOUND OR "${ASSIMP_DIR}" STREQUAL "${EMBEDDED_ASSIMP_DIR}")


# ASSIMP_ROOT_DIR - the root directory where the installation can be found
# ASSIMP_CXX_FLAGS - extra flags for compilation
# ASSIMP_LINK_FLAGS - extra flags for linking
# ASSIMP_INCLUDE_DIRS - include directories
# ASSIMP_LIBRARY_DIRS - link directories
# ASSIMP_LIBRARIES - libraries to link plugins with
IF (ASSIMP_FOUND)
	IF ($ENV{VERBOSE})	
		MESSAGE(STATUS "Assimp:")
		MESSAGE(STATUS " ASSIMP_INCLUDE_DIRS: ${ASSIMP_INCLUDE_DIRS}")
		MESSAGE(STATUS " ASSIMP_CXX_FLAGS: ${ASSIMP_CXX_FLAGS}")
		MESSAGE(STATUS " ASSIMP_LINK_FLAGS: ${ASSIMP_LINK_FLAGS}")
		MESSAGE(STATUS " ASSIMP_LIBRARIES: ${ASSIMP_LIBRARIES}")
	ENDIF ($ENV{VERBOSE})	
ENDIF (ASSIMP_FOUND)
	
