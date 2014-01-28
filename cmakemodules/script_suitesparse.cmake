# SuiteSparse: (for Cholmod, CSparse, etc.)
# ------------------------------------------

SET(CMAKE_MRPT_HAS_SUITESPARSE 0)
SET(CMAKE_MRPT_HAS_SUITESPARSE_SYSTEM 0)


set(SuiteSparse_USE_LAPACK_BLAS ON)
find_package(SuiteSparse QUIET NO_MODULE)  # 1st: Try to locate the *config.cmake file.
if(NOT SuiteSparse_FOUND)
	# Only use the FindSuiteSparse.cmake module in Unix or if the user explicitly enables it in Windows:
	IF (WIN32)
		SET(DEFAULT_SUITESPARSE_USE_FIND_MODULE "OFF")
	ELSE (WIN32)
		SET(DEFAULT_SUITESPARSE_USE_FIND_MODULE "ON")
	ENDIF (WIN32)	
	SET(SUITESPARSE_USE_FIND_MODULE ${DEFAULT_SUITESPARSE_USE_FIND_MODULE} CACHE BOOL "Use CMake module to locate SuiteSparse?")
	
	IF(SUITESPARSE_USE_FIND_MODULE)
        set(SuiteSparse_VERBOSE OFF)
        find_package(SuiteSparse REQUIRED) # 2nd: Use FindSuiteSparse.cmake module
        include_directories(${SuiteSparse_INCLUDE_DIRS})
	ENDIF(SUITESPARSE_USE_FIND_MODULE)
else(NOT SuiteSparse_FOUND)
		IF($ENV{VERBOSE})
			message(STATUS "Find SuiteSparse : include(${USE_SuiteSparse})")
		ENDIF($ENV{VERBOSE})
        include(${USE_SuiteSparse})
endif(NOT SuiteSparse_FOUND)

if(SuiteSparse_FOUND)
	IF($ENV{VERBOSE})
		MESSAGE(STATUS "SuiteSparse_LIBS: ${SuiteSparse_LIBRARIES}")
	ENDIF($ENV{VERBOSE})

	APPEND_MRPT_LIBS(${SuiteSparse_LIBRARIES})

	SET(CMAKE_MRPT_HAS_SUITESPARSE 1)
	SET(CMAKE_MRPT_HAS_SUITESPARSE_SYSTEM 1)
endif(SuiteSparse_FOUND)


# SuiteSparse: CSparse
# ----------------------------
IF(CSPARSE_FOUND)
	IF($ENV{VERBOSE})
		MESSAGE(STATUS "CSparse: includedir: ${CSPARSE_INCLUDE_DIR}")
		MESSAGE(STATUS "         libs: ${CSPARSE_LIBRARY}")
	ENDIF($ENV{VERBOSE})

	INCLUDE_DIRECTORIES(${CSPARSE_INCLUDE_DIR})
	APPEND_MRPT_LIBS(${CSPARSE_LIBRARY})

	SET(CMAKE_MRPT_HAS_CSPARSE 1)
	SET(CMAKE_MRPT_HAS_CSPARSE_SYSTEM 1)
ELSE(CSPARSE_FOUND)
	# Default: use embedded version
	SET(CMAKE_MRPT_HAS_CSPARSE 1)
	SET(CMAKE_MRPT_HAS_CSPARSE_SYSTEM 0)
ENDIF(CSPARSE_FOUND)
