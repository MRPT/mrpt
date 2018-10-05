# SuiteSparse: (for Cholmod, CSparse, etc.)
# ------------------------------------------

set(CMAKE_MRPT_HAS_SUITESPARSE 0)
set(CMAKE_MRPT_HAS_SUITESPARSE_SYSTEM 0)


set(SuiteSparse_USE_LAPACK_BLAS ON)
find_package(SuiteSparse QUIET NO_MODULE)  # 1st: Try to locate the *config.cmake file.
if(NOT SuiteSparse_FOUND)
	# Only use the FindSuiteSparse.cmake module in Unix or if the user explicitly enables it in Windows:
	if (WIN32)
		set(DEFAULT_SUITESPARSE_USE_FIND_MODULE "OFF")
	else (WIN32)
		set(DEFAULT_SUITESPARSE_USE_FIND_MODULE "ON")
	endif (WIN32)	
	set(SUITESPARSE_USE_FIND_MODULE ${DEFAULT_SUITESPARSE_USE_FIND_MODULE} CACHE BOOL "Use CMake module to locate SuiteSparse?")
	
	if(SUITESPARSE_USE_FIND_MODULE)
        set(SuiteSparse_VERBOSE OFF)
        find_package(SuiteSparse QUIET) # 2nd: Use FindSuiteSparse.cmake module
        include_directories(${SuiteSparse_INCLUDE_DIRS})
	endif(SUITESPARSE_USE_FIND_MODULE)
else(NOT SuiteSparse_FOUND)
		if($ENV{VERBOSE})
			message(STATUS "Find SuiteSparse : include(${USE_SuiteSparse})")
		endif($ENV{VERBOSE})
        include(${USE_SuiteSparse})
endif(NOT SuiteSparse_FOUND)

if(SuiteSparse_FOUND)
	if($ENV{VERBOSE})
		message(STATUS "SuiteSparse_LIBS: ${SuiteSparse_LIBRARIES}")
	endif($ENV{VERBOSE})

	#APPEND_MRPT_LIBS(${SuiteSparse_LIBRARIES})

	set(CMAKE_MRPT_HAS_SUITESPARSE 1)
	set(CMAKE_MRPT_HAS_SUITESPARSE_SYSTEM 1)
endif(SuiteSparse_FOUND)


