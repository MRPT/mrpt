# SuiteSparse: (for Cholmod, CSparse, etc.)
# ------------------------------------------

set(CMAKE_MRPT_HAS_SUITESPARSE 0)
set(CMAKE_MRPT_HAS_SUITESPARSE_SYSTEM 0)


set(SuiteSparse_USE_LAPACK_BLAS ON)
find_package(SuiteSparse QUIET NO_MODULE)  # 1st: Try to locate the *config.cmake file.
if(NOT SuiteSparse_FOUND)
	# Only use the FindSuiteSparse.cmake module in Unix or if the user explicitly enables it in Windows:
	if(WIN32)
		set(DEFAULT_SUITESPARSE_USE_FIND_MODULE "OFF")
	else()
		set(DEFAULT_SUITESPARSE_USE_FIND_MODULE "ON")
	endif()
	set(SUITESPARSE_USE_FIND_MODULE ${DEFAULT_SUITESPARSE_USE_FIND_MODULE} CACHE BOOL "Use CMake module to locate SuiteSparse?")

	if(SUITESPARSE_USE_FIND_MODULE)
		set(SuiteSparse_VERBOSE OFF)
		find_package(SuiteSparse QUIET) # 2nd: Use FindSuiteSparse.cmake module
	endif()
else()
		if($ENV{VERBOSE})
			message(STATUS "Find SuiteSparse : include(${USE_SuiteSparse})")
		endif()
		include(${USE_SuiteSparse})
endif()

if(SuiteSparse_FOUND)
	if($ENV{VERBOSE})
		message(STATUS "SuiteSparse_LIBRARIES    : ${SuiteSparse_LIBRARIES}")
		message(STATUS "SuiteSparse_INCLUDE_DIRS : ${SuiteSparse_INCLUDE_DIRS}")
		if (TARGET SuiteSparse::CXSparse)
			set(x_ "DOES exist")
		else()
			set(x_ "Does NOT exist")
		endif()
		message(STATUS "SuiteSparse::CXSparse    : ${x_}")
		unset(x_)
	endif()

	set(CMAKE_MRPT_HAS_SUITESPARSE 1)
	set(CMAKE_MRPT_HAS_SUITESPARSE_SYSTEM 1)
endif()
