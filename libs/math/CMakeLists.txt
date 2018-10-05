#  See "DeclareMRPTLib.cmake"
list(APPEND math_EXTRA_SRCS		"${MRPT_LIBS_ROOT}/math/*.cpp" "${MRPT_LIBS_ROOT}/math/*.h")
list(APPEND math_EXTRA_SRCS_NAME 	"math" "math")

# CSparse
if(NOT CMAKE_MRPT_HAS_SUITESPARSE)
	list(APPEND math_EXTRA_SRCS		"${MRPT_LIBS_ROOT}/math/src/CSparse/*.c" "${MRPT_LIBS_ROOT}/math/include/mrpt/otherlibs/CSparse/*.h")
	list(APPEND math_EXTRA_SRCS_NAME 	"math/CSparse" "math/CSparse")
	include_directories("${MRPT_LIBS_ROOT}/math/include/mrpt/otherlibs/CSparse/")
endif()

# kmeans++
list(APPEND math_EXTRA_SRCS		"${MRPT_LIBS_ROOT}/math/src/kmeans++/*.cpp" "${MRPT_LIBS_ROOT}/math/src/kmeans++/*.h")
list(APPEND math_EXTRA_SRCS_NAME 	"math/kmeans" "math/kmeans")

# debug visualizer for Eigen matrices
if (MSVC AND MSVC_VERSION GREATER 1800) # VS>= 2013
	list(APPEND math_EXTRA_SRCS			"${CMAKE_SOURCE_DIR}/debug/*.natvis")
	list(APPEND math_EXTRA_SRCS_NAME 	"MSVC debug natvis")
endif()

# extra dependencies required by unit tests in this module:
set_property(GLOBAL PROPERTY mrpt_math_UNIT_TEST_EXTRA_DEPS mrpt-io)

define_mrpt_lib(
	math 		# Lib name
	# Dependencies:
	mrpt-core
	mrpt-containers
	mrpt-typemeta
	mrpt-serialization
	mrpt-random
	mrpt-system
	mrpt-nanoflann
	)

if(BUILD_mrpt-math)
	# Ignore precompiled headers in some sources:
	if(MRPT_ENABLE_PRECOMPILED_HDRS AND MSVC)
		set_source_files_properties(
			${math/CSparse_FILES}
			${math/kmeans_FILES}
			PROPERTIES COMPILE_FLAGS "/Y-"
			)
	endif()

	if (NOT "${SuiteSparse_LIBRARIES}" STREQUAL "")
		target_link_libraries(mrpt-math PRIVATE ${SuiteSparse_LIBRARIES})
	endif ()
endif()
