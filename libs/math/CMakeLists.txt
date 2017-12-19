#  See "DeclareMRPTLib.cmake"
LIST(APPEND math_EXTRA_SRCS		"${MRPT_LIBS_ROOT}/math/*.cpp" "${MRPT_LIBS_ROOT}/math/*.h")
LIST(APPEND math_EXTRA_SRCS_NAME 	"math" "math")

# CSparse
IF(NOT CMAKE_MRPT_HAS_SUITESPARSE)
	LIST(APPEND math_EXTRA_SRCS		"${MRPT_LIBS_ROOT}/math/src/CSparse/*.c" "${MRPT_LIBS_ROOT}/math/include/mrpt/otherlibs/CSparse/*.h")
	LIST(APPEND math_EXTRA_SRCS_NAME 	"math/CSparse" "math/CSparse")
	INCLUDE_DIRECTORIES("${MRPT_LIBS_ROOT}/math/include/mrpt/otherlibs/CSparse/")
ENDIF()

# kmeans++
LIST(APPEND math_EXTRA_SRCS		"${MRPT_LIBS_ROOT}/math/src/kmeans++/*.cpp" "${MRPT_LIBS_ROOT}/math/src/kmeans++/*.h")
LIST(APPEND math_EXTRA_SRCS_NAME 	"math/kmeans" "math/kmeans")

# debug visualizer for Eigen matrices
if (MSVC AND MSVC_VERSION GREATER 1800) # VS>= 2013
	LIST(APPEND math_EXTRA_SRCS			"${CMAKE_SOURCE_DIR}/debug/*.natvis")
	LIST(APPEND math_EXTRA_SRCS_NAME 	"MSVC debug natvis")
endif()

define_mrpt_lib(
	math 		# Lib name
	# Dependencies:
	mrpt-core
	mrpt-containers
	mrpt-typemeta
	mrpt-serialization
	mrpt-random
	mrpt-system
	)

IF(BUILD_mrpt-math)
	# Ignore precompiled headers in some sources:
	IF(MRPT_ENABLE_PRECOMPILED_HDRS AND MSVC)
		set_source_files_properties(
			${math/CSparse_FILES}
			${math/kmeans_FILES}
			PROPERTIES COMPILE_FLAGS "/Y-"
			)
	ENDIF()

	if (NOT "${SuiteSparse_LIBRARIES}" STREQUAL "")
		target_link_libraries(mrpt-math PRIVATE ${SuiteSparse_LIBRARIES})
	endif ()
ENDIF()
