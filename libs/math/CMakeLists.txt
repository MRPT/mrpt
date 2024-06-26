﻿#  See "DeclareMRPTLib.cmake"
list(APPEND math_EXTRA_SRCS		"${MRPT_LIBS_ROOT}/math/*.cpp" "${MRPT_LIBS_ROOT}/math/*.h")
list(APPEND math_EXTRA_SRCS_NAME 	"math" "math")

# CSparse
if(NOT CMAKE_MRPT_HAS_SUITESPARSE)
	list(APPEND math_EXTRA_SRCS		"${MRPT_LIBS_ROOT}/math/src/CSparse/*.c" "${MRPT_LIBS_ROOT}/math/include/mrpt/3rdparty/CSparse/*.h")
	list(APPEND math_EXTRA_SRCS_NAME 	"math/CSparse" "math/CSparse")
	include_directories("${MRPT_LIBS_ROOT}/math/include/mrpt/3rdparty/CSparse/")
endif()

# kmeans++
list(APPEND math_EXTRA_SRCS		"${MRPT_LIBS_ROOT}/math/src/kmeans++/*.cpp" "${MRPT_LIBS_ROOT}/math/src/kmeans++/*.h")
list(APPEND math_EXTRA_SRCS_NAME 	"math/kmeans" "math/kmeans")

# debug visualizer for Eigen matrices
if (MSVC AND MSVC_VERSION GREATER 1800) # VS>= 2013
	list(APPEND math_EXTRA_SRCS			"${MRPT_SOURCE_DIR}/debug/*.natvis")
	list(APPEND math_EXTRA_SRCS_NAME 	"MSVC debug natvis")
endif()

# extra dependencies required by unit tests in this module:
set_property(GLOBAL PROPERTY mrpt_math_UNIT_TEST_EXTRA_DEPS mrpt-io)

# eigen3:
if (MRPT_EIGEN_USE_EMBEDDED_VERSION)
	set(eigen_dep EP_eigen3)
else()
	set(eigen_dep Eigen3) # find_package() lib name
endif()

# For Ubuntu 16.04, the Eigen version does not support imported target,
# so create it ourselves in the mrpt-math-config.cmake file:
set(EXTRA_CONFIG_CMDS
"if (NOT TARGET Eigen3::Eigen)\n\
	add_library(Eigen3::Eigen INTERFACE IMPORTED GLOBAL)\n\
	if (TARGET Eigen)\n\
		target_link_libraries(Eigen3::Eigen INTERFACE Eigen)\n\
	else()\n\
		if(EXISTS \"${EIGEN3_INCLUDE_DIR}\")\n\
			set(eigen_dirs ${EIGEN3_INCLUDE_DIR})\n\
		else()\n\
			set(eigen_dirs ${EIGEN_EMBEDDED_INCLUDE_DIR})\n\
		endif()\n\
		set_target_properties(Eigen3::Eigen PROPERTIES\n\
			INTERFACE_INCLUDE_DIRECTORIES\n\
				$<BUILD_INTERFACE:\${eigen_dirs}>\n\
		)\n\
	endif()\n\
endif()\n"
)

define_mrpt_lib(
	math 		# Lib name
	# Dependencies:
	mrpt-serialization
	mrpt-random
	mrpt-system
	# Other imported targets:
	nanoflann # find_package() lib name
	${eigen_dep}
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

	# eigen3:
	target_link_libraries(math PUBLIC Eigen3::Eigen)
	if (MSVC)
		# This is to prevent the error (in Debug builds):
		# LNK1189：library limit of 65535 objects exceeded
		target_compile_options(math PRIVATE /Ob2)
	endif()

	# nanoflann:
	target_link_libraries(math PUBLIC nanoflann::nanoflann)

	if (TARGET SuiteSparse::CXSparse)
		# Modern cmake interface?
		target_link_libraries(math PUBLIC SuiteSparse::CXSparse)
	else()
		if (NOT "${SuiteSparse_LIBRARIES}" STREQUAL "")
			target_link_libraries(math PRIVATE ${SuiteSparse_LIBRARIES})
		endif ()
		if (NOT "${SuiteSparse_LIBRARIES}" STREQUAL "")
			target_include_directories(math PUBLIC ${SuiteSparse_INCLUDE_DIRS})
		endif ()
	endif()

	# Minimize debug info for this module:
	#mrpt_reduced_debug_symbols(math)
endif()
