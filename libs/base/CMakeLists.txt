
# Lists of directories with source files:
#  See "DeclareMRPTLib.cmake" for explanations
# -------------------------------------------------
macro(add_base_srcs _PATH)
	LIST(APPEND base_EXTRA_SRCS			"${MRPT_LIBS_ROOT}/base/src/${_PATH}/*.cpp" "${MRPT_LIBS_ROOT}/base/include/mrpt/${_PATH}/*.h")
	LIST(APPEND base_EXTRA_SRCS_NAME 	"${_PATH}" "${_PATH}")
endmacro(add_base_srcs)

add_base_srcs(bayes)
add_base_srcs(compress)
add_base_srcs(math)
add_base_srcs(poses)
add_base_srcs(random)
add_base_srcs(synch)
add_base_srcs(system)
add_base_srcs(utils)

LIST(APPEND base_EXTRA_SRCS       "${MRPT_LIBS_ROOT}/base/src/utils/simpleini/*.h")
LIST(APPEND base_EXTRA_SRCS_NAME  "utils/simpleini")
IF(NOT CMAKE_MRPT_HAS_CLANG_SYSTEM)
	LIST(APPEND base_EXTRA_SRCS       "${MRPT_LIBS_ROOT}/base/src/utils/simpleini/*.c")
	LIST(APPEND base_EXTRA_SRCS_NAME  "utils/simpleini")
ENDIF(NOT CMAKE_MRPT_HAS_CLANG_SYSTEM)

LIST(APPEND base_EXTRA_SRCS			"${MRPT_LIBS_ROOT}/base/src/utils/xmlparser/*.cpp" "${MRPT_LIBS_ROOT}/base/src/utils/xmlparser/*.h")
LIST(APPEND base_EXTRA_SRCS_NAME 	"utils/xmlparser" "utils/xmlparser")

# CSparse
IF(NOT CMAKE_MRPT_HAS_SUITESPARSE)
	LIST(APPEND base_EXTRA_SRCS		"${MRPT_LIBS_ROOT}/base/src/math/CSparse/*.c" "${MRPT_LIBS_ROOT}/base/include/mrpt/otherlibs/CSparse/*.h")
	LIST(APPEND base_EXTRA_SRCS_NAME 	"math/CSparse" "math/CSparse")
	INCLUDE_DIRECTORIES("${MRPT_LIBS_ROOT}/base/include/mrpt/otherlibs/CSparse/")
ENDIF(NOT CMAKE_MRPT_HAS_SUITESPARSE)

# Eigen library
LIST(APPEND base_EXTRA_SRCS		"${MRPT_LIBS_ROOT}/base/include/Eigen/*")
LIST(APPEND base_EXTRA_SRCS_NAME 	"eigen_lib")

# kmeans++
LIST(APPEND base_EXTRA_SRCS		"${MRPT_LIBS_ROOT}/base/src/math/kmeans++/*.cpp" "${MRPT_LIBS_ROOT}/base/src/math/kmeans++/*.h")
LIST(APPEND base_EXTRA_SRCS_NAME 	"math/kmeans" "math/kmeans")

# Embedded version of libjpeg for (mostly) Windows:
IF (CMAKE_MRPT_HAS_JPEG AND NOT CMAKE_MRPT_HAS_JPEG_SYSTEM)
	LIST(APPEND base_EXTRA_SRCS			"${MRPT_LIBS_ROOT}/base/src/utils/jpeglib/*.cpp" "${MRPT_LIBS_ROOT}/base/src/utils/jpeglib/*.h")
	LIST(APPEND base_EXTRA_SRCS_NAME 	"utils/jpeglib" "utils/jpeglib")
ENDIF (CMAKE_MRPT_HAS_JPEG AND NOT CMAKE_MRPT_HAS_JPEG_SYSTEM)

# Embedded version of zlib, if really needed:
# Only include zlib if not available as system library OR it's already in wxWidgets:
IF(CMAKE_MRPT_HAS_ZLIB)
	IF (CMAKE_MRPT_HAS_ZLIB_SYSTEM_IS_WX OR NOT CMAKE_MRPT_HAS_ZLIB_SYSTEM)
		INCLUDE_DIRECTORIES("${MRPT_LIBS_ROOT}/base/src/compress/zlib/")
		IF (NOT WIN32 OR NOT CMAKE_MRPT_HAS_WXWIDGETS)
			LIST(APPEND base_EXTRA_SRCS			"${MRPT_LIBS_ROOT}/base/src/compress/zlib/*.c" "${MRPT_LIBS_ROOT}/base/src/compress/zlib/*.h")
			LIST(APPEND base_EXTRA_SRCS_NAME 	"compress/zlib" "compress/zlib")
		ENDIF (NOT WIN32 OR NOT CMAKE_MRPT_HAS_WXWIDGETS)
	ENDIF (CMAKE_MRPT_HAS_ZLIB_SYSTEM_IS_WX OR NOT CMAKE_MRPT_HAS_ZLIB_SYSTEM)
ENDIF(CMAKE_MRPT_HAS_ZLIB)

IF(CMAKE_MRPT_HAS_OPENCV)
	INCLUDE_DIRECTORIES(${OpenCV_INCLUDE_DIR} ${OpenCV_INCLUDE_DIRS} ${OPENCV_INCLUDE_DIRS} ${OPENCV_INCLUDEDIR})
ENDIF(CMAKE_MRPT_HAS_OPENCV)

# mexplus header-only library for interaction with Matlab MEX libraries
IF(CMAKE_MRPT_HAS_MATLAB)
        INCLUDE_DIRECTORIES("${MRPT_SOURCE_DIR}/otherlibs/mexplus/")
ENDIF(CMAKE_MRPT_HAS_MATLAB)

# Documentation files:
LIST(APPEND base_EXTRA_SRCS			"${CMAKE_SOURCE_DIR}/doc/doxygen-pages/*.h")
LIST(APPEND base_EXTRA_SRCS_NAME 	"Documentation")

#---------------------------------------------
# Macro declared in "DeclareMRPTLib.cmake":
#---------------------------------------------
define_mrpt_lib(
	base 		# Lib name
				# Dependencies: None
	)

IF(BUILD_mrpt-base) 
	# Ignore precompiled headers in some sources:
	IF(MRPT_ENABLE_PRECOMPILED_HDRS)
		IF (MSVC)
			set_source_files_properties(
				${math/CSparse_FILES}
				${math/kmeans_FILES}
				${utils/xmlparser_FILES}
				${utils/simpleini_FILES}	
				${utils/jpeglib_FILES}
				${compress/zlib_FILES}
				PROPERTIES COMPILE_FLAGS "/Y-"
				)

		ENDIF (MSVC)
	ENDIF(MRPT_ENABLE_PRECOMPILED_HDRS)

	IF(CMAKE_MRPT_HAS_OPENCV)
		target_link_libraries(mrpt-base ${OpenCV_LIBRARIES})
	ENDIF(CMAKE_MRPT_HAS_OPENCV)

	if (NOT "${SuiteSparse_LIBRARIES}" STREQUAL "")
		target_link_libraries(mrpt-base ${SuiteSparse_LIBRARIES})
	endif (NOT "${SuiteSparse_LIBRARIES}" STREQUAL "")

	IF(CMAKE_MRPT_HAS_WXWIDGETS)
		target_link_libraries(mrpt-base ${wxWidgets_LIBRARIES})
	ENDIF(CMAKE_MRPT_HAS_WXWIDGETS)


	# For GCC<4.5.1 and 64bit architectures, we need to disable vectorization at one problematic file (weird bug!)
	# JL: It seems the bug is still there for GCC 4.6.0, so disable vect. in all cases for that file :-(
	#IF(CMAKE_COMPILER_IS_GNUCXX AND "${CMAKE_GCC_REGEX_VERSION}" VERSION_LESS "4.5.1" AND CMAKE_MRPT_WORD_SIZE EQUAL 64)
	IF(CMAKE_COMPILER_IS_GNUCXX AND WIN32)
		# Skip for MinGW
	ELSE(CMAKE_COMPILER_IS_GNUCXX AND WIN32)		
		set_source_files_properties(
			${MRPT_LIBS_ROOT}/base/src/poses/CPose3DQuatPDF.cpp
			PROPERTIES COMPILE_FLAGS -DEIGEN_DONT_VECTORIZE
			)
	ENDIF(CMAKE_COMPILER_IS_GNUCXX AND WIN32)		
	#ENDIF(CMAKE_COMPILER_IS_GNUCXX AND "${CMAKE_GCC_REGEX_VERSION}" VERSION_LESS "4.5.1" AND CMAKE_MRPT_WORD_SIZE EQUAL 64)
ENDIF(BUILD_mrpt-base) 

