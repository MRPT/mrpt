
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

LIST(APPEND base_EXTRA_SRCS			"${MRPT_LIBS_ROOT}/base/src/utils/xmlparser/*.cpp" "${MRPT_LIBS_ROOT}/base/src/utils/xmlparser/*.h")
LIST(APPEND base_EXTRA_SRCS_NAME 	"utils/xmlparser" "utils/xmlparser")

# CSparse
IF(NOT CMAKE_MRPT_HAS_SUITESPARSE)
	LIST(APPEND base_EXTRA_SRCS		"${MRPT_LIBS_ROOT}/base/src/math/CSparse/*.c" "${MRPT_LIBS_ROOT}/base/include/mrpt/otherlibs/CSparse/*.h")
	LIST(APPEND base_EXTRA_SRCS_NAME 	"math/CSparse" "math/CSparse")
	INCLUDE_DIRECTORIES("${MRPT_LIBS_ROOT}/base/include/mrpt/otherlibs/CSparse/")
ENDIF()

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
ENDIF ()

# Embedded version of zlib, if really needed:
# Only include zlib if not available as system library OR it's already in wxWidgets:
IF(CMAKE_MRPT_HAS_ZLIB)
	IF(CMAKE_MRPT_HAS_ZLIB_SYSTEM_IS_WX OR NOT CMAKE_MRPT_HAS_ZLIB_SYSTEM)
		INCLUDE_DIRECTORIES("${MRPT_LIBS_ROOT}/base/src/compress/zlib/")
		IF(NOT MSVC OR NOT CMAKE_MRPT_HAS_WXWIDGETS)
			LIST(APPEND base_EXTRA_SRCS			"${MRPT_LIBS_ROOT}/base/src/compress/zlib/*.c" "${MRPT_LIBS_ROOT}/base/src/compress/zlib/*.h")
			LIST(APPEND base_EXTRA_SRCS_NAME 	"compress/zlib" "compress/zlib")
		ENDIF()
	ENDIF()
ENDIF()

IF(CMAKE_MRPT_HAS_OPENCV)
	INCLUDE_DIRECTORIES(${OpenCV_INCLUDE_DIR} ${OpenCV_INCLUDE_DIRS} ${OPENCV_INCLUDE_DIRS} ${OPENCV_INCLUDEDIR})
ENDIF()

if (MSVC AND MSVC_VERSION GREATER 1800) # VS>= 2013
	LIST(APPEND base_EXTRA_SRCS			"${CMAKE_SOURCE_DIR}/debug/*.natvis")
	LIST(APPEND base_EXTRA_SRCS_NAME 	"MSVC debug natvis")
endif()

# We need pthread's
if (${CMAKE_CXX_COMPILER_ID} STREQUAL "Clang" OR CMAKE_COMPILER_IS_GNUCXX)
	IF (UNIX OR APPLE)
		SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread")
		SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -pthread")
	ENDIF()
endif()


#---------------------------------------------
# Macro declared in "DeclareMRPTLib.cmake":
#---------------------------------------------
define_mrpt_lib(
	base 		# Lib name
				# Dependencies: None
	)

IF(BUILD_mrpt-base)
	IF(WIN32)
		target_link_libraries(mrpt-base Dbghelp) # For mrpt::system::getCallStackBackTrace()
	ENDIF()

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

		ENDIF ()
	ENDIF()

	IF(CMAKE_MRPT_HAS_OPENCV)
		target_link_libraries(mrpt-base ${OpenCV_LIBRARIES})
	ENDIF()

	if (NOT "${SuiteSparse_LIBRARIES}" STREQUAL "")
		target_link_libraries(mrpt-base ${SuiteSparse_LIBRARIES})
	endif ()

	IF(CMAKE_MRPT_HAS_WXWIDGETS)
		target_link_libraries(mrpt-base ${wxWidgets_LIBRARIES})
	ENDIF()

	IF(CMAKE_MRPT_HAS_ZLIB)
		IF(NOT CMAKE_MRPT_HAS_ZLIB_SYSTEM_IS_WX AND CMAKE_MRPT_HAS_ZLIB_SYSTEM)
			target_link_libraries(mrpt-base ${MRPT_ZLIB_LIBS})
		ENDIF()
	ENDIF()

	IF(CMAKE_MRPT_HAS_JPEG)
		target_link_libraries(mrpt-base ${JPEG_LIBS})
	ENDIF()
ENDIF()
