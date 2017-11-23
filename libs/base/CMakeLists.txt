
# Lists of directories with source files:
#  See "DeclareMRPTLib.cmake" for explanations
# -------------------------------------------------
macro(add_base_srcs _PATH)
	LIST(APPEND base_EXTRA_SRCS			"${MRPT_LIBS_ROOT}/base/src/${_PATH}/*.cpp" "${MRPT_LIBS_ROOT}/base/include/mrpt/${_PATH}/*.h")
	LIST(APPEND base_EXTRA_SRCS_NAME 	"${_PATH}" "${_PATH}")
endmacro(add_base_srcs)

add_base_srcs(bayes)
add_base_srcs(compress)
add_base_srcs(poses)
add_base_srcs(utils)

LIST(APPEND base_EXTRA_SRCS			"${MRPT_LIBS_ROOT}/base/src/utils/xmlparser/*.cpp" "${MRPT_LIBS_ROOT}/base/src/utils/xmlparser/*.h")
LIST(APPEND base_EXTRA_SRCS_NAME 	"utils/xmlparser" "utils/xmlparser")

# Embedded version of libjpeg for (mostly) Windows:
IF (CMAKE_MRPT_HAS_JPEG AND NOT CMAKE_MRPT_HAS_JPEG_SYSTEM)
	LIST(APPEND base_EXTRA_SRCS			"${MRPT_LIBS_ROOT}/base/src/utils/jpeglib/*.cpp" "${MRPT_LIBS_ROOT}/base/src/utils/jpeglib/*.h")
	LIST(APPEND base_EXTRA_SRCS_NAME 	"utils/jpeglib" "utils/jpeglib")
ENDIF ()


IF(CMAKE_MRPT_HAS_OPENCV)
	INCLUDE_DIRECTORIES(${OpenCV_INCLUDE_DIR} ${OpenCV_INCLUDE_DIRS} ${OPENCV_INCLUDE_DIRS} ${OPENCV_INCLUDEDIR})
ENDIF()

# We need pthread's in C files:
if ((${CMAKE_CXX_COMPILER_ID} STREQUAL "Clang" OR CMAKE_COMPILER_IS_GNUCXX) AND NOT WIN32)
	add_compile_options(-pthread)
endif()

#---------------------------------------------
# Macro declared in "DeclareMRPTLib.cmake":
#---------------------------------------------
define_mrpt_lib(
	base 		# Lib name
	# Dependencies:
	mrpt-core
	mrpt-system
	)

IF(BUILD_mrpt-base)
	# We need pthread's on unices
	target_link_libraries(mrpt-base
		PRIVATE
		Threads::Threads)

	# Ignore precompiled headers in some sources:
	IF(MRPT_ENABLE_PRECOMPILED_HDRS)
		IF (MSVC)
			set_source_files_properties(
				${utils/xmlparser_FILES}
				${utils/jpeglib_FILES}
				PROPERTIES COMPILE_FLAGS "/Y-"
				)

		ENDIF ()
	ENDIF()

	IF(CMAKE_MRPT_HAS_OPENCV)
		target_link_libraries(mrpt-base PRIVATE ${OpenCV_LIBRARIES})
	ENDIF()

	IF(CMAKE_MRPT_HAS_JPEG)
		target_link_libraries(mrpt-base PRIVATE ${JPEG_LIBS})
	ENDIF()
ENDIF()
