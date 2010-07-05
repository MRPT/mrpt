
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
add_base_srcs(scan_matching)

LIST(APPEND base_EXTRA_SRCS			"${MRPT_LIBS_ROOT}/base/src/utils/simpleini/*.c" "${MRPT_LIBS_ROOT}/base/src/utils/simpleini/*.h")
LIST(APPEND base_EXTRA_SRCS_NAME 	"utils/simpleini" "utils/simpleini")

LIST(APPEND base_EXTRA_SRCS			"${MRPT_LIBS_ROOT}/base/src/utils/xmlparser/*.cpp" "${MRPT_LIBS_ROOT}/base/src/utils/xmlparser/*.h")
LIST(APPEND base_EXTRA_SRCS_NAME 	"utils/xmlparser" "utils/xmlparser")

# ann library:
LIST(APPEND base_EXTRA_SRCS		"${MRPT_LIBS_ROOT}/base/src/ann/*.cpp" "${MRPT_LIBS_ROOT}/base/include/mrpt/otherlibs/ann/*.h")
LIST(APPEND base_EXTRA_SRCS_NAME 	"ann" "ann")
INCLUDE_DIRECTORIES("${MRPT_LIBS_ROOT}/base/include/mrpt/otherlibs/")

LIST(APPEND base_EXTRA_SRCS		"${MRPT_LIBS_ROOT}/base/src/math/CSparse/*.c" "${MRPT_LIBS_ROOT}/base/include/mrpt/otherlibs/CSparse/*.h")
LIST(APPEND base_EXTRA_SRCS_NAME 	"math/CSparse" "math/CSparse")
INCLUDE_DIRECTORIES("${MRPT_LIBS_ROOT}/base/include/mrpt/otherlibs/CSparse/")

# Embedded version of libjpeg for (mostly) Windows:
IF (CMAKE_MRPT_HAS_JPEG AND NOT CMAKE_MRPT_HAS_JPEG_SYSTEM)
	LIST(APPEND base_EXTRA_SRCS			"${MRPT_LIBS_ROOT}/base/src/utils/jpeglib/*.cpp" "${MRPT_LIBS_ROOT}/base/src/utils/jpeglib/*.h")
	LIST(APPEND base_EXTRA_SRCS_NAME 	"utils/jpeglib" "utils/jpeglib")
ENDIF (CMAKE_MRPT_HAS_JPEG AND NOT CMAKE_MRPT_HAS_JPEG_SYSTEM)

# Embedded version of zlib, if really needed:
# Only include zlib if not available as system library OR it's already in wxWidgets:
IF(CMAKE_MRPT_HAS_ZLIB AND NOT CMAKE_MRPT_HAS_ZLIB_SYSTEM)
	INCLUDE_DIRECTORIES("${MRPT_LIBS_ROOT}/base/src/compress/zlib/")
	IF (NOT WIN32 OR NOT CMAKE_MRPT_HAS_WXWIDGETS)
		LIST(APPEND base_EXTRA_SRCS			"${MRPT_LIBS_ROOT}/base/src/compress/zlib/*.c" "${MRPT_LIBS_ROOT}/base/src/compress/zlib/*.h")
		LIST(APPEND base_EXTRA_SRCS_NAME 	"compress/zlib" "compress/zlib")
	ENDIF (NOT WIN32 OR NOT CMAKE_MRPT_HAS_WXWIDGETS)
ENDIF(CMAKE_MRPT_HAS_ZLIB AND NOT CMAKE_MRPT_HAS_ZLIB_SYSTEM)


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
	
# Ignore precompiled headers in some sources:
IF(MRPT_ENABLE_PRECOMPILED_HDRS)
	IF (MSVC)
		set_source_files_properties(
			${math/CSparse_FILES}
			${utils/xmlparser_FILES}
			${utils/simpleini_FILES}	
			${utils/jpeglib_FILES}
			${compress/zlib_FILES}
			${ann_FILES}
			PROPERTIES COMPILE_FLAGS "/Y-"
			)

	ENDIF (MSVC)
ENDIF(MRPT_ENABLE_PRECOMPILED_HDRS)
	
