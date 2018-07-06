#  See "DeclareMRPTLib.cmake"
LIST(APPEND io_EXTRA_SRCS		"${MRPT_LIBS_ROOT}/io/*.cpp" "${MRPT_LIBS_ROOT}/io/*.h")
LIST(APPEND io_EXTRA_SRCS_NAME 	"io" "io")

# Embedded version of zlib, if really needed:
# Only include zlib if not available as system library OR it's already in wxWidgets:
IF(CMAKE_MRPT_HAS_ZLIB)
	IF(CMAKE_MRPT_HAS_ZLIB_SYSTEM_IS_WX OR NOT CMAKE_MRPT_HAS_ZLIB_SYSTEM)
		INCLUDE_DIRECTORIES("${MRPT_LIBS_ROOT}/io/src/zlib/")
		IF(NOT MSVC OR NOT CMAKE_MRPT_HAS_ZLIB_SYSTEM_IS_WX)
			LIST(APPEND io_EXTRA_SRCS			"${MRPT_LIBS_ROOT}/io/src/zlib/*.c" "${MRPT_LIBS_ROOT}/io/src/zlib/*.h")
			LIST(APPEND io_EXTRA_SRCS_NAME 	"io/zlib" "io/zlib")
		ENDIF()
	ENDIF()
ENDIF()

define_mrpt_lib(
	io 		# Lib name
	# Dependencies:
	mrpt-core
	mrpt-system
	)

IF(BUILD_mrpt-io)
	# Ignore precompiled headers in some sources:
	IF(MRPT_ENABLE_PRECOMPILED_HDRS AND MSVC)
		set_source_files_properties(
			${io/zlib_FILES}
			PROPERTIES COMPILE_FLAGS "/Y-"
			)
	ENDIF()

	IF(CMAKE_MRPT_HAS_ZLIB)
		IF(NOT CMAKE_MRPT_HAS_ZLIB_SYSTEM_IS_WX AND CMAKE_MRPT_HAS_ZLIB_SYSTEM)
			target_link_libraries(mrpt-io PRIVATE ${MRPT_ZLIB_LIBS})
		ENDIF()
	ENDIF()

	# Use wxWidgets version of libzip (gz* funtions)
	IF(MSVC AND CMAKE_MRPT_HAS_WXWIDGETS)
		target_link_libraries(mrpt-io PRIVATE ${wxWidgets_LIBRARIES})
	ENDIF()
ENDIF()
