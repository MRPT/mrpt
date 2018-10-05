#  See "DeclareMRPTLib.cmake"
list(APPEND io_EXTRA_SRCS		"${MRPT_LIBS_ROOT}/io/*.cpp" "${MRPT_LIBS_ROOT}/io/*.h")
list(APPEND io_EXTRA_SRCS_NAME 	"io" "io")

# Embedded version of zlib, if really needed:
# Only include zlib if not available as system library OR it's already in wxWidgets:
if(CMAKE_MRPT_HAS_ZLIB)
	if(CMAKE_MRPT_HAS_ZLIB_SYSTEM_IS_WX OR NOT CMAKE_MRPT_HAS_ZLIB_SYSTEM)
		include_directories("${MRPT_LIBS_ROOT}/io/src/zlib/")
		if(NOT MSVC OR NOT CMAKE_MRPT_HAS_ZLIB_SYSTEM_IS_WX)
			list(APPEND io_EXTRA_SRCS			"${MRPT_LIBS_ROOT}/io/src/zlib/*.c" "${MRPT_LIBS_ROOT}/io/src/zlib/*.h")
			list(APPEND io_EXTRA_SRCS_NAME 	"io/zlib" "io/zlib")
		endif()
	endif()
endif()

define_mrpt_lib(
	io 		# Lib name
	# Dependencies:
	mrpt-core
	mrpt-system
	)

if(BUILD_mrpt-io)
	# Ignore precompiled headers in some sources:
	if(MRPT_ENABLE_PRECOMPILED_HDRS AND MSVC)
		set_source_files_properties(
			${io/zlib_FILES}
			PROPERTIES COMPILE_FLAGS "/Y-"
			)
	endif()

	if(CMAKE_MRPT_HAS_ZLIB)
		if(NOT CMAKE_MRPT_HAS_ZLIB_SYSTEM_IS_WX AND CMAKE_MRPT_HAS_ZLIB_SYSTEM)
			target_link_libraries(mrpt-io PRIVATE ${MRPT_ZLIB_LIBS})
		endif()
	endif()

	# Use wxWidgets version of libzip (gz* funtions)
	if(MSVC AND CMAKE_MRPT_HAS_WXWIDGETS)
		target_link_libraries(mrpt-io PRIVATE ${wxWidgets_LIBRARIES})
	endif()
endif()
