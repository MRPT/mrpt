# Check for ffmpeg C libraries: libavcodec, libavutil, libavformat, libswscale
#  These libs are linked against mrpt-hwdrivers only (in shared libs,
#  in static all user apps will have to link against this)
# ====================================================================
set(CMAKE_MRPT_HAS_FFMPEG 0)
set(CMAKE_MRPT_HAS_FFMPEG_SYSTEM 0)
set(MRPT_FFMPEG_LIBRARIES "")
set(MRPT_FFMPEG_INCLUDE_DIRS "")
set(MRPT_FFMPEG_LINK_DIRS "")

# DISABLE_FFMPEG
# ---------------------
option(DISABLE_FFMPEG "Force not using FFMPEG library" "OFF")
mark_as_advanced(DISABLE_FFMPEG)
if(DISABLE_FFMPEG)
	return()
endif()

if(PKG_CONFIG_FOUND)
	PKG_CHECK_MODULES(LIBAVCODEC  QUIET libavcodec)
	PKG_CHECK_MODULES(LIBAVUTIL   QUIET libavutil)
	PKG_CHECK_MODULES(LIBAVFORMAT QUIET libavformat)
	PKG_CHECK_MODULES(LIBSWSCALE  QUIET libswscale)
	if(LIBAVCODEC_FOUND AND LIBAVUTIL_FOUND AND LIBAVFORMAT_FOUND AND LIBSWSCALE_FOUND)
		set(CMAKE_MRPT_HAS_FFMPEG 1)
		set(CMAKE_MRPT_HAS_FFMPEG_SYSTEM 1)

		if (NOT "${LIBAVCODEC_INCLUDEDIR}" STREQUAL "")
		    list(APPEND MRPT_FFMPEG_INCLUDE_DIRS "${LIBAVCODEC_INCLUDEDIR}/")
			list(APPEND MRPT_FFMPEG_INCLUDE_DIRS "${LIBAVCODEC_INCLUDEDIR}/ffmpeg")
			list(APPEND MRPT_FFMPEG_INCLUDE_DIRS "${LIBAVCODEC_INCLUDEDIR}/libavcodec")
			list(APPEND MRPT_FFMPEG_INCLUDE_DIRS "${LIBAVUTIL_INCLUDEDIR}")
		endif()

		if (NOT "${LIBAVFORMAT_INCLUDEDIR}" STREQUAL "")
		    list(APPEND MRPT_FFMPEG_INCLUDE_DIRS "${LIBAVFORMAT_INCLUDEDIR}")
			list(APPEND MRPT_FFMPEG_INCLUDE_DIRS "${LIBAVFORMAT_INCLUDEDIR}/ffmpeg")
			list(APPEND MRPT_FFMPEG_INCLUDE_DIRS "${LIBAVFORMAT_INCLUDEDIR}/libavformat")
		endif()

		if (NOT "${LIBSWSCALE_INCLUDEDIR}" STREQUAL "")
		    list(APPEND MRPT_FFMPEG_INCLUDE_DIRS "${LIBSWSCALE_INCLUDEDIR}")
			list(APPEND MRPT_FFMPEG_INCLUDE_DIRS "${LIBSWSCALE_INCLUDEDIR}/ffmpeg")
			list(APPEND MRPT_FFMPEG_INCLUDE_DIRS "${LIBSWSCALE_INCLUDEDIR}/libswscale")
		endif()

		if (NOT "${LIBAVCODEC_LIBDIR}" STREQUAL "")
		    list(APPEND MRPT_FFMPEG_LINK_DIRS "${LIBAVCODEC_LIBDIR}")
		endif()
		if (NOT "${LIBAVUTIL_LIBDIR}" STREQUAL "")
		    list(APPEND MRPT_FFMPEG_LINK_DIRS "${LIBAVUTIL_LIBDIR}")
		endif()
		if (NOT "${LIBAVFORMAT_LIBDIR}" STREQUAL "")
		    list(APPEND MRPT_FFMPEG_LINK_DIRS "${LIBAVFORMAT_LIBDIR}")
		endif()
		if (NOT "${LIBSWSCALE_LIBDIR}" STREQUAL "")
		    list(APPEND MRPT_FFMPEG_LINK_DIRS "${LIBSWSCALE_LIBDIR}")
		endif()

		set(MRPT_FFMPEG_LIBRARIES "${MRPT_FFMPEG_LIBRARIES}" "${LIBAVCODEC_LIBRARIES}" "${LIBAVFORMAT_LIBRARIES}" "${LIBAVUTIL_LIBRARIES}" "${LIBSWSCALE_LIBRARIES}")
	endif()
endif()

if(MSVC)
	set( MRPT_HAS_FFMPEG_WIN32 OFF CACHE BOOL "Add support for IP cameras and all FFmpeg-capable video sources")
endif()

if(MRPT_HAS_FFMPEG_WIN32)
	set( FFMPEG_WIN32_ROOT_DIR "" CACHE PATH "Path to FFmpeg win32 build root directory (See http://ffmpeg.arrozcru.org/builds/)")

	# Set to 1, next check for missing things and set to 0 on any error & report message:
	set(CMAKE_MRPT_HAS_FFMPEG 1)
	set(CMAKE_MRPT_HAS_FFMPEG_SYSTEM 1)

	if(NOT EXISTS "${FFMPEG_WIN32_ROOT_DIR}")
		set(CMAKE_MRPT_HAS_FFMPEG 0)
		set(CMAKE_MRPT_HAS_FFMPEG_SYSTEM 0)
		message("The directory 'FFMPEG_WIN32_ROOT_DIR' does not exists. Turn off FFmpeg support or provide the correct path.")
	endif(NOT EXISTS "${FFMPEG_WIN32_ROOT_DIR}")

	if(NOT EXISTS "${FFMPEG_WIN32_ROOT_DIR}/include/libavcodec" OR NOT EXISTS "${FFMPEG_WIN32_ROOT_DIR}/include/libavformat" OR NOT EXISTS "${FFMPEG_WIN32_ROOT_DIR}/include/libavutil" OR NOT EXISTS "${FFMPEG_WIN32_ROOT_DIR}/include/libswscale")
		set(CMAKE_MRPT_HAS_FFMPEG 0)
		set(CMAKE_MRPT_HAS_FFMPEG_SYSTEM 0)
		message("The directory 'FFMPEG_WIN32_ROOT_DIR' does not contain include/{libavcodec,libavformat,libavutil,libswscale}. Turn off FFmpeg support or provide the correct path.")
	endif(NOT EXISTS "${FFMPEG_WIN32_ROOT_DIR}/include/libavcodec" OR NOT EXISTS "${FFMPEG_WIN32_ROOT_DIR}/include/libavformat" OR NOT EXISTS "${FFMPEG_WIN32_ROOT_DIR}/include/libavutil" OR NOT EXISTS "${FFMPEG_WIN32_ROOT_DIR}/include/libswscale")

	if(NOT EXISTS "${FFMPEG_WIN32_ROOT_DIR}/lib")
		set(CMAKE_MRPT_HAS_FFMPEG 0)
		set(CMAKE_MRPT_HAS_FFMPEG_SYSTEM 0)
		message("The directory 'FFMPEG_WIN32_ROOT_DIR' does not contain /lib/. Turn off FFmpeg support or provide the correct path.")
	endif(NOT EXISTS "${FFMPEG_WIN32_ROOT_DIR}/lib")

	# We need the .lib files: avcodec-52.lib, avformat-52.lib, avutil-49.lib, swscale-0.lib
	file(GLOB FFMPEG_WIN32_AVCODEC_LIB "${FFMPEG_WIN32_ROOT_DIR}/lib/avcodec*.lib")
	file(GLOB FFMPEG_WIN32_AVUTIL_LIB "${FFMPEG_WIN32_ROOT_DIR}/lib/avutil*.lib")
	file(GLOB FFMPEG_WIN32_AVFORMAT_LIB "${FFMPEG_WIN32_ROOT_DIR}/lib/avformat*.lib")
	file(GLOB FFMPEG_WIN32_SWSCALE_LIB "${FFMPEG_WIN32_ROOT_DIR}/lib/swscale*.lib")

	if (NOT EXISTS ${FFMPEG_WIN32_AVCODEC_LIB})
		set(CMAKE_MRPT_HAS_FFMPEG 0)
		set(CMAKE_MRPT_HAS_FFMPEG_SYSTEM 0)
		message("avcodec-XX.lib not found under '${FFMPEG_WIN32_ROOT_DIR}/lib'. Turn off FFmpeg support or provide the correct path.")
	endif (NOT EXISTS ${FFMPEG_WIN32_AVCODEC_LIB})

	if (NOT EXISTS ${FFMPEG_WIN32_AVUTIL_LIB})
		set(CMAKE_MRPT_HAS_FFMPEG 0)
		set(CMAKE_MRPT_HAS_FFMPEG_SYSTEM 0)
		message("avutil-XX.lib not found under '${FFMPEG_WIN32_ROOT_DIR}/lib'. Turn off FFmpeg support or provide the correct path.")
	endif (NOT EXISTS ${FFMPEG_WIN32_AVUTIL_LIB})

	if (NOT EXISTS ${FFMPEG_WIN32_AVFORMAT_LIB})
		set(CMAKE_MRPT_HAS_FFMPEG 0)
		set(CMAKE_MRPT_HAS_FFMPEG_SYSTEM 0)
		message("avformat-XX.lib not found under '${FFMPEG_WIN32_ROOT_DIR}/lib'. Turn off FFmpeg support or provide the correct path.")
	endif (NOT EXISTS ${FFMPEG_WIN32_AVFORMAT_LIB})

	if (NOT EXISTS ${FFMPEG_WIN32_SWSCALE_LIB})
		set(CMAKE_MRPT_HAS_FFMPEG 0)
		set(CMAKE_MRPT_HAS_FFMPEG_SYSTEM 0)
		message("swscale-XX.lib not found under '${FFMPEG_WIN32_ROOT_DIR}/lib'. Turn off FFmpeg support or provide the correct path.")
	endif (NOT EXISTS ${FFMPEG_WIN32_SWSCALE_LIB})

	list(APPEND MRPT_FFMPEG_INCLUDE_DIRS "${FFMPEG_WIN32_ROOT_DIR}/include")
	list(APPEND MRPT_FFMPEG_INCLUDE_DIRS "${FFMPEG_WIN32_ROOT_DIR}/include/libavcodec")
	list(APPEND MRPT_FFMPEG_INCLUDE_DIRS "${FFMPEG_WIN32_ROOT_DIR}/include/libavformat")
	#list(APPEND MRPT_FFMPEG_INCLUDE_DIRS "${FFMPEG_WIN32_ROOT_DIR}/include/libavutil")
	list(APPEND MRPT_FFMPEG_INCLUDE_DIRS "${FFMPEG_WIN32_ROOT_DIR}/include/libswscale")

	set(MRPT_FFMPEG_LIBRARIES ${MRPT_FFMPEG_LIBRARIES} "${FFMPEG_WIN32_AVCODEC_LIB}" "${FFMPEG_WIN32_AVUTIL_LIB}" "${FFMPEG_WIN32_AVFORMAT_LIB}" "${FFMPEG_WIN32_SWSCALE_LIB}")
endif()

if($ENV{VERBOSE})
    message(STATUS " ffmpeg libs: ${MRPT_FFMPEG_LIBRARIES}")
endif()

add_library(imp_ffmpeg INTERFACE IMPORTED)
set_target_properties(imp_ffmpeg
    PROPERTIES
	INTERFACE_INCLUDE_DIRECTORIES "${MRPT_FFMPEG_INCLUDE_DIRS}"
	INTERFACE_LINK_LIBRARIES "${MRPT_FFMPEG_LIBRARIES}"
	INTERFACE_LINK_DIRECTORIES "${MRPT_FFMPEG_LINK_DIRS}"
	)


# -- install DLLs --
if(WIN32)
	if (EXISTS "${FFMPEG_WIN32_ROOT_DIR}/bin")
		file(GLOB_RECURSE EXTRA_DLLS "${FFMPEG_WIN32_ROOT_DIR}/bin/*.dll")
		foreach(F ${EXTRA_DLLS})
			install(FILES "${F}" DESTINATION bin)
		endforeach()
	endif()
endif()
