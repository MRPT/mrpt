# Check for Cloudini (point cloud compression, used to serialize point maps):
# the system package first, then the git submodule in 3rdparty/cloudini.
# Defines CMAKE_MRPT_HAS_CLOUDINI, CMAKE_MRPT_HAS_CLOUDINI_SYSTEM and, if found,
# MRPT_CLOUDINI_TARGET.
set(CMAKE_MRPT_HAS_CLOUDINI 0)
set(CMAKE_MRPT_HAS_CLOUDINI_SYSTEM 0)

option(MRPT_WITH_CLOUDINI "Compress point maps with Cloudini when serializing them" ON)

# Builds the embedded copy as a static, PIC library, within this function scope
# so its options do not affect the rest of the project:
function(mrpt_add_embedded_cloudini)
	set(BUILD_SHARED_LIBS OFF)
	set(CMAKE_POSITION_INDEPENDENT_CODE ON)
	set(CLOUDINI_FORCE_VENDORED_DEPS OFF)  # use the system zstd and lz4
	set(CLOUDINI_BUILD_TOOLS OFF)
	set(CLOUDINI_BUILD_BENCHMARKS OFF)
	set(CLOUDINI_WITH_PCL OFF)
	set(CLOUDINI_INSTALL OFF)
	# Prefer the shared system zstd and lz4: their static archives may not be
	# PIC, so they could not be linked into a shared mrpt_maps. Cloudini uses
	# these targets if defined, or builds its own copies otherwise.
	find_library(MRPT_CLOUDINI_ZSTD_LIBRARY NAMES zstd)
	find_path(MRPT_CLOUDINI_ZSTD_INCLUDE_DIR NAMES zstd.h)
	if (MRPT_CLOUDINI_ZSTD_LIBRARY AND MRPT_CLOUDINI_ZSTD_INCLUDE_DIR AND NOT TARGET zstd::libzstd_shared)
		add_library(zstd::libzstd_shared UNKNOWN IMPORTED)
		set_target_properties(zstd::libzstd_shared PROPERTIES
			IMPORTED_LOCATION "${MRPT_CLOUDINI_ZSTD_LIBRARY}"
			INTERFACE_INCLUDE_DIRECTORIES "${MRPT_CLOUDINI_ZSTD_INCLUDE_DIR}")
	endif()
	find_library(MRPT_CLOUDINI_LZ4_LIBRARY NAMES lz4)
	find_path(MRPT_CLOUDINI_LZ4_INCLUDE_DIR NAMES lz4.h)
	if (MRPT_CLOUDINI_LZ4_LIBRARY AND MRPT_CLOUDINI_LZ4_INCLUDE_DIR AND NOT TARGET LZ4::lz4_shared)
		add_library(LZ4::lz4_shared UNKNOWN IMPORTED)
		set_target_properties(LZ4::lz4_shared PROPERTIES
			IMPORTED_LOCATION "${MRPT_CLOUDINI_LZ4_LIBRARY}"
			INTERFACE_INCLUDE_DIRECTORIES "${MRPT_CLOUDINI_LZ4_INCLUDE_DIR}")
	endif()
	mark_as_advanced(MRPT_CLOUDINI_ZSTD_LIBRARY MRPT_CLOUDINI_ZSTD_INCLUDE_DIR
		MRPT_CLOUDINI_LZ4_LIBRARY MRPT_CLOUDINI_LZ4_INCLUDE_DIR)
	add_subdirectory(
		${mrpt_maps_SOURCE_DIR}/3rdparty/cloudini/cloudini_lib
		${mrpt_maps_BINARY_DIR}/3rdparty/cloudini
		EXCLUDE_FROM_ALL)
	# Its codec runs much faster built with -O3 than with the -O2 of
	# RelWithDebInfo or distribution builds:
	if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
		target_compile_options(cloudini_lib PRIVATE $<$<NOT:$<CONFIG:Debug>>:-O3>)
	endif()
endfunction()

if (MRPT_WITH_CLOUDINI)
	# Cloudini streams are little-endian only:
	if ("${CMAKE_CXX_BYTE_ORDER}" STREQUAL "BIG_ENDIAN")
		message(STATUS "Cloudini: disabled on big-endian platforms")
	else()
		find_package(cloudini_lib QUIET)
		# Target names of the standalone and ROS (ament) packages:
		foreach(_target cloudini::cloudini_lib cloudini_lib::cloudini_lib)
			if (TARGET ${_target})
				set(MRPT_CLOUDINI_TARGET ${_target})
				set(CMAKE_MRPT_HAS_CLOUDINI 1)
				set(CMAKE_MRPT_HAS_CLOUDINI_SYSTEM 1)
				break()
			endif()
		endforeach()

		if (NOT CMAKE_MRPT_HAS_CLOUDINI)
			if (EXISTS "${mrpt_maps_SOURCE_DIR}/3rdparty/cloudini/cloudini_lib/CMakeLists.txt")
				mrpt_add_embedded_cloudini()
				set(MRPT_CLOUDINI_TARGET cloudini::cloudini_lib)
				set(CMAKE_MRPT_HAS_CLOUDINI 1)
			else()
				message(STATUS
					"Cloudini: not found, and the git submodule is not initialized "
					"(git submodule update --init). Point maps will be stored uncompressed.")
			endif()
		endif()
	endif()
endif()

message(STATUS "Cloudini: CMAKE_MRPT_HAS_CLOUDINI=${CMAKE_MRPT_HAS_CLOUDINI} (system: ${CMAKE_MRPT_HAS_CLOUDINI_SYSTEM})")
