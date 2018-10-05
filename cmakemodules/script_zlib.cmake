# Check for system zlib:
# ===================================================
set(CMAKE_MRPT_HAS_ZLIB 1)	# Always present: system or built-in
set(CMAKE_MRPT_HAS_ZLIB_SYSTEM_IS_WX 0)
if (MSVC)
	set(CMAKE_MRPT_HAS_ZLIB_SYSTEM 0)
else()
	find_package(ZLIB)
	if (ZLIB_FOUND)
			#message(STATUS "Found library: zlib - Include: ${ZLIB_INCLUDE_DIR}")
			include_directories("${ZLIB_INCLUDE_DIR}")

			set(MRPT_ZLIB_LIBS ${ZLIB_LIBRARIES}) # APPEND_MRPT_LIBS(z)

			set(CMAKE_MRPT_HAS_ZLIB_SYSTEM 1)
			set(CMAKE_MRPT_HAS_ZLIB_SYSTEM_IS_WX 0)
	else()
			# If we are using wxWidgets we dont need this... for now check if this is MinGW on Windows...
			if (WIN32 AND CMAKE_MRPT_HAS_WXWIDGETS)
				set(CMAKE_MRPT_HAS_ZLIB_SYSTEM 1)
				set(CMAKE_MRPT_HAS_ZLIB_SYSTEM_IS_WX 1)
			else()
				set(CMAKE_MRPT_HAS_ZLIB_SYSTEM 0)
			endif()
	endif()
endif()

# Update: wxWidgets >=3.1.1 no longer comes with zlib
if (CMAKE_MRPT_HAS_WXWIDGETS AND CMAKE_MRPT_HAS_ZLIB_SYSTEM_IS_WX AND (NOT "${wxWidgets_VERSION_STRING}" VERSION_LESS "3.1.1"))
	# Use embedded version instead:
	set(CMAKE_MRPT_HAS_ZLIB_SYSTEM 0)
	set(CMAKE_MRPT_HAS_ZLIB_SYSTEM_IS_WX 0)
endif()
