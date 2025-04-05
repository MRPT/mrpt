# Check for system zlib:
# ===================================================
set(CMAKE_MRPT_HAS_ZLIB 1 CACHE INTERNAL "")	# Always present: system or built-in
set(CMAKE_MRPT_HAS_ZLIB_SYSTEM_IS_WX 0 CACHE INTERNAL "")
if (MSVC)
	set(CMAKE_MRPT_HAS_ZLIB_SYSTEM 0 CACHE INTERNAL "")
else()
	find_package(ZLIB)
	if (ZLIB_FOUND)
			#message(STATUS "Found library: zlib - Include: ${ZLIB_INCLUDE_DIR}")
			#message(STATUS "Found library: zlib - Include: ${ZLIB_LIBRARIES}")
			set(CMAKE_MRPT_HAS_ZLIB_SYSTEM 1 CACHE INTERNAL "")
			set(CMAKE_MRPT_HAS_ZLIB_SYSTEM_IS_WX 0 CACHE INTERNAL "")
	else()
			# If we are using wxWidgets we dont need this... for now check if this is MinGW on Windows...
			if (WIN32 AND CMAKE_MRPT_HAS_WXWIDGETS)
				set(CMAKE_MRPT_HAS_ZLIB_SYSTEM 1 CACHE INTERNAL "")
				set(CMAKE_MRPT_HAS_ZLIB_SYSTEM_IS_WX 1 CACHE INTERNAL "")
			else()
				set(CMAKE_MRPT_HAS_ZLIB_SYSTEM 0)
			endif()
	endif()
endif()

# Update: wxWidgets >=3.1.1 no longer comes with zlib
if (CMAKE_MRPT_HAS_WXWIDGETS AND CMAKE_MRPT_HAS_ZLIB_SYSTEM_IS_WX AND (NOT "${wxWidgets_VERSION_STRING}" VERSION_LESS "3.1.1"))
	# Use embedded version instead:
	set(CMAKE_MRPT_HAS_ZLIB_SYSTEM 0 CACHE INTERNAL "")
	set(CMAKE_MRPT_HAS_ZLIB_SYSTEM_IS_WX 0 CACHE INTERNAL "")
endif()
