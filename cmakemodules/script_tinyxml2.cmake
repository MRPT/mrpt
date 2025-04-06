# Check for the TINYXML2 library
# ===================================================
set(CMAKE_MRPT_HAS_TINYXML2 0)
set(CMAKE_MRPT_HAS_TINYXML2_SYSTEM 0)

# This option will be available only on Linux, hence it's declared here:
option(DISABLE_TINYXML2 "Do not use the tinyxml2 library" 0)
mark_as_advanced(DISABLE_TINYXML2)

if (DISABLE_TINYXML2)
	return()
endif()

set(_use_embeded_default "OFF")
if (WIN32 OR APPLE)
	set(_use_embeded_default "ON")
endif()

set(TINYXML2_USE_EMBEDDED_VERSION ${_use_embeded_default} CACHE BOOL "Download tinyxml2 and use it instead of system version")
mark_as_advanced(TINYXML2_USE_EMBEDDED_VERSION)

unset(_use_embeded_default)

if (TINYXML2_USE_EMBEDDED_VERSION)
	# download on the fly:
	set(TINYXML2_VERSION_TO_DOWNLOAD "7.1.0" CACHE STRING "Download from this GitHub tag")

	if (NOT EXISTS "${MRPT_BINARY_DIR}/3rdparty/tinyxml2/tinyxml2.h")
		file(DOWNLOAD
			https://github.com/leethomason/tinyxml2/raw/${TINYXML2_VERSION_TO_DOWNLOAD}/tinyxml2.h
			"${MRPT_BINARY_DIR}/3rdparty/tinyxml2/tinyxml2.h" SHOW_PROGRESS)
		file(DOWNLOAD
			https://github.com/leethomason/tinyxml2/raw/${TINYXML2_VERSION_TO_DOWNLOAD}/tinyxml2.cpp
			"${MRPT_BINARY_DIR}/3rdparty/tinyxml2/tinyxml2.cpp" SHOW_PROGRESS)
	endif()

	set(CMAKE_MRPT_HAS_TINYXML2 1)
	set(CMAKE_MRPT_HAS_TINYXML2_SYSTEM 0)
endif()

if(UNIX AND NOT CMAKE_MRPT_HAS_TINYXML2)
	find_path(TINYXML2_INCLUDE_DIR tinyxml2.h)
	mark_as_advanced(TINYXML2_INCLUDE_DIR)

	find_library(TINYXML2_LIBRARY NAMES tinyxml2)
	mark_as_advanced(TINYXML2_LIBRARY)

	if(TINYXML2_INCLUDE_DIR AND TINYXML2_LIBRARY)
		set(CMAKE_MRPT_HAS_TINYXML2 1)
		set(CMAKE_MRPT_HAS_TINYXML2_SYSTEM 1)

		add_library(imp_tinyxml2 INTERFACE IMPORTED)
		set_target_properties(imp_tinyxml2
			PROPERTIES
			INTERFACE_INCLUDE_DIRECTORIES "${TINYXML2_INCLUDE_DIR}"
			INTERFACE_LINK_LIBRARIES "${TINYXML2_LIBRARY}"
			)
	endif()
endif()

if(${CMAKE_MRPT_HAS_TINYXML2} AND "$ENV{VERBOSE}")
	message(STATUS "libtinyxml2 configuration:")
	message(STATUS "  TINYXML2_INCLUDE_DIR: ${TINYXML2_INCLUDE_DIR}")
	message(STATUS "  TINYXML2_LIBRARY: ${TINYXML2_LIBRARY}")
endif()
