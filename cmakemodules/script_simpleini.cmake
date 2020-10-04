# Check for system libsimpleini
# ===================================================
set(CMAKE_MRPT_HAS_SIMPLEINI 0)
set(CMAKE_MRPT_HAS_SIMPLEINI_SYSTEM 0)

# 1st) Try to locate the pkg via pkg-config:
find_package(PkgConfig QUIET)
if(PKG_CONFIG_FOUND)
	PKG_CHECK_MODULES(SIMPLEINI QUIET simpleini)
	if (SIMPLEINI_FOUND)
		if ($ENV{VERBOSE})
			message(STATUS "SIMPLEINI: Found via pkg-config")
			message(STATUS " SIMPLEINI_INCLUDE_DIRS=${SIMPLEINI_INCLUDE_DIRS}")
			message(STATUS " SIMPLEINI_CFLAGS=${SIMPLEINI_CFLAGS}")
		endif()

		set(CMAKE_MRPT_HAS_SIMPLEINI 1)
		set(CMAKE_MRPT_HAS_SIMPLEINI_SYSTEM 1)
	endif()
endif()

if (NOT SIMPLEINI_FOUND)
		# Set to use embedded copy:
	set(SIMPLEINI_INCLUDE_DIRS ${MRPT_SOURCE_DIR}/3rdparty/simpleini)

	set(CMAKE_MRPT_HAS_SIMPLEINI 1)
	set(CMAKE_MRPT_HAS_SIMPLEINI_SYSTEM 0)
endif()
