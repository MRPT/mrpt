# Check for system libsimpleini
# ===================================================
set(CMAKE_MRPT_HAS_SIMPLEINI 0)
set(CMAKE_MRPT_HAS_SIMPLEINI_SYSTEM 0)

# 1st) Try to locate the pkg via pkg-config:
find_path(SIMPLEINI_INCLUDE_DIRS SimpleIni.h)
mark_as_advanced(SIMPLEINI_INCLUDE_DIRS)

if(SIMPLEINI_INCLUDE_DIRS)
		if ($ENV{VERBOSE})
			message(STATUS "SIMPLEINI: Found in the system")
			message(STATUS " SIMPLEINI_INCLUDE_DIRS=${SIMPLEINI_INCLUDE_DIRS}")
		endif()

		set(CMAKE_MRPT_HAS_SIMPLEINI 1)
		set(CMAKE_MRPT_HAS_SIMPLEINI_SYSTEM 0)
endif()

if (NOT SIMPLEINI_FOUND)
		# Set to use embedded copy:
	set(SIMPLEINI_INCLUDE_DIRS ${MRPT_SOURCE_DIR}/3rdparty/simpleini)

	set(CMAKE_MRPT_HAS_SIMPLEINI 1)
	set(CMAKE_MRPT_HAS_SIMPLEINI_SYSTEM 1)
endif()
