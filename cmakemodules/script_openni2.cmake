# Check for the OpenNI2 library:
#  pkg-config if available (Linux), otherwise via 
# Originally based on: https://github.com/rgbdemo/nestk/blob/master/cmake/FindOpenNI2.cmake
# =========================================================
set(CMAKE_MRPT_HAS_OPENNI2 0)

option(DISABLE_OPENNI2 "Disable the OpenNI2 library, even if automatically found" "OFF")
mark_as_advanced(DISABLE_OPENNI2)

if (NOT DISABLE_OPENNI2) # Allow the user to force not using this lib

if (PKG_CONFIG_FOUND)
	pkg_check_modules(PC_OPENNI QUIET libopenni2)
endif (PKG_CONFIG_FOUND)

# Build the expected names of the environment variables (Windows only) where OpenNI2 can be found:
if (CMAKE_MRPT_WORD_SIZE EQUAL 64)
	set(ENV_OPNI2_INCLUDE "OPENNI2_INCLUDE64")
	set(ENV_OPNI2_LIB     "OPENNI2_LIB64")
	set(ENV_OPNI2_REDIST  "OPENNI2_REDIST64")
else (CMAKE_MRPT_WORD_SIZE EQUAL 64)
	set(ENV_OPNI2_INCLUDE "OPENNI2_INCLUDE")
	set(ENV_OPNI2_LIB     "OPENNI2_LIB")
	set(ENV_OPNI2_REDIST  "OPENNI2_REDIST")
endif (CMAKE_MRPT_WORD_SIZE EQUAL 64)

# Create option for OpenNI2 and guess its default value: 
set(DEFAULT_MRPT_HAS_OPENNI2 0)
if(PC_OPENNI_FOUND OR NOT "$ENV{${ENV_OPNI2_INCLUDE}}" STREQUAL "")
	set(DEFAULT_MRPT_HAS_OPENNI2 1)
endif(PC_OPENNI_FOUND OR NOT "$ENV{${ENV_OPNI2_INCLUDE}}" STREQUAL "")

option(MRPT_HAS_OPENNI2 "Support for the OpenNI2 library" ${DEFAULT_MRPT_HAS_OPENNI2})

if(MRPT_HAS_OPENNI2)
	#set(OPENNI2_DEFINITIONS ${PC_OPENNI_CFLAGS_OTHER}) #JL: Remove?

	#add a hint so that it can find it without the pkg-config
	find_path(OPENNI2_INCLUDE_DIR OpenNI.h
			  HINTS ${PC_OPENNI_INCLUDEDIR} ${PC_OPENNI_INCLUDE_DIRS} /usr/include/openni2 /usr/include/ni2
			  PATHS "$ENV{PROGRAMFILES}/OpenNI2/Include" "$ENV{PROGRAMW6432}/OpenNI2/Include" "$ENV{${ENV_OPNI2_INCLUDE}}"
			  PATH_SUFFIXES openni ni
			  DOC "Path to the include directory containing OpenNI.h,etc.")
	#add a hint so that it can find it without the pkg-config
	find_library(OPENNI2_LIBRARY
				 NAMES OpenNI2
				 HINTS ${PC_OPENNI_LIBDIR} ${PC_OPENNI_LIBRARY_DIRS} /usr/lib
				 PATHS "$ENV{PROGRAMFILES}/OpenNI2/Redist" "$ENV{PROGRAMW6432}/OpenNI2/Redist" "$ENV{PROGRAMW6432}/OpenNI2" "$ENV{${ENV_OPNI2_LIB}}"
				 PATH_SUFFIXES lib lib64
				 DOC "The OpenNI2.lib file path")

	set(OPENNI2_INCLUDE_DIRS ${OPENNI2_INCLUDE_DIR})
	set(OPENNI2_LIBRARIES ${OPENNI2_LIBRARY})

	if (OPENNI2_INCLUDE_DIRS AND OPENNI2_LIBRARIES)
		set(CMAKE_MRPT_HAS_OPENNI2  1)
		set(CMAKE_MRPT_HAS_OPENNI2_SYSTEM  1)
		include_directories(${OPENNI2_INCLUDE_DIRS})
		#APPEND_MRPT_LIBS(${OPENNI2_LIBRARIES})

		# Add include directories as "-isystem" to avoid warnings :
		ADD_DIRECTORIES_AS_ISYSTEM(OPENNI2_INCLUDE_DIRS)
		
		if ($ENV{VERBOSE})
			message(STATUS "OpenNI2 found (include: ${OPENNI2_INCLUDE_DIR}, lib: ${OPENNI2_LIBRARY})")
		endif ($ENV{VERBOSE})
	else (OPENNI2_INCLUDE_DIRS AND OPENNI2_LIBRARIES)
		message(FATAL_ERROR "OpenNI2 not found: Either correctly set OPENNI2_INCLUDE_DIR and OPENNI2_LIBRARY or uncheck MRPT_HAS_OPENNI2")
	endif (OPENNI2_INCLUDE_DIRS AND OPENNI2_LIBRARIES)
endif(MRPT_HAS_OPENNI2)

endif (NOT DISABLE_OPENNI2)
