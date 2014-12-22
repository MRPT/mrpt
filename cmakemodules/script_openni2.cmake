# Check for the OpenNI2 library:
#  pkg-config if available (Linux), otherwise via 
# Originally based on: https://github.com/rgbdemo/nestk/blob/master/cmake/FindOpenNI2.cmake
# =========================================================
SET(CMAKE_MRPT_HAS_OPENNI2 0)

OPTION(DISABLE_OPENNI2 "Disable the OpenNI2 library, even if automatically found" "OFF")
MARK_AS_ADVANCED(DISABLE_OPENNI2)

IF (NOT DISABLE_OPENNI2) # Allow the user to force not using this lib

IF (PKG_CONFIG_FOUND)
	pkg_check_modules(PC_OPENNI ${_QUIET} libopenni2)
ENDIF (PKG_CONFIG_FOUND)

# Build the expected names of the environment variables (Windows only) where OpenNI2 can be found:
IF (CMAKE_MRPT_WORD_SIZE EQUAL 64)
	SET(ENV_OPNI2_INCLUDE "OPENNI2_INCLUDE64")
	SET(ENV_OPNI2_LIB     "OPENNI2_LIB64")
	SET(ENV_OPNI2_REDIST  "OPENNI2_REDIST64")
ELSE (CMAKE_MRPT_WORD_SIZE EQUAL 64)
	SET(ENV_OPNI2_INCLUDE "OPENNI2_INCLUDE")
	SET(ENV_OPNI2_LIB     "OPENNI2_LIB")
	SET(ENV_OPNI2_REDIST  "OPENNI2_REDIST")
ENDIF (CMAKE_MRPT_WORD_SIZE EQUAL 64)

# Create option for OpenNI2 and guess its default value: 
SET(DEFAULT_MRPT_HAS_OPENNI2 0)
IF(PC_OPENNI_FOUND OR NOT "$ENV{${ENV_OPNI2_INCLUDE}}" STREQUAL "")
	SET(DEFAULT_MRPT_HAS_OPENNI2 1)
ENDIF(PC_OPENNI_FOUND OR NOT "$ENV{${ENV_OPNI2_INCLUDE}}" STREQUAL "")

OPTION(MRPT_HAS_OPENNI2 "Support for the OpenNI2 library" ${DEFAULT_MRPT_HAS_OPENNI2})

IF(MRPT_HAS_OPENNI2)
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

	IF (OPENNI2_INCLUDE_DIRS AND OPENNI2_LIBRARIES)
		SET(CMAKE_MRPT_HAS_OPENNI2  1)
		SET(CMAKE_MRPT_HAS_OPENNI2_SYSTEM  1)
		include_directories(${OPENNI2_INCLUDE_DIRS})
		#APPEND_MRPT_LIBS(${OPENNI2_LIBRARIES})

		# Add include directories as "-isystem" to avoid warnings :
		ADD_DIRECTORIES_AS_ISYSTEM(OPENNI2_INCLUDE_DIRS)
		
		if ($ENV{VERBOSE})
			message(STATUS "OpenNI2 found (include: ${OPENNI2_INCLUDE_DIR}, lib: ${OPENNI2_LIBRARY})")
		endif ($ENV{VERBOSE})
	ELSE (OPENNI2_INCLUDE_DIRS AND OPENNI2_LIBRARIES)
		message(FATAL_ERROR "OpenNI2 not found: Either correctly set OPENNI2_INCLUDE_DIR and OPENNI2_LIBRARY or uncheck MRPT_HAS_OPENNI2")
	ENDIF (OPENNI2_INCLUDE_DIRS AND OPENNI2_LIBRARIES)
ENDIF(MRPT_HAS_OPENNI2)

ENDIF (NOT DISABLE_OPENNI2)
