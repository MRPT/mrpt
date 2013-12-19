# Check for the OpenNI2 library:
#  pkg-config if available (Linux), otherwise via 
# Originally based on: https://github.com/rgbdemo/nestk/blob/master/cmake/FindOpenNI2.cmake
# =========================================================
SET(CMAKE_MRPT_HAS_OPENNI2 0)

OPTION(DISABLE_OPENNI2 "Disable the OpenNI2 library, even if automatically found" "OFF")
MARK_AS_ADVANCED(DISABLE_OPENNI2)

IF (NOT DISABLE_OPENNI2) # Allow the user to force not using this lib

IF (PKG_CONFIG_FOUND)
	if(${CMAKE_VERSION} VERSION_LESS 2.8.2)
		pkg_check_modules(PC_OPENNI openni2-dev)
	else(${CMAKE_VERSION} VERSION_LESS 2.8.2)
		pkg_check_modules(PC_OPENNI QUIET openni2-dev)
	endif(${CMAKE_VERSION} VERSION_LESS 2.8.2)
ENDIF (PKG_CONFIG_FOUND)

OPTION(MRPT_HAS_OPENNI2 "Support for the OpenNI2 library" "${PC_OPENNI_FOUND}")

IF(MRPT_HAS_OPENNI2)
	#set(OPENNI2_DEFINITIONS ${PC_OPENNI_CFLAGS_OTHER}) #JL: Remove?

	#add a hint so that it can find it without the pkg-config
	find_path(OPENNI2_INCLUDE_DIR OpenNI.h
			  HINTS ${PC_OPENNI_INCLUDEDIR} ${PC_OPENNI_INCLUDE_DIRS} /usr/include/openni2 /usr/include/ni2
			  PATHS "$ENV{PROGRAMFILES}/OpenNI2/Include" "$ENV{PROGRAMW6432}/OpenNI2/Include"
			  PATH_SUFFIXES openni ni
			  DOC "Path to the include directory containing OpenNI.h,etc.")
	#add a hint so that it can find it without the pkg-config
	find_library(OPENNI2_LIBRARY
				 NAMES OpenNI2
				 HINTS ${PC_OPENNI_LIBDIR} ${PC_OPENNI_LIBRARY_DIRS} /usr/lib
				 PATHS "$ENV{PROGRAMFILES}/OpenNI2/Redist" "$ENV{PROGRAMW6432}/OpenNI2/Redist" "$ENV{PROGRAMW6432}/OpenNI2"
				 PATH_SUFFIXES lib lib64
				 DOC "The OpenNI2.lib file path")

	set(OPENNI2_INCLUDE_DIRS ${OPENNI2_INCLUDE_DIR})
	set(OPENNI2_LIBRARIES ${OPENNI2_LIBRARY})

	IF (OPENNI2_INCLUDE_DIRS AND OPENNI2_LIBRARIES)
		SET(CMAKE_MRPT_HAS_OPENNI2  1)
		SET(CMAKE_MRPT_HAS_OPENNI2_SYSTEM  1)
		include_directories(${OPENNI2_INCLUDE_DIRS})
		APPEND_MRPT_LIBS(${OPENNI2_LIBRARIES})

		# Add include directories as "-isystem" to avoid warnings :
		ADD_DIRECTORIES_AS_ISYSTEM(OPENNI2_INCLUDE_DIRS)
		
		message(STATUS "OpenNI2 found (include: ${OPENNI2_INCLUDE_DIR}, lib: ${OPENNI2_LIBRARY})")
	ELSE (OPENNI2_INCLUDE_DIRS AND OPENNI2_LIBRARIES)
		message(FATAL_ERROR "OpenNI2 not found: Either correctly set OPENNI2_INCLUDE_DIR and OPENNI2_LIBRARY or uncheck MRPT_HAS_OPENNI2")
	ENDIF (OPENNI2_INCLUDE_DIRS AND OPENNI2_LIBRARIES)
ENDIF(MRPT_HAS_OPENNI2)

ENDIF (NOT DISABLE_OPENNI2)