# Look for the libusb1 lib
# ------------------------------
OPTION(DISABLE_LIBUSB "Force to not detect the libusb library" OFF)
mark_as_advanced(DISABLE_LIBUSB)

IF (NOT DISABLE_LIBUSB)
	IF(UNIX)
		IF(PKG_CONFIG_FOUND)
			PKG_CHECK_MODULES(PKG_LIBUSB10 ${_QUIET} libusb-1.0)
		ENDIF(PKG_CONFIG_FOUND)
	ENDIF(UNIX)
ELSE (NOT DISABLE_LIBUSB)
	SET(PKG_LIBUSB10_FOUND "")
ENDIF (NOT DISABLE_LIBUSB)

# Look for a system version of libfreenect
# -----------------------------------------
OPTION(DISABLE_DETECT_LIBFREENECT "Force to not detect system libfreenect library" OFF)
mark_as_advanced(DISABLE_DETECT_LIBFREENECT)

IF (NOT DISABLE_DETECT_LIBFREENECT)
	IF(UNIX)
		IF(PKG_CONFIG_FOUND)
			PKG_CHECK_MODULES(PKG_LIBFREENECT ${_QUIET} libfreenect)
		ENDIF(PKG_CONFIG_FOUND)
	ENDIF(UNIX)
ELSE (NOT DISABLE_DETECT_LIBFREENECT)
	SET(PKG_LIBFREENECT_FOUND "")
ENDIF (NOT DISABLE_DETECT_LIBFREENECT)


# Build the XBox Kinect support (via libfreenect)
# ===================================================
# Deduce default ON/OFF state of Kinect support:
IF (UNIX)
	IF (PKG_LIBUSB10_FOUND)
		SET(BUILD_KINECT_DEFAULT ON)
	ELSE(PKG_LIBUSB10_FOUND)
		SET(BUILD_KINECT_DEFAULT OFF)
	ENDIF(PKG_LIBUSB10_FOUND)
ELSE(UNIX)
	SET(BUILD_KINECT_DEFAULT OFF)
ENDIF(UNIX)

SET(CMAKE_MRPT_HAS_KINECT 0)  # Will be set to 1 only if all conditions are OK
SET(CMAKE_MRPT_HAS_FREENECT 0)
SET(CMAKE_MRPT_HAS_FREENECT_SYSTEM 0) # This means libfreenect is already built somewhere else in the system.

SET(BUILD_KINECT ${BUILD_KINECT_DEFAULT} CACHE BOOL "Build support for Xbox Kinect")
IF(BUILD_KINECT)
	# Check deps on Linux/Windows!!!

	IF(UNIX)
		# Kinect for Linux: Embedded sources of libfreenect ---------
		IF(NOT PKG_CONFIG_FOUND)
			MESSAGE(SEND_ERROR "Kinect support: pkg-config is required! Please install it.")
		ELSE(NOT PKG_CONFIG_FOUND)
			# If a system version is found, use it:
			IF(PKG_LIBFREENECT_FOUND)
					#APPEND_MRPT_LIBS(${PKG_LIBFREENECT_LIBRARIES})
					SET(FREENECT_LIBS ${PKG_LIBFREENECT_LIBRARIES})
					IF($ENV{VERBOSE})					
						MESSAGE(STATUS "- PKG_LIBFREENECT_LIBRARIES: ${PKG_LIBFREENECT_LIBRARIES}")
					ENDIF($ENV{VERBOSE})					

				SET(CMAKE_MRPT_HAS_KINECT 1)
				SET(CMAKE_MRPT_HAS_FREENECT 1)
				SET(CMAKE_MRPT_HAS_FREENECT_SYSTEM 1)
			ELSE(PKG_LIBFREENECT_FOUND)
				IF(PKG_LIBUSB10_FOUND)
					SET(CMAKE_MRPT_HAS_KINECT 1)
					SET(CMAKE_MRPT_HAS_FREENECT 1)

					SET(LIBUSB10_LIBS ${PKG_LIBUSB10_LIBRARIES}) #APPEND_MRPT_LIBS(${PKG_LIBUSB10_LIBRARIES})
					IF($ENV{VERBOSE})					
						MESSAGE(STATUS "- PKG_LIBUSB10_LIBRARIES: ${PKG_LIBUSB10_LIBRARIES}")
					ENDIF($ENV{VERBOSE})					
				ELSE(PKG_LIBUSB10_FOUND)
					MESSAGE(SEND_ERROR "BUILD_KINECT requires libusb-1.0. Install it or disable BUILD_KINECT")
				ENDIF(PKG_LIBUSB10_FOUND)
			ENDIF(PKG_LIBFREENECT_FOUND)

		ENDIF(NOT PKG_CONFIG_FOUND)
	ELSEIF(WIN32)
		# Kinect for Win32: libfreenect ---------
		SET(BUILD_KINECT_USE_FREENECT ON  CACHE BOOL "Kinect using OpenKinect's libfreenect")

		IF (BUILD_KINECT_USE_FREENECT)
			# Find packages needed to build library in Windows
			find_package(libusb-1.0 REQUIRED)

			if (LIBUSB_1_FOUND)
				SET(FREENECT_LIBS ${LIBUSB_1_LIBRARIES})

				# All OK:
				SET(CMAKE_MRPT_HAS_KINECT 1)
				SET(CMAKE_MRPT_HAS_FREENECT 1)
				SET(CMAKE_MRPT_HAS_FREENECT_SYSTEM 0)  # use embedded version
			else (LIBUSB_1_FOUND)
				# Error:
				MESSAGE(SEND_ERROR "*** ERROR *** Please, set libusb-1 variables or disable BUILD_KINECT.")
			endif (LIBUSB_1_FOUND)

		ENDIF (BUILD_KINECT_USE_FREENECT)

	ELSE(UNIX)
		MESSAGE(SEND_ERROR "Sorry! Kinect is supported on Unix or Win32 only. Please, disable BUILD_KINECT.")
	ENDIF(UNIX)

ELSE(BUILD_KINECT)
	SET(CMAKE_MRPT_HAS_KINECT 0)
ENDIF(BUILD_KINECT)
