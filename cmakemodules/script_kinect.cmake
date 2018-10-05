# Look for the libusb1 lib
# ------------------------------
option(DISABLE_LIBUSB "Force to not detect the libusb library" OFF)
mark_as_advanced(DISABLE_LIBUSB)

if (NOT DISABLE_LIBUSB)
	if(UNIX)
		if(PKG_CONFIG_FOUND)
			PKG_CHECK_MODULES(PKG_LIBUSB10 QUIET libusb-1.0)
		endif(PKG_CONFIG_FOUND)
	endif(UNIX)
else (NOT DISABLE_LIBUSB)
	set(PKG_LIBUSB10_FOUND "")
endif (NOT DISABLE_LIBUSB)

# Look for a system version of libfreenect
# -----------------------------------------
option(DISABLE_DETECT_LIBFREENECT "Force to not detect system libfreenect library" OFF)
mark_as_advanced(DISABLE_DETECT_LIBFREENECT)

if (NOT DISABLE_DETECT_LIBFREENECT)
	if(UNIX)
		if(PKG_CONFIG_FOUND)
			PKG_CHECK_MODULES(PKG_LIBFREENECT QUIET libfreenect)
		endif(PKG_CONFIG_FOUND)
	endif(UNIX)
else (NOT DISABLE_DETECT_LIBFREENECT)
	set(PKG_LIBFREENECT_FOUND "")
endif (NOT DISABLE_DETECT_LIBFREENECT)


# Build the XBox Kinect support (via libfreenect)
# ===================================================
# Deduce default ON/OFF state of Kinect support:
if (UNIX)
	if (PKG_LIBUSB10_FOUND)
		set(BUILD_KINECT_DEFAULT ON)
	else(PKG_LIBUSB10_FOUND)
		set(BUILD_KINECT_DEFAULT OFF)
	endif(PKG_LIBUSB10_FOUND)
else(UNIX)
	set(BUILD_KINECT_DEFAULT OFF)
endif(UNIX)

set(CMAKE_MRPT_HAS_KINECT 0)  # Will be set to 1 only if all conditions are OK
set(CMAKE_MRPT_HAS_FREENECT 0)
set(CMAKE_MRPT_HAS_FREENECT_SYSTEM 0) # This means libfreenect is already built somewhere else in the system.

set(BUILD_KINECT ${BUILD_KINECT_DEFAULT} CACHE BOOL "Build support for Xbox Kinect")
if(BUILD_KINECT)
	# Check deps on Linux/Windows!!!

	if(UNIX)
		# Kinect for Linux: Embedded sources of libfreenect ---------
		if(NOT PKG_CONFIG_FOUND)
			message(SEND_ERROR "Kinect support: pkg-config is required! Please install it.")
		else(NOT PKG_CONFIG_FOUND)
			# If a system version is found, use it:
			if(PKG_LIBFREENECT_FOUND)
					#APPEND_MRPT_LIBS(${PKG_LIBFREENECT_LIBRARIES})
					set(FREENECT_LIBS ${PKG_LIBFREENECT_LIBRARIES})
					if($ENV{VERBOSE})					
						message(STATUS "- PKG_LIBFREENECT_LIBRARIES: ${PKG_LIBFREENECT_LIBRARIES}")
					endif($ENV{VERBOSE})					

				set(CMAKE_MRPT_HAS_KINECT 1)
				set(CMAKE_MRPT_HAS_FREENECT 1)
				set(CMAKE_MRPT_HAS_FREENECT_SYSTEM 1)
			else(PKG_LIBFREENECT_FOUND)
				if(PKG_LIBUSB10_FOUND)
					set(CMAKE_MRPT_HAS_KINECT 1)
					set(CMAKE_MRPT_HAS_FREENECT 1)

					set(LIBUSB10_LIBS ${PKG_LIBUSB10_LIBRARIES}) #APPEND_MRPT_LIBS(${PKG_LIBUSB10_LIBRARIES})
					if($ENV{VERBOSE})					
						message(STATUS "- PKG_LIBUSB10_LIBRARIES: ${PKG_LIBUSB10_LIBRARIES}")
					endif($ENV{VERBOSE})					
				else(PKG_LIBUSB10_FOUND)
					message(SEND_ERROR "BUILD_KINECT requires libusb-1.0. Install it or disable BUILD_KINECT")
				endif(PKG_LIBUSB10_FOUND)
			endif(PKG_LIBFREENECT_FOUND)

		endif(NOT PKG_CONFIG_FOUND)
	elseif(WIN32)
		# Kinect for Win32: libfreenect ---------
		set(BUILD_KINECT_USE_FREENECT ON  CACHE BOOL "Kinect using OpenKinect's libfreenect")

		if (BUILD_KINECT_USE_FREENECT)
			# Find packages needed to build library in Windows
			find_package(libusb-1.0 REQUIRED)

			if (LIBUSB_1_FOUND)
				set(FREENECT_LIBS ${LIBUSB_1_LIBRARIES})

				# All OK:
				set(CMAKE_MRPT_HAS_KINECT 1)
				set(CMAKE_MRPT_HAS_FREENECT 1)
				set(CMAKE_MRPT_HAS_FREENECT_SYSTEM 0)  # use embedded version
			else (LIBUSB_1_FOUND)
				# Error:
				message(SEND_ERROR "*** ERROR *** Please, set libusb-1 variables or disable BUILD_KINECT.")
			endif (LIBUSB_1_FOUND)

		endif (BUILD_KINECT_USE_FREENECT)

	else(UNIX)
		message(SEND_ERROR "Sorry! Kinect is supported on Unix or Win32 only. Please, disable BUILD_KINECT.")
	endif(UNIX)

else(BUILD_KINECT)
	set(CMAKE_MRPT_HAS_KINECT 0)
endif(BUILD_KINECT)
