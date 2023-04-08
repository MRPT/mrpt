# Look for the libusb1 lib
# ------------------------------
option(DISABLE_LIBUSB "Force to not detect the libusb library" OFF)
mark_as_advanced(DISABLE_LIBUSB)

if (NOT DISABLE_LIBUSB)
	if(UNIX)
		if(PKG_CONFIG_FOUND)
			PKG_CHECK_MODULES(PKG_LIBUSB10 QUIET libusb-1.0 IMPORTED_TARGET GLOBAL) # PkgConfig::PKG_LIBUSB10
		endif()
	endif(UNIX)
else ()
	set(PKG_LIBUSB10_FOUND "")
endif ()

# Look for a system version of libfreenect
# -----------------------------------------
option(DISABLE_DETECT_LIBFREENECT "Force to not detect system libfreenect library" OFF)
mark_as_advanced(DISABLE_DETECT_LIBFREENECT)

if (NOT DISABLE_DETECT_LIBFREENECT)
	if(UNIX)
		if(PKG_CONFIG_FOUND)
			PKG_CHECK_MODULES(PKG_LIBFREENECT QUIET libfreenect IMPORTED_TARGET GLOBAL)  # PkgConfig::PKG_LIBFREENECT
		endif()
	endif()
else ()
	set(PKG_LIBFREENECT_FOUND "")
endif ()


# Build the XBox Kinect support (via libfreenect)
# ===================================================
# Deduce default ON/OFF state of Kinect support:
if (UNIX)
	if (PKG_LIBUSB10_FOUND)
		set(MRPT_WITH_KINECT_DEFAULT ON)
	else()
		set(MRPT_WITH_KINECT_DEFAULT OFF)
	endif()
else()
	set(MRPT_WITH_KINECT_DEFAULT OFF)
endif()

set(CMAKE_MRPT_HAS_KINECT 0)  # Will be set to 1 only if all conditions are OK
set(CMAKE_MRPT_HAS_FREENECT 0)
set(CMAKE_MRPT_HAS_FREENECT_SYSTEM 0) # This means libfreenect is already built somewhere else in the system.

set(MRPT_WITH_KINECT ${MRPT_WITH_KINECT_DEFAULT} CACHE BOOL "Build support for Xbox Kinect")
if(MRPT_WITH_KINECT)
	# Check deps on Linux/Windows!!!

	if(UNIX)
		# Kinect for Linux: Embedded sources of libfreenect ---------
		if(NOT PKG_CONFIG_FOUND)
			message(SEND_ERROR "Kinect support: pkg-config is required! Please install it.")
		else()
			# If a system version is found, use it:
			if(PKG_LIBFREENECT_FOUND)
					set(FREENECT_LIBS PkgConfig::PKG_LIBFREENECT)
					if($ENV{VERBOSE})
						message(STATUS "- PKG_LIBFREENECT_LIBRARIES: PkgConfig::PKG_LIBFREENECT")
					endif()

				set(CMAKE_MRPT_HAS_KINECT 1)
				set(CMAKE_MRPT_HAS_FREENECT 1)
				set(CMAKE_MRPT_HAS_FREENECT_SYSTEM 1)
			else()
				if(PKG_LIBUSB10_FOUND)
					set(CMAKE_MRPT_HAS_KINECT 1)
					set(CMAKE_MRPT_HAS_FREENECT 1)

					set(LIBUSB10_LIBS PkgConfig::PKG_LIBUSB10)
					if($ENV{VERBOSE})
						message(STATUS "- PKG_LIBUSB10_LIBRARIES: PkgConfig::PKG_LIBUSB10")
					endif()
				else()
					message(SEND_ERROR "MRPT_WITH_KINECT requires libusb-1.0. Install it or disable MRPT_WITH_KINECT")
				endif()
			endif()
		endif()
	elseif()
		# Kinect for Win32: libfreenect ---------
		set(MRPT_WITH_KINECT_USE_FREENECT ON  CACHE BOOL "Kinect using OpenKinect's libfreenect")

		if (MRPT_WITH_KINECT_USE_FREENECT)
			# Find packages needed to build library in Windows
			PKG_CHECK_MODULES(PKG_LIBUSB10 QUIET libusb-1.0 IMPORTED_TARGET GLOBAL) # PkgConfig::PKG_LIBUSB10

			if (LIBUSB_1_FOUND)
				set(FREENECT_LIBS PkgConfig::PKG_LIBUSB10)

				# All OK:
				set(CMAKE_MRPT_HAS_KINECT 1)
				set(CMAKE_MRPT_HAS_FREENECT 1)
				set(CMAKE_MRPT_HAS_FREENECT_SYSTEM 0)  # use embedded version
			else ()
				# Error:
				message(SEND_ERROR "*** ERROR *** Please, set libusb-1 variables or disable MRPT_WITH_KINECT.")
			endif()
		endif ()
	else()
		message(SEND_ERROR "Sorry! Kinect is supported on Unix or Win32 only. Please, disable MRPT_WITH_KINECT.")
	endif()

else()
	set(CMAKE_MRPT_HAS_KINECT 0)
endif()
