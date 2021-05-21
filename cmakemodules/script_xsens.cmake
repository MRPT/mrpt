# Build the xSENS support in mrpt-hwdrivers?
# ===================================================
# Default build MT4 only if we have libusb-1.0 & libudev
set(CMAKE_MRPT_HAS_LIBUDEV 0)  # Declare these vars system-wide just in case other future classes depend on this lib
set(CMAKE_MRPT_HAS_LIBUDEV_SYSTEM 0)

if (WIN32)
	set(DEFAULT_BUILD_MT4 "OFF")
	if (HAVE_WINUSB_H AND NOT MINGW)
		set(DEFAULT_BUILD_MT4 "ON")
	endif (HAVE_WINUSB_H)
else()
	set(DEFAULT_BUILD_MT4 "OFF")
	if (PKG_CONFIG_FOUND)
		PKG_CHECK_MODULES(PKG_LIBUSB10 QUIET libusb-1.0)
		PKG_CHECK_MODULES(PKG_LIBUDEV  QUIET libudev)
		if(PKG_LIBUSB10_FOUND AND PKG_LIBUDEV_FOUND)
			set(DEFAULT_BUILD_MT4 "ON")
		endif(PKG_LIBUSB10_FOUND AND PKG_LIBUDEV_FOUND)

		if (PKG_LIBUDEV_FOUND)
			set(CMAKE_MRPT_HAS_LIBUDEV 1)
			set(CMAKE_MRPT_HAS_LIBUDEV_SYSTEM 1)
		endif (PKG_LIBUDEV_FOUND)
	endif (PKG_CONFIG_FOUND)
endif()
set(BUILD_XSENS "${DEFAULT_BUILD_MT4}" CACHE BOOL "Build xSens 4th generation libraries (interface 4th generation xSens MT* devices)")

# Check user doesn't enable it without prerequisites:
if ("${DEFAULT_BUILD_MT4}" STREQUAL "OFF" AND BUILD_XSENS)
	# Force disable:
	set(BUILD_XSENS OFF CACHE BOOL "Build xSens 4th generation libraries (interface 4th generation xSens MT* devices)" FORCE)
	# Warning msg:
	message(STATUS "*Warning*: Disabling XSens MT4 due to lack of required libs (libusb1.0 & libudev)")
endif ("${DEFAULT_BUILD_MT4}" STREQUAL "OFF" AND BUILD_XSENS)

# checks for MT4:
if (BUILD_XSENS)
	if (WIN32)
		# In Windows: Library WinUsb
		# It's supposed to come by default with Windows XP SP2 and newer, but some have reported problems, so:
		if (NOT HAVE_WINUSB_H)
			message(SEND_ERROR "BUILD_XSENS requires <winusb.h>. Fix the missing header, or disable BUILD_XSENS")
		endif ()
	else()
		# In Linux: libusb-1.0
		if(PKG_LIBUSB10_FOUND)
			# Perfect, we have libusb-1.0
			set(XSENS4_LIBS ${XSENS4_LIBS} ${PKG_LIBUDEV_LIBRARIES})
		else(PKG_LIBUSB10_FOUND)
			message(SEND_ERROR "BUILD_XSENS requires libusb-1.0. Install it or disable BUILD_XSENS")
		endif(PKG_LIBUSB10_FOUND)

		# In Linux: libdev
		if (PKG_LIBUDEV_FOUND)
			set(XSENS4_LIBS ${XSENS4_LIBS} ${PKG_LIBUDEV_LIBRARIES})
		endif ()
	endif()
endif ()

# Create config vars for xSens:
set(CMAKE_MRPT_HAS_xSENS 0)
set(CMAKE_MRPT_HAS_xSENS_SYSTEM 0)
if(BUILD_XSENS)
	set(CMAKE_MRPT_HAS_xSENS 1)
	set(CMAKE_MRPT_HAS_xSENS_SYSTEM 0)
endif()
