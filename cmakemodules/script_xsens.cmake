# Build the xSENS support in mrpt-hwdrivers?
# ===================================================
SET(BUILD_XSENS_MT3 ON CACHE BOOL "Build xSens 3rd generation libraries (interface old xSens MTi devices)")

# Default build MT4 only if we have libusb-1.0 & libudev
SET(CMAKE_MRPT_HAS_LIBUDEV 0)  # Declare these vars system-wide just in case other future classes depend on this lib
SET(CMAKE_MRPT_HAS_LIBUDEV_SYSTEM 0)

IF (WIN32)
	SET(DEFAULT_BUILD_MT4 "OFF")
	IF (HAVE_WINUSB_H)
		SET(DEFAULT_BUILD_MT4 "ON")
	ENDIF (HAVE_WINUSB_H)	
ELSE(WIN32)
	SET(DEFAULT_BUILD_MT4 "OFF")
	IF (PKG_CONFIG_FOUND)
		PKG_CHECK_MODULES(PKG_LIBUSB10 ${_QUIET} libusb-1.0)
		PKG_CHECK_MODULES(PKG_LIBUDEV  ${_QUIET} libudev)
		IF(PKG_LIBUSB10_FOUND AND PKG_LIBUDEV_FOUND)
			SET(DEFAULT_BUILD_MT4 "ON")
		ENDIF(PKG_LIBUSB10_FOUND AND PKG_LIBUDEV_FOUND)

		IF (PKG_LIBUDEV_FOUND)
			SET(CMAKE_MRPT_HAS_LIBUDEV 1)
			SET(CMAKE_MRPT_HAS_LIBUDEV_SYSTEM 1)
		ENDIF (PKG_LIBUDEV_FOUND)
	ENDIF (PKG_CONFIG_FOUND)
ENDIF(WIN32)
SET(BUILD_XSENS_MT4 "${DEFAULT_BUILD_MT4}" CACHE BOOL "Build xSens 4th generation libraries (interface 4th generation xSens MT* devices)")

# Check user doesn't enable it without prerequisites:
IF ("${DEFAULT_BUILD_MT4}" STREQUAL "OFF" AND BUILD_XSENS_MT4)
	# Force disable:
	SET(BUILD_XSENS_MT4 OFF CACHE BOOL "Build xSens 4th generation libraries (interface 4th generation xSens MT* devices)" FORCE)
	# Warning msg:
	MESSAGE(STATUS "*Warning*: Disabling XSens MT4 due to lack of required libs (libusb1.0 & libudev)")
ENDIF ("${DEFAULT_BUILD_MT4}" STREQUAL "OFF" AND BUILD_XSENS_MT4)

# Create config vars for MT3:
SET(CMAKE_MRPT_HAS_xSENS_MT3 0)
SET(CMAKE_MRPT_HAS_xSENS_MT3_SYSTEM 0)
IF(BUILD_XSENS_MT3)
	SET(CMAKE_MRPT_HAS_xSENS_MT3 1)
	SET(CMAKE_MRPT_HAS_xSENS_MT3_SYSTEM 0)
ENDIF(BUILD_XSENS_MT3)

# Additional checks for MT4:
IF (BUILD_XSENS_MT4)
	IF (WIN32)
		# In Windows: Library WinUsb
		# It's supposed to come by default with Windows XP SP2 and newer, but some have reported problems, so:
		IF (NOT HAVE_WINUSB_H)
			MESSAGE(SEND_ERROR "BUILD_XSENS_MT4 requires <winusb.h>. Fix the missing header, or disable BUILD_XSENS_MT4")
		ENDIF (NOT HAVE_WINUSB_H)	
	ELSE(WIN32)
		# In Linux: libusb-1.0
		IF(PKG_LIBUSB10_FOUND)
			# Perfect, we have libusb-1.0
			SET(XSENS4_LIBS ${XSENS4_LIBS} ${PKG_LIBUDEV_LIBRARIES}) #APPEND_MRPT_LIBS(${PKG_LIBUSB10_LIBRARIES})
		ELSE(PKG_LIBUSB10_FOUND)
			MESSAGE(SEND_ERROR "BUILD_XSENS_MT4 requires libusb-1.0. Install it or disable BUILD_XSENS_MT4")
		ENDIF(PKG_LIBUSB10_FOUND)

		# In Linux: libdev
		IF (PKG_LIBUDEV_FOUND)
			SET(XSENS4_LIBS ${XSENS4_LIBS} ${PKG_LIBUDEV_LIBRARIES}) # APPEND_MRPT_LIBS(${PKG_LIBUDEV_LIBRARIES})
		ENDIF (PKG_LIBUDEV_FOUND)
	ENDIF(WIN32)
ENDIF (BUILD_XSENS_MT4)

# Create config vars for MT4:
SET(CMAKE_MRPT_HAS_xSENS_MT4 0)
SET(CMAKE_MRPT_HAS_xSENS_MT4_SYSTEM 0)
IF(BUILD_XSENS_MT4)
	SET(CMAKE_MRPT_HAS_xSENS_MT4 1)
	SET(CMAKE_MRPT_HAS_xSENS_MT4_SYSTEM 0)
ENDIF(BUILD_XSENS_MT4)
