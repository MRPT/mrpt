# Look for the libusb1 lib
# ------------------------------
IF(UNIX OR APPLE)
	IF(PKG_CONFIG_FOUND)
        	PKG_CHECK_MODULES(PKG_LIBUSB10 libusb-1.0)
	ENDIF(PKG_CONFIG_FOUND)
ENDIF(UNIX OR APPLE)


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
SET(CMAKE_MRPT_HAS_CL_NUI 0)
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
			IF(PKG_LIBUSB10_FOUND)
				SET(CMAKE_MRPT_HAS_KINECT 1)
				SET(CMAKE_MRPT_HAS_FREENECT 1)
				#MESSAGE(STATUS "PKG_LIBUSB10_LIBRARIES: ${PKG_LIBUSB10_LIBRARIES}")
				APPEND_MRPT_LIBS(${PKG_LIBUSB10_LIBRARIES})
			ELSE(PKG_LIBUSB10_FOUND)
				MESSAGE(SEND_ERROR "BUILD_KINECT requires libusb-1.0. Install it or disable BUILD_KINECT")
			ENDIF(PKG_LIBUSB10_FOUND)
		ENDIF(NOT PKG_CONFIG_FOUND)
	ELSEIF(WIN32)
		# Kinect for Win32: CL NUI or external libfreenect built by the user ---------

		SET(BUILD_KINECT_USE_CLNUI    OFF CACHE BOOL "Kinect using CL NUI SDK")
		SET(BUILD_KINECT_USE_FREENECT ON  CACHE BOOL "Kinect using OpenKinect's libfreenect")

		IF((NOT BUILD_KINECT_USE_FREENECT AND NOT BUILD_KINECT_USE_CLNUI) OR (BUILD_KINECT_USE_FREENECT AND BUILD_KINECT_USE_CLNUI))
			MESSAGE(SEND_ERROR
				"You must select the Kinect preferred SDK: OpenKinect freenect or CL NUI\n"
				"Mark just one from BUILD_KINECT_USE_CLNUI or BUILD_KINECT_USE_CLNUI, or disable BUILD_KINECT.\n"
				"For more info read: http://www.mrpt.org/Kinect_and_MRPT\n")
		ENDIF((NOT BUILD_KINECT_USE_FREENECT AND NOT BUILD_KINECT_USE_CLNUI) OR (BUILD_KINECT_USE_FREENECT AND BUILD_KINECT_USE_CLNUI))

		# With external Libfreenect:
		IF (BUILD_KINECT_USE_FREENECT)
			# Find packages needed to build library in Windows
			find_package(libusb-1.0 REQUIRED)

			if (LIBUSB_1_FOUND)
				SET(FREENECT_LIBS ${LIBUSB_1_LIBRARIES})

				SET(LIBUSB_1_DLL_FOR_INSTALL "" CACHE PATH "Path to the libusb0.dll file to install along MRPT packages")
				MARK_AS_ADVANCED(LIBUSB_1_DLL_FOR_INSTALL)

				# All OK:
				SET(CMAKE_MRPT_HAS_KINECT 1)
				SET(CMAKE_MRPT_HAS_FREENECT 1)
				SET(CMAKE_MRPT_HAS_FREENECT_SYSTEM 0)  # use embedded version
			else (LIBUSB_1_FOUND)
				# Error:
				MESSAGE(SEND_ERROR "*** ERROR *** Please, set libusb-1 variables or disable BUILD_KINECT.")
			endif (LIBUSB_1_FOUND)

		ENDIF (BUILD_KINECT_USE_FREENECT)

		# With the CL NUI SDK:
		IF (BUILD_KINECT_USE_CLNUI)
			FIND_PATH(BASE_CLNUI_SDK_DIR  "Code Laboratories/CL NUI Platform/SDK/")
			MARK_AS_ADVANCED(BASE_CLNUI_SDK_DIR)

			IF(BASE_CLNUI_SDK_DIR)
				SET(CLNUI_SDK_DIR "${BASE_CLNUI_SDK_DIR}/Code Laboratories/CL NUI Platform/SDK/" CACHE PATH "Path to CL NUI Platform/SDK/ directory")
				MESSAGE(STATUS "Kinect CL NUI SDK found in: ${CLNUI_SDK_DIR}")
			ELSE(BASE_CLNUI_SDK_DIR)
				SET(CLNUI_SDK_DIR "" CACHE PATH "Path to CL NUI Platform/SDK/ directory")
			ENDIF(BASE_CLNUI_SDK_DIR)

			IF (EXISTS "${CLNUI_SDK_DIR}/Include/CLNUIDevice.h")
				SET(CMAKE_MRPT_HAS_KINECT 1)
				SET(CMAKE_MRPT_HAS_CL_NUI 1)

				INCLUDE_DIRECTORIES("${CLNUI_SDK_DIR}/Include")
				APPEND_MRPT_LIBS("${CLNUI_SDK_DIR}/Lib/CLNUIDevice.lib")
			ELSE (EXISTS "${CLNUI_SDK_DIR}/Include/CLNUIDevice.h")
				MESSAGE(SEND_ERROR "*** ERROR *** CL NUI Platform needed for Kinect. Please, install it and set CLNUI_SDK_DIR or disable BUILD_KINECT")
			ENDIF (EXISTS "${CLNUI_SDK_DIR}/Include/CLNUIDevice.h")
		ENDIF (BUILD_KINECT_USE_CLNUI)

	ELSE(UNIX)
		MESSAGE(SEND_ERROR "Sorry! Kinect is supported on Unix or Win32 only. Please, disable BUILD_KINECT.")
	ENDIF(UNIX)

ELSE(BUILD_KINECT)
	SET(CMAKE_MRPT_HAS_KINECT 0)
ENDIF(BUILD_KINECT)
