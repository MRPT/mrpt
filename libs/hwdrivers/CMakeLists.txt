# Lists of directories with source files:
#  See "DeclareMRPTLib.cmake" for explanations
# -------------------------------------------------

# Build xSens MT3?
IF(CMAKE_MRPT_HAS_xSENS_MT3)
	SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -pthread")

	LIST(APPEND hwdrivers_EXTRA_SRCS
		"${MRPT_LIBS_ROOT}/hwdrivers/src/xSens_MT3/*.cpp"
		"${MRPT_LIBS_ROOT}/hwdrivers/src/xSens_MT3/*.h"
		)
	LIST(APPEND hwdrivers_EXTRA_SRCS_NAME "xSens_MT3" "xSens_MT3" )
ENDIF(CMAKE_MRPT_HAS_xSENS_MT3)

# Build xSens MT4?
IF(CMAKE_MRPT_HAS_xSENS_MT4)
	LIST(APPEND hwdrivers_EXTRA_SRCS
		"${MRPT_LIBS_ROOT}/hwdrivers/src/xSens_MT4/xcommunication/src/*.cpp"
		"${MRPT_LIBS_ROOT}/hwdrivers/src/xSens_MT4/xcommunication/src/*.c"
		"${MRPT_LIBS_ROOT}/hwdrivers/src/xSens_MT4/xcommunication/include/xsens/*.h"
		"${MRPT_LIBS_ROOT}/hwdrivers/src/xSens_MT4/xstypes/src/*.c"
		"${MRPT_LIBS_ROOT}/hwdrivers/src/xSens_MT4/xstypes/include/xsens/*.h"
		)
	LIST(APPEND hwdrivers_EXTRA_SRCS_NAME 
		"xSens_MT4" "xSens_MT4" "xSens_MT4" 
		"xSens_MT4" "xSens_MT4"
		)
	# XSens MT4 sources seem to expect to have both [DIR]/ and [DIR]/xsens/ as INCLUDE_DIRS:
	INCLUDE_DIRECTORIES(
		"${MRPT_LIBS_ROOT}/hwdrivers/src/xSens_MT4/xcommunication/include/"
		"${MRPT_LIBS_ROOT}/hwdrivers/src/xSens_MT4/xcommunication/include/xsens/"
		"${MRPT_LIBS_ROOT}/hwdrivers/src/xSens_MT4/xstypes/include/"
		"${MRPT_LIBS_ROOT}/hwdrivers/src/xSens_MT4/xstypes/include/xsens/"
		)
ENDIF(CMAKE_MRPT_HAS_xSENS_MT4)

# Build Intersense devices?
IF(CMAKE_MRPT_HAS_INTERSENSE)
   LIST(APPEND hwdrivers_EXTRA_SRCS
      "${MRPT_LIBS_ROOT}/hwdrivers/src/isense/*.c"
      "${MRPT_LIBS_ROOT}/hwdrivers/src/isense/*.h"
      )

   LIST(APPEND hwdrivers_EXTRA_SRCS_NAME "isense" "isense" )
ENDIF(CMAKE_MRPT_HAS_INTERSENSE)

# Build Kinect support via libfreenect?
IF(CMAKE_MRPT_HAS_FREENECT)
	IF (CMAKE_MRPT_HAS_FREENECT_SYSTEM)
		# No need to build if it's already built in the system:
		IF (NOT "${OPENKINECT_FREENECT_INCLUDE_DIR}" STREQUAL "")
			INCLUDE_DIRECTORIES("${OPENKINECT_FREENECT_INCLUDE_DIR}")
		ENDIF (NOT "${OPENKINECT_FREENECT_INCLUDE_DIR}" STREQUAL "")
	ELSE (CMAKE_MRPT_HAS_FREENECT_SYSTEM)
		# Embedded version:
		INCLUDE_DIRECTORIES("${MRPT_LIBS_ROOT}/hwdrivers/src/libfreenect/")
		LIST(APPEND hwdrivers_EXTRA_SRCS
			"${MRPT_LIBS_ROOT}/hwdrivers/src/libfreenect/*.c"
			"${MRPT_LIBS_ROOT}/hwdrivers/src/libfreenect/*.h"
			)
		LIST(APPEND hwdrivers_EXTRA_SRCS_NAME 
			"libfreenect"
			"libfreenect"
			)

		# Special stuff for windows:
		if (WIN32)
			LIST(APPEND hwdrivers_EXTRA_SRCS
				"${MRPT_LIBS_ROOT}/hwdrivers/src/libfreenect/platform/windows/libusb10emu/libusb-1.0/libusbemu.cpp"
				"${MRPT_LIBS_ROOT}/hwdrivers/src/libfreenect/platform/windows/libusb10emu/libusb-1.0/failguard.cpp"
				)
			LIST(APPEND hwdrivers_EXTRA_SRCS_NAME 
				"libfreenect"
				"libfreenect"
				)

			INCLUDE_DIRECTORIES("${MRPT_LIBS_ROOT}/hwdrivers/src/libfreenect/platform/windows")
			INCLUDE_DIRECTORIES("${MRPT_LIBS_ROOT}/hwdrivers/src/libfreenect/platform/windows/libusb10emu")
			# libfreenect.h includes libusb.h:
			include_directories("${LIBUSB_1_INCLUDE_DIRS}")

		endif(WIN32)
			
	ENDIF (CMAKE_MRPT_HAS_FREENECT_SYSTEM)	
ENDIF(CMAKE_MRPT_HAS_FREENECT)

# Build ARIA?
IF(CMAKE_MRPT_HAS_ARIA)
	LIST(APPEND hwdrivers_EXTRA_SRCS
		"${MRPT_LIBS_ROOT}/hwdrivers/src/aria/src/*.cpp"
		"${MRPT_LIBS_ROOT}/hwdrivers/src/aria/include/*.h"
		)
	LIST(APPEND hwdrivers_EXTRA_SRCS_NAME 
		"aria"
		"aria"
		)
ENDIF(CMAKE_MRPT_HAS_ARIA)

# Build RPLIDAR?
IF(CMAKE_MRPT_HAS_ROBOPEAK_LIDAR)
	LIST(APPEND hwdrivers_EXTRA_SRCS
		"${MRPT_LIBS_ROOT}/hwdrivers/src/rplidar/src/*.cpp"
		"${MRPT_LIBS_ROOT}/hwdrivers/src/rplidar/src/hal/*.cpp"
		"${MRPT_LIBS_ROOT}/hwdrivers/src/rplidar/include/*.h"
		)
	LIST(APPEND hwdrivers_EXTRA_SRCS_NAME 
		"RoboPeakLidar"
		"RoboPeakLidar"
		"RoboPeakLidar"
		)
	IF (WIN32)
		LIST(APPEND hwdrivers_EXTRA_SRCS "${MRPT_LIBS_ROOT}/hwdrivers/src/rplidar/src/arch/win32/*.cpp")
		LIST(APPEND hwdrivers_EXTRA_SRCS_NAME "RoboPeakLidar")
	ELSE()
		LIST(APPEND hwdrivers_EXTRA_SRCS "${MRPT_LIBS_ROOT}/hwdrivers/src/rplidar/src/arch/linux/*.cpp")
		LIST(APPEND hwdrivers_EXTRA_SRCS_NAME "RoboPeakLidar")
	ENDIF()
	INCLUDE_DIRECTORIES("${MRPT_LIBS_ROOT}/hwdrivers/src/rplidar/include/")
	INCLUDE_DIRECTORIES("${MRPT_LIBS_ROOT}/hwdrivers/src/rplidar/src/")
ENDIF(CMAKE_MRPT_HAS_ROBOPEAK_LIDAR)

if(CMAKE_MRPT_HAS_LIBDC1394_2)
	IF ("${LIBDC1394_2_INCLUDE_DIRS}")
		INCLUDE_DIRECTORIES("${LIBDC1394_2_INCLUDE_DIRS}")
	ENDIF ("${LIBDC1394_2_INCLUDE_DIRS}")
	IF ("${LIBDC1394_2_LIBRARY_DIRS}")
		LINK_DIRECTORIES("${LIBDC1394_2_LIBRARY_DIRS}")
	ENDIF ("${LIBDC1394_2_LIBRARY_DIRS}")
endif(CMAKE_MRPT_HAS_LIBDC1394_2)


IF(CMAKE_MRPT_HAS_DUO3D)
	INCLUDE_DIRECTORIES("${DUO3D_INCLUDE_DIR}")
	LINK_DIRECTORIES("${DUO3D_LIB_DIR}")
ENDIF(CMAKE_MRPT_HAS_DUO3D)

IF(CMAKE_MRPT_HAS_OPENNI2)
	include_directories(${OPENNI2_INCLUDE_DIRS})
	ADD_DIRECTORIES_AS_ISYSTEM(OPENNI2_INCLUDE_DIRS)
ENDIF(CMAKE_MRPT_HAS_OPENNI2)

IF(CMAKE_MRPT_HAS_SWISSRANGE)
	INCLUDE_DIRECTORIES("${SWR_LIBMESASR_DIR}")
ENDIF(CMAKE_MRPT_HAS_SWISSRANGE)

IF(CMAKE_MRPT_HAS_OPENCV)
	INCLUDE_DIRECTORIES(${OpenCV_INCLUDE_DIR} ${OpenCV_INCLUDE_DIRS} ${OPENCV_INCLUDE_DIRS} ${OPENCV_INCLUDEDIR})
ENDIF()

IF(CMAKE_MRPT_HAS_LIBPCAP)
	INCLUDE_DIRECTORIES(${PCAP_INCLUDE_DIR})
ENDIF()


# Macro declared in "DeclareMRPTLib.cmake":
define_mrpt_lib(
	# Lib name
	hwdrivers 	
	# Dependencies
	mrpt-base
	mrpt-maps  # used to laser preview, etc.
	mrpt-obs 
	mrpt-gui    # used to camera preview, etc.
	mrpt-opengl  # needed by mrpt-gui
	mrpt-maps # needed by laser preview
	mrpt-vision  # needed to stereo rectification
	)

IF(BUILD_mrpt-hwdrivers) 
	# Ignore precompiled headers in intersense sources:
	IF(CMAKE_MRPT_HAS_INTERSENSE)
		IF(MRPT_ENABLE_PRECOMPILED_HDRS AND MSVC)
			set_source_files_properties(${isense_FILES} PROPERTIES COMPILE_FLAGS "/Y-")
		ENDIF(MRPT_ENABLE_PRECOMPILED_HDRS AND MSVC)
	ENDIF(CMAKE_MRPT_HAS_INTERSENSE)
	
	# Link to NITE2 libraries:
	IF(CMAKE_MRPT_HAS_NITE2)
		# link_directories() was added in the configuration script under /cmakemodules/*_ni.cmake
		INCLUDE_DIRECTORIES(${NITE2_INCLUDE_DIR})
		TARGET_LINK_LIBRARIES(mrpt-hwdrivers ${NITE2_LIB_FILE_RELEASE})
	ENDIF(CMAKE_MRPT_HAS_NITE2)

	# Ignore precompiled headers in xSens sources:
	IF(CMAKE_MRPT_HAS_xSENS_MT3 OR CMAKE_MRPT_HAS_xSENS_MT4)
		IF(MRPT_ENABLE_PRECOMPILED_HDRS AND MSVC)
			set_source_files_properties(${xSens_MT3_FILES} ${xSens_MT4_FILES} PROPERTIES COMPILE_FLAGS "/Y-")
		ENDIF(MRPT_ENABLE_PRECOMPILED_HDRS AND MSVC)

		# Avoid some annoying warnings:
		IF (CMAKE_COMPILER_IS_GNUCXX OR ${CMAKE_CXX_COMPILER_ID} STREQUAL "Clang")
			set_source_files_properties(${xSens_MT3_FILES} ${xSens_MT4_FILES} PROPERTIES COMPILE_FLAGS "-Wno-variadic-macros")
			# -D_... = Needed to have CLOCK_REALTIME,... in gcc
			# -fms-extensions: Unnamed structs in unions
			set_source_files_properties(${xSens_MT4_FILES} PROPERTIES COMPILE_FLAGS "-D_POSIX_C_SOURCE=199309L -fms-extensions") 
		ENDIF ()

		ADD_DEFINITIONS(-D_LOG_TO_STDOUT)
	ENDIF(CMAKE_MRPT_HAS_xSENS_MT3 OR CMAKE_MRPT_HAS_xSENS_MT4)

	IF(CMAKE_MRPT_HAS_OPENCV)
		target_link_libraries(mrpt-hwdrivers ${OpenCV_LIBRARIES})
	ENDIF()

	IF(CMAKE_MRPT_HAS_LIBPCAP)
		target_link_libraries(mrpt-hwdrivers ${PCAP_LIBRARY})

		# Windows: 
		# Set wpcap.dll as DELAYLOAD, so users without that driver can use binaries compiled with wpcap support:
		IF (MSVC)
			set_property(TARGET mrpt-hwdrivers APPEND_STRING PROPERTY LINK_FLAGS " /DELAYLOAD:\"wpcap.dll\"")
		ENDIF (MSVC)
	ENDIF()

	# XSens MT4 required libs:
	IF (BUILD_XSENS_MT4)
		TARGET_LINK_LIBRARIES(mrpt-hwdrivers ${XSENS4_LIBS})
	ENDIF (BUILD_XSENS_MT4)
	
	IF(CMAKE_MRPT_HAS_OPENNI2)
		TARGET_LINK_LIBRARIES(mrpt-hwdrivers ${OPENNI2_LIBRARIES})
	ENDIF(CMAKE_MRPT_HAS_OPENNI2)

	IF(CMAKE_MRPT_HAS_SWISSRANGE)
		TARGET_LINK_LIBRARIES(mrpt-hwdrivers ${MRPT_SWR_LIBS})
	ENDIF(CMAKE_MRPT_HAS_SWISSRANGE)

	IF(CMAKE_MRPT_HAS_FTDI_SYSTEM)
		TARGET_LINK_LIBRARIES(mrpt-hwdrivers ${FTDI_LIBS})
	ENDIF(CMAKE_MRPT_HAS_FTDI_SYSTEM)
	
	IF(CMAKE_MRPT_HAS_DUO3D)
		TARGET_LINK_LIBRARIES(mrpt-hwdrivers "${DUO3D_LIB_DIR}/DUOLib.lib" "${DUO3D_LIB_DIR}/Dense3D.lib")
	ENDIF(CMAKE_MRPT_HAS_DUO3D)

	# Seems to be required to link against prebuilt libavcodec, etc.
	IF (MSVC)
		#set_target_properties()
		set_property(TARGET mrpt-hwdrivers APPEND_STRING PROPERTY LINK_FLAGS " /SAFESEH:NO")
	ENDIF()

	IF(CMAKE_MRPT_HAS_FREENECT)
		# Add linking against these libs:
		IF(NOT "${FREENECT_LIBS}" STREQUAL "")
			TARGET_LINK_LIBRARIES(mrpt-hwdrivers ${FREENECT_LIBS})
			
			# Windows: 
			# Set libusb0.dll as DELAYLOAD, so users without that driver 
			# still can use binaries compiled with libusb support:
			IF (MSVC)
				set_property(TARGET mrpt-hwdrivers APPEND_STRING PROPERTY LINK_FLAGS " /DELAYLOAD:\"libusb0.dll\"")
			ENDIF (MSVC)
		ENDIF(NOT "${FREENECT_LIBS}" STREQUAL "")
		
		if (NOT CMAKE_MRPT_HAS_FREENECT_SYSTEM)
			# Using embedded version:
			# Ignore precompiled headers:
			IF(MRPT_ENABLE_PRECOMPILED_HDRS AND MSVC)
				set_source_files_properties(${libfreenect_FILES} PROPERTIES COMPILE_FLAGS "/Y-")
			ENDIF(MRPT_ENABLE_PRECOMPILED_HDRS AND MSVC)
		
			if (MSVC)
				set_source_files_properties(${libfreenect_FILES} PROPERTIES LANGUAGE CXX) 			# Build as C++
			endif(MSVC)
		endif (NOT CMAKE_MRPT_HAS_FREENECT_SYSTEM)
	ENDIF(CMAKE_MRPT_HAS_FREENECT)

	IF(NOT "${LIBUSB10_LIBS}" STREQUAL "")
		TARGET_LINK_LIBRARIES(mrpt-hwdrivers ${LIBUSB10_LIBS})
	ENDIF(NOT "${LIBUSB10_LIBS}" STREQUAL "")

	# build with link on Phidget21 lib
	IF(CMAKE_MRPT_HAS_PHIDGET)
		INCLUDE_DIRECTORIES(${PHIDGETS_INCLUDE_DIR})
		TARGET_LINK_LIBRARIES(mrpt-hwdrivers ${PHIDGETS_LIBRARIES})
	ENDIF(CMAKE_MRPT_HAS_PHIDGET)

	if(CMAKE_MRPT_HAS_LIBDC1394_2)
		TARGET_LINK_LIBRARIES(mrpt-hwdrivers ${LIBDC1394_2_LIBRARIES})
	endif(CMAKE_MRPT_HAS_LIBDC1394_2)

	# Link to National Instrument libraries:
	IF(MRPT_HAS_NI_DAQmxBASE)
		# link_directories() was added in the configuration script under /cmakemodules/*_ni.cmake
		INCLUDE_DIRECTORIES(${NI_DAQmxBASE_INCLUDE_DIR})
		TARGET_LINK_LIBRARIES(mrpt-hwdrivers ${NI_DAQmxBASE_LIB_FILE})
	ENDIF(MRPT_HAS_NI_DAQmxBASE)
	IF(MRPT_HAS_NI_DAQmx)
		# link_directories() was added in the configuration script under /cmakemodules/*_ni.cmake
		INCLUDE_DIRECTORIES(${NI_DAQmx_INCLUDE_DIR})
		TARGET_LINK_LIBRARIES(mrpt-hwdrivers ${NI_DAQmx_LIB_FILE})
	ENDIF(MRPT_HAS_NI_DAQmx)

	# Link to PGR FlyCapture2 libraries:
	IF(CMAKE_MRPT_HAS_FLYCAPTURE2)
		# link_directories() was added in the configuration script under /cmakemodules/*_ni.cmake
		INCLUDE_DIRECTORIES(${PGR_FLYCAPTURE2_INCLUDE_DIR})
		TARGET_LINK_LIBRARIES(mrpt-hwdrivers ${PGR_FLYCAPTURE2_LIB_FILE_RELEASE})
		# Windows: Set dll as DELAYLOAD, so users without that driver can use precompiled binaries:
		IF(MSVC)
			set_property(TARGET mrpt-hwdrivers APPEND_STRING PROPERTY LINK_FLAGS " /DELAYLOAD:\"FLYCAPTURE2.DLL\"")
		ENDIF()
	ENDIF(CMAKE_MRPT_HAS_FLYCAPTURE2)

	# Link to PGR Triclops libraries:
	IF(CMAKE_MRPT_HAS_TRICLOPS)
		# link_directories() was added in the configuration script under /cmakemodules/*_ni.cmake
		INCLUDE_DIRECTORIES(${PGR_TRICLOPS_INCLUDE_DIR})
		TARGET_LINK_LIBRARIES(mrpt-hwdrivers ${PGR_TRICLOPS_LIB_FILE_RELEASE})
		TARGET_LINK_LIBRARIES(mrpt-hwdrivers ${PGR_FC2BRIDGE_LIB_FILE_RELEASE})
		#TARGET_LINK_LIBRARIES(mrpt-hwdrivers ${PGR_PNMUTILS_LIB_FILE_RELEASE}) # Not used in mrpt
		# Windows: Set dll as DELAYLOAD, so users without that driver can use precompiled binaries:
		IF(MSVC)
			get_filename_component(PGR_TRICLOPS_LIBNAME ${PGR_TRICLOPS_LIB_FILE_RELEASE} NAME_WE)
			get_filename_component(PGR_FC2BRIDGE_LIBNAME ${PGR_FC2BRIDGE_LIB_FILE_RELEASE} NAME_WE)
			set_property(TARGET mrpt-hwdrivers APPEND_STRING PROPERTY LINK_FLAGS " /DELAYLOAD:\"${PGR_TRICLOPS_LIBNAME}.DLL\"")
			set_property(TARGET mrpt-hwdrivers APPEND_STRING PROPERTY LINK_FLAGS " /DELAYLOAD:\"${PGR_FC2BRIDGE_LIBNAME}.DLL\"")
		ENDIF()
	ENDIF(CMAKE_MRPT_HAS_TRICLOPS)

	# Ignore precompiled headers in RPLIDAR sources:
	IF(CMAKE_MRPT_HAS_ROBOPEAK_LIDAR)		
		IF(MRPT_ENABLE_PRECOMPILED_HDRS AND MSVC)
			set_source_files_properties(${RoboPeakLidar_FILES} PROPERTIES COMPILE_FLAGS "/Y-")
		ENDIF(MRPT_ENABLE_PRECOMPILED_HDRS AND MSVC)
	ENDIF(CMAKE_MRPT_HAS_ROBOPEAK_LIDAR)
		
	# Build with link on SVS lib
	IF(CMAKE_MRPT_HAS_SVS)
		TARGET_LINK_LIBRARIES(mrpt-hwdrivers dcap)
		TARGET_LINK_LIBRARIES(mrpt-hwdrivers svs)
		TARGET_LINK_LIBRARIES(mrpt-hwdrivers svscalc)
		TARGET_LINK_LIBRARIES(mrpt-hwdrivers svscap)
	ENDIF(CMAKE_MRPT_HAS_SVS)
	IF(CMAKE_MRPT_HAS_ARIA)
		# Ignore precompiled headers in xSens sources:
		IF(MRPT_ENABLE_PRECOMPILED_HDRS AND MSVC)
			set_source_files_properties(${aria_FILES} PROPERTIES COMPILE_FLAGS "/Y-")
		ENDIF(MRPT_ENABLE_PRECOMPILED_HDRS AND MSVC)
	
		INCLUDE_DIRECTORIES("${MRPT_LIBS_ROOT}/hwdrivers/src/aria/include/")

		ADD_DEFINITIONS(-DARIA_STATIC)
		IF(MSVC)
			ADD_DEFINITIONS(-DWIN32)
			ADD_DEFINITIONS(-D_WINDOWS)

			IF(NOT CMAKE_BUILD_TYPE MATCHES "Debug")
				ADD_DEFINITIONS(-DNDEBUG)
			ENDIF(NOT CMAKE_BUILD_TYPE MATCHES "Debug")

			SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /wd4267")	# Get rid of a type conversion warning in Aria lib for MSVC
		ENDIF(MSVC)

		IF(UNIX)
			TARGET_LINK_LIBRARIES(mrpt-hwdrivers dl)
		ENDIF(UNIX)
	
	ENDIF(CMAKE_MRPT_HAS_ARIA)
	
	# Remove the "--pedantic" flag for the file "CFFMPEG_InputStream.cpp" since headers 
	#  have some errors:
	IF(CMAKE_COMPILER_IS_GNUCXX AND CMAKE_MRPT_HAS_FFMPEG_SYSTEM)
		SET(EXTRA_ISYSTEMS "")
		IF(EXISTS "${LIBAVCODEC_INCLUDEDIR}")
			SET(EXTRA_ISYSTEMS "-isystem ${LIBAVCODEC_INCLUDEDIR} ")
		ENDIF(EXISTS "${LIBAVCODEC_INCLUDEDIR}")

		IF(EXISTS "${LIBAVCODEC_INCLUDEDIR}/ffmpeg")
			SET(EXTRA_ISYSTEMS "-isystem ${LIBAVCODEC_INCLUDEDIR}/ffmpeg ")
		ENDIF(EXISTS "${LIBAVCODEC_INCLUDEDIR}/ffmpeg")

		IF(EXISTS "${LIBAVCODEC_INCLUDEDIR}/libavcodec")
			SET(EXTRA_ISYSTEMS "-isystem ${LIBAVCODEC_INCLUDEDIR}/libavcodec ")
		ENDIF(EXISTS "${LIBAVCODEC_INCLUDEDIR}/libavcodec")

		IF(EXISTS "${LIBAVCODEC_INCLUDEDIR}/libavformat")
			SET(EXTRA_ISYSTEMS "-isystem ${LIBAVCODEC_INCLUDEDIR}/libavformat ")
		ENDIF(EXISTS "${LIBAVCODEC_INCLUDEDIR}/libavformat")

		IF (NOT "${EXTRA_ISYSTEMS}" STREQUAL "")
			set_source_files_properties(
				${MRPT_LIBS_ROOT}/hwdrivers/src/CFFMPEG_InputStream.cpp 
				PROPERTIES COMPILE_FLAGS ${EXTRA_ISYSTEMS}
				)
		ENDIF (NOT "${EXTRA_ISYSTEMS}" STREQUAL "")
	ENDIF(CMAKE_COMPILER_IS_GNUCXX AND CMAKE_MRPT_HAS_FFMPEG_SYSTEM)

	IF(CMAKE_MRPT_HAS_WXWIDGETS)
		target_link_libraries(mrpt-hwdrivers ${wxWidgets_LIBRARIES})
	ENDIF(CMAKE_MRPT_HAS_WXWIDGETS)

ENDIF(BUILD_mrpt-hwdrivers) 


