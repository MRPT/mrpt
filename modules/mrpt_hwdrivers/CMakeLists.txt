cmake_minimum_required(VERSION 3.16)
project(mrpt_hwdrivers LANGUAGES C CXX)

find_package(mrpt_common REQUIRED)
find_package(mrpt_comms REQUIRED)
find_package(mrpt_maps REQUIRED)
find_package(mrpt_viz REQUIRED)

# Optional dependency detection scripts:
include(${CMAKE_CURRENT_SOURCE_DIR}/cmake/script_ffmpeg.cmake REQUIRED)
include(${CMAKE_CURRENT_SOURCE_DIR}/cmake/script_flycapture2.cmake REQUIRED)
include(${CMAKE_CURRENT_SOURCE_DIR}/cmake/script_kinect.cmake REQUIRED)
include(${CMAKE_CURRENT_SOURCE_DIR}/cmake/script_mynteye.cmake REQUIRED)
include(${CMAKE_CURRENT_SOURCE_DIR}/cmake/script_openni2.cmake REQUIRED)
include(${CMAKE_CURRENT_SOURCE_DIR}/cmake/script_pcap.cmake REQUIRED)
include(${CMAKE_CURRENT_SOURCE_DIR}/cmake/script_phidget.cmake REQUIRED)
include(${CMAKE_CURRENT_SOURCE_DIR}/cmake/script_triclops.cmake REQUIRED)
include(${CMAKE_CURRENT_SOURCE_DIR}/cmake/script_libdc1394.cmake REQUIRED)

# Build Kinect support via libfreenect?
if(CMAKE_MRPT_HAS_FREENECT AND CMAKE_MRPT_HAS_FREENECT_SYSTEM)
	if(NOT "${OPENKINECT_FREENECT_INCLUDE_DIR}" STREQUAL "")
		include_directories("${OPENKINECT_FREENECT_INCLUDE_DIR}")
	endif()
endif()

if(CMAKE_MRPT_HAS_LIBDC1394_2)
	if ("${LIBDC1394_2_INCLUDE_DIRS}")
		include_directories("${LIBDC1394_2_INCLUDE_DIRS}")
	endif ()
	if ("${LIBDC1394_2_LIBRARY_DIRS}")
		link_directories("${LIBDC1394_2_LIBRARY_DIRS}")
	endif ()
endif()

if(CMAKE_MRPT_HAS_LIBPCAP)
	include_directories(${PCAP_INCLUDE_DIR})
endif()

set(LIB_SOURCES
  src/C2DRangeFinderAbstract.cpp
  src/CCANBusReader.cpp
  src/CCameraSensor.cpp
  src/CEnoseModular.cpp
  src/CFFMPEG_InputStream.cpp
  src/CGenericSensor.cpp
  src/CGillAnemometer.cpp
  src/CGPSInterface.cpp
  src/CGPSInterface_parser_NMEA.cpp
  src/CGPSInterface_parser_NOVATEL_OEM6.cpp
  src/CGPS_NTRIP.cpp
  src/CGyroKVHDSP3000.cpp
  src/CHokuyoURG.cpp
  src/CIbeoLuxETH.cpp
  src/CImageGrabber_dc1394.cpp
  src/CImageGrabber_FlyCapture2.cpp
  src/CImageGrabber_OpenCV.cpp
  src/CIMUXSens_MT4.cpp
  src/CImpinjRFID.cpp
  src/CJoystick.cpp
  src/CKinect.cpp
  src/CLMS100eth.cpp
  src/CMyntEyeCamera.cpp
  src/CNationalInstrumentsDAQ.cpp
  src/CNTRIPClient.cpp
  src/CNTRIPEmitter.cpp
  src/COpenNI2Generic.cpp
  src/COpenNI2_RGBD360.cpp
  src/COpenNI2Sensor.cpp
  src/CPhidgetInterfaceKitProximitySensors.cpp
  src/CRaePID.cpp
  src/CRoboPeakLidar.cpp
  src/CServoeNeck.cpp
  src/CSickLaserSerial.cpp
  src/CSickLaserUSB.cpp
  src/CSICKTim561Eth_2050101.cpp
  src/CStereoGrabber_Bumblebee_libdc1394.cpp
  src/CTaoboticsIMU.cpp
  src/CVelodyneScanner.cpp
  src/CWirelessPower.cpp
  src/registerAllClasses.cpp
  # Tests:
  tests/CGPSInterface_unittest.cpp
  tests/CVelodyneScanner_unittest.cpp
)

set(LIB_PUBLIC_HEADERS
  include/mrpt/hwdrivers/C2DRangeFinderAbstract.h
  include/mrpt/hwdrivers/CCANBusReader.h
  include/mrpt/hwdrivers/CCameraSensor.h
  include/mrpt/hwdrivers/CEnoseModular.h
  include/mrpt/hwdrivers/CFFMPEG_InputStream.h
  include/mrpt/hwdrivers/CGenericSensor.h
  include/mrpt/hwdrivers/CGillAnemometer.h
  include/mrpt/hwdrivers/CGPSInterface.h
  include/mrpt/hwdrivers/CGPS_NTRIP.h
  include/mrpt/hwdrivers/CGyroKVHDSP3000.h
  include/mrpt/hwdrivers/CHokuyoURG.h
  include/mrpt/hwdrivers/CIbeoLuxETH.h
  include/mrpt/hwdrivers/CImageGrabber_dc1394.h
  include/mrpt/hwdrivers/CImageGrabber_FlyCapture2.h
  include/mrpt/hwdrivers/CImageGrabber_OpenCV.h
  include/mrpt/hwdrivers/CIMUXSens_MT4.h
  include/mrpt/hwdrivers/CImpinjRFID.h
  include/mrpt/hwdrivers/CJoystick.h
  include/mrpt/hwdrivers/CKinect.h
  include/mrpt/hwdrivers/CLMS100eth.h
  include/mrpt/hwdrivers/CMyntEyeCamera.h
  include/mrpt/hwdrivers/CNationalInstrumentsDAQ.h
  include/mrpt/hwdrivers/CNTRIPClient.h
  include/mrpt/hwdrivers/CNTRIPEmitter.h
  include/mrpt/hwdrivers/COpenNI2Generic.h
  include/mrpt/hwdrivers/COpenNI2Generic_CDevice.h
  include/mrpt/hwdrivers/COpenNI2_RGBD360.h
  include/mrpt/hwdrivers/COpenNI2Sensor.h
  include/mrpt/hwdrivers/CPhidgetInterfaceKitProximitySensors.h
  include/mrpt/hwdrivers/CPtuBase.h
  include/mrpt/hwdrivers/CRaePID.h
  include/mrpt/hwdrivers/CRoboPeakLidar.h
  include/mrpt/hwdrivers/CServoeNeck.h
  include/mrpt/hwdrivers/CSickLaserSerial.h
  include/mrpt/hwdrivers/CSickLaserUSB.h
  include/mrpt/hwdrivers/CSICKTim561Eth_2050101.h
  include/mrpt/hwdrivers/CStereoGrabber_Bumblebee_libdc1394.h
  include/mrpt/hwdrivers/CTaoboticsIMU.h
  include/mrpt/hwdrivers/CVelodyneScanner.h
  include/mrpt/hwdrivers/CWirelessPower.h
  include/mrpt/hwdrivers/registerAllClasses.h
)

mrpt_add_library(
  TARGET ${PROJECT_NAME}
  SOURCES ${LIB_SOURCES} ${LIB_PUBLIC_HEADERS}
  PUBLIC_LINK_LIBRARIES
    mrpt::mrpt_comms
    mrpt::mrpt_maps
    mrpt::mrpt_viz
  CMAKE_DEPENDENCIES
    mrpt_comms
    mrpt_maps
    mrpt_viz
)

if(NOT BUILD_${PROJECT_NAME})
	return()
endif()

# We need pthread's on unices
target_link_libraries(${PROJECT_NAME} PRIVATE Threads::Threads)

if(CMAKE_MRPT_HAS_LIBPCAP)
	target_link_libraries(${PROJECT_NAME} PRIVATE ${PCAP_LIBRARY})
	if (MSVC)
		set_property(TARGET ${PROJECT_NAME} APPEND_STRING PROPERTY LINK_FLAGS " /DELAYLOAD:\"wpcap.dll\"")
	endif ()
endif()

# XSens MT4 required libs:
if (MRPT_WITH_XSENS)
	target_link_libraries(${PROJECT_NAME} PRIVATE ${XSENS4_LIBS})
endif ()

if(UNIX)
    target_link_libraries(${PROJECT_NAME} PRIVATE dl)
endif()

if(CMAKE_MRPT_HAS_OPENNI2)
	target_link_libraries(${PROJECT_NAME} PRIVATE ${MRPT_OPENNI2_LIBRARIES})
endif()

if (MSVC)
	set_property(TARGET ${PROJECT_NAME} APPEND_STRING PROPERTY LINK_FLAGS " /SAFESEH:NO")
	set_target_properties(${PROJECT_NAME} PROPERTIES LINK_FLAGS "/DELAYLOAD:wlanapi.dll")
endif()

if(CMAKE_MRPT_HAS_FREENECT)
	if(NOT "${FREENECT_LIBS}" STREQUAL "")
		target_link_libraries(${PROJECT_NAME} PRIVATE ${FREENECT_LIBS})
		if (MSVC)
			set_property(TARGET ${PROJECT_NAME} APPEND_STRING PROPERTY LINK_FLAGS " /DELAYLOAD:\"libusb0.dll\"")
		endif ()
	endif()
	if(NOT CMAKE_MRPT_HAS_FREENECT_SYSTEM)
		target_link_libraries(${PROJECT_NAME} PRIVATE mrpt_freenect)
	endif()
endif()

# build with link on Phidget21 lib
if(CMAKE_MRPT_HAS_PHIDGET)
	include_directories(${PHIDGETS_INCLUDE_DIR})
	target_link_libraries(${PROJECT_NAME} PRIVATE ${PHIDGETS_LIBRARIES})
endif()

if(CMAKE_MRPT_HAS_LIBDC1394_2)
	target_link_libraries(${PROJECT_NAME} PRIVATE ${LIBDC1394_2_LIBRARIES})
endif()

# Link to National Instrument libraries:
if(MRPT_HAS_NI_DAQmxBASE)
	include_directories(${NI_DAQmxBASE_INCLUDE_DIR})
	target_link_libraries(${PROJECT_NAME} PRIVATE ${NI_DAQmxBASE_LIB_FILE})
endif()
if(MRPT_HAS_NI_DAQmx)
	include_directories(${NI_DAQmx_INCLUDE_DIR})
	target_link_libraries(${PROJECT_NAME} PRIVATE ${NI_DAQmx_LIB_FILE})
endif()

# Link to PGR FlyCapture2 libraries:
if(CMAKE_MRPT_HAS_FLYCAPTURE2)
	include_directories(${PGR_FLYCAPTURE2_INCLUDE_DIR})
	target_link_libraries(${PROJECT_NAME} PRIVATE ${PGR_FLYCAPTURE2_LIB_FILE_RELEASE})
	if(MSVC)
		set_property(TARGET ${PROJECT_NAME} APPEND_STRING PROPERTY LINK_FLAGS " /DELAYLOAD:\"FLYCAPTURE2.DLL\"")
	endif()
endif()

# Link to PGR Triclops libraries:
if(CMAKE_MRPT_HAS_TRICLOPS)
	include_directories(${PGR_TRICLOPS_INCLUDE_DIR})
	target_link_libraries(${PROJECT_NAME} PRIVATE ${PGR_TRICLOPS_LIB_FILE_RELEASE})
	target_link_libraries(${PROJECT_NAME} PRIVATE ${PGR_FC2BRIDGE_LIB_FILE_RELEASE})
	if(MSVC)
		get_filename_component(PGR_TRICLOPS_LIBNAME ${PGR_TRICLOPS_LIB_FILE_RELEASE} NAME_WE)
		get_filename_component(PGR_FC2BRIDGE_LIBNAME ${PGR_FC2BRIDGE_LIB_FILE_RELEASE} NAME_WE)
		set_property(TARGET ${PROJECT_NAME} APPEND_STRING PROPERTY LINK_FLAGS " /DELAYLOAD:\"${PGR_TRICLOPS_LIBNAME}.DLL\"")
		set_property(TARGET ${PROJECT_NAME} APPEND_STRING PROPERTY LINK_FLAGS " /DELAYLOAD:\"${PGR_FC2BRIDGE_LIBNAME}.DLL\"")
	endif()
endif()

if(CMAKE_MRPT_HAS_ROBOPEAK_LIDAR)
	target_link_libraries(${PROJECT_NAME} PRIVATE mrpt_rplidar)
endif()

# Remove the "--pedantic" flag for the file "CFFMPEG_InputStream.cpp" since headers
#  have some errors:
if(CMAKE_COMPILER_IS_GNUCXX AND CMAKE_MRPT_HAS_FFMPEG_SYSTEM)
	set(EXTRA_ISYSTEMS "")
	if(EXISTS "${LIBAVCODEC_INCLUDEDIR}")
		set(EXTRA_ISYSTEMS "-isystem ${LIBAVCODEC_INCLUDEDIR} ")
	endif()
	if(EXISTS "${LIBAVCODEC_INCLUDEDIR}/ffmpeg")
		set(EXTRA_ISYSTEMS "-isystem ${LIBAVCODEC_INCLUDEDIR}/ffmpeg ")
	endif()
	if(EXISTS "${LIBAVCODEC_INCLUDEDIR}/libavcodec")
		set(EXTRA_ISYSTEMS "-isystem ${LIBAVCODEC_INCLUDEDIR}/libavcodec ")
	endif()
	if(EXISTS "${LIBAVCODEC_INCLUDEDIR}/libavformat")
		set(EXTRA_ISYSTEMS "-isystem ${LIBAVCODEC_INCLUDEDIR}/libavformat ")
	endif()
	if (NOT "${EXTRA_ISYSTEMS}" STREQUAL "")
		set_source_files_properties(
			${CMAKE_CURRENT_SOURCE_DIR}/src/CFFMPEG_InputStream.cpp
			PROPERTIES COMPILE_FLAGS ${EXTRA_ISYSTEMS}
			)
	endif ()
endif()

if (CMAKE_MRPT_HAS_FFMPEG)
  if (NOT TARGET imp_ffmpeg)
	  message(ERROR "imp_ffmpeg should be defined!")
  endif()
  target_link_libraries(${PROJECT_NAME} PRIVATE imp_ffmpeg)
endif()

if(CMAKE_MRPT_HAS_WXWIDGETS)
	if (NOT TARGET imp_wxwidgets)
		message(ERROR "imp_wxwidgets should be defined!")
	endif()
	target_link_libraries(${PROJECT_NAME} PRIVATE imp_wxwidgets)
endif()

if (CMAKE_MRPT_HAS_MYNTEYE_D)
	target_link_libraries(${PROJECT_NAME} PRIVATE mynteye_depth)
endif()

# For velodyne driver:
if (MINGW)
	target_link_libraries(${PROJECT_NAME} PRIVATE WS2_32 winmm)
endif()
