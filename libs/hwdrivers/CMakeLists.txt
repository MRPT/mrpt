# Lists of directories with source files:
#  See "DeclareMRPTLib.cmake" for explanations
# -------------------------------------------------

# Build Intersense devices?
if(CMAKE_MRPT_HAS_INTERSENSE)
   list(APPEND hwdrivers_EXTRA_SRCS
      "${MRPT_LIBS_ROOT}/hwdrivers/src/isense/*.c"
      "${MRPT_LIBS_ROOT}/hwdrivers/src/isense/*.h"
      )

   list(APPEND hwdrivers_EXTRA_SRCS_NAME "isense" "isense" )
endif()

# Build Kinect support via libfreenect?
if(CMAKE_MRPT_HAS_FREENECT AND CMAKE_MRPT_HAS_FREENECT_SYSTEM)
	# No need to build if it's already built in the system:
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


if(CMAKE_MRPT_HAS_DUO3D)
	include_directories("${DUO3D_INCLUDE_DIR}")
	link_directories("${DUO3D_LIB_DIR}")
endif()

if(CMAKE_MRPT_HAS_SWISSRANGE)
	include_directories("${SWR_LIBMESASR_DIR}")
endif()

if(CMAKE_MRPT_HAS_LIBPCAP)
	include_directories(${PCAP_INCLUDE_DIR})
endif()

# Macro declared in "DeclareMRPTLib.cmake":
define_mrpt_lib(
	# Lib name
	hwdrivers
	# Dependencies
	mrpt-comms # serial ports, etc.
	mrpt-maps  # used to laser preview, etc.
	mrpt-gui    # used to camera preview, etc.
	mrpt-vision  # needed to stereo rectification
	)

if(NOT BUILD_mrpt-hwdrivers)
	return()
endif()

# We need pthread's on unices
target_link_libraries(hwdrivers PRIVATE Threads::Threads)

# Ignore precompiled headers in intersense sources:
if(CMAKE_MRPT_HAS_INTERSENSE)
	if(MRPT_ENABLE_PRECOMPILED_HDRS AND MSVC)
		set_source_files_properties(${isense_FILES} PROPERTIES COMPILE_FLAGS "/Y-")
	endif()
endif()

# Link to NITE2 libraries:
if(CMAKE_MRPT_HAS_NITE2)
	# link_directories() was added in the configuration script under /cmakemodules/*_ni.cmake
	include_directories(${NITE2_INCLUDE_DIR})
	target_link_libraries(hwdrivers PRIVATE ${NITE2_LIB_FILE_RELEASE})
endif()

# XSens libs:
if(TARGET mrpt_xsens_mt3)
	target_link_libraries(hwdrivers PRIVATE mrpt_xsens_mt3)
endif()
if(TARGET mrpt_xsens_mt4)
	target_link_libraries(hwdrivers PRIVATE mrpt_xsens_mt4)
endif()

if(CMAKE_MRPT_HAS_OPENCV)
	target_link_libraries(hwdrivers PRIVATE imp_opencv)
endif()

if(CMAKE_MRPT_HAS_LIBPCAP)
	target_link_libraries(hwdrivers PRIVATE ${PCAP_LIBRARY})

	# Windows:
	# Set wpcap.dll as DELAYLOAD, so users without that driver can use binaries compiled with wpcap support:
	if (MSVC)
		set_property(TARGET hwdrivers APPEND_STRING PROPERTY LINK_FLAGS " /DELAYLOAD:\"wpcap.dll\"")
	endif ()
endif()

# XSens MT4 required libs:
if (BUILD_XSENS_MT4)
	target_link_libraries(hwdrivers PRIVATE ${XSENS4_LIBS})
endif ()

if(UNIX)
    target_link_libraries(hwdrivers PRIVATE dl)
endif()

if(CMAKE_MRPT_HAS_OPENNI2)
	target_link_libraries(hwdrivers PRIVATE ${MRPT_OPENNI2_LIBRARIES})
endif()

if(CMAKE_MRPT_HAS_SWISSRANGE)
	target_link_libraries(hwdrivers PRIVATE ${MRPT_SWR_LIBS})
endif()

if(CMAKE_MRPT_HAS_DUO3D)
	target_link_libraries(hwdrivers PRIVATE  "${DUO3D_LIB_DIR}/DUOLib.lib" "${DUO3D_LIB_DIR}/Dense3D.lib")
endif()

# Seems to be required to link against prebuilt libavcodec, etc.
if (MSVC)
	#set_target_properties()
	set_property(TARGET hwdrivers APPEND_STRING PROPERTY LINK_FLAGS " /SAFESEH:NO")

	set_target_properties(hwdrivers PROPERTIES LINK_FLAGS "/DELAYLOAD:wlanapi.dll")
endif()

if(CMAKE_MRPT_HAS_FREENECT)
	# Add linking against these libs:
	if(NOT "${FREENECT_LIBS}" STREQUAL "")
		target_link_libraries(hwdrivers PRIVATE ${FREENECT_LIBS})

		# Windows:
		# Set libusb0.dll as DELAYLOAD, so users without that driver
		# still can use binaries compiled with libusb support:
		if (MSVC)
			set_property(TARGET hwdrivers APPEND_STRING PROPERTY LINK_FLAGS " /DELAYLOAD:\"libusb0.dll\"")
		endif ()
	endif()

	if(NOT CMAKE_MRPT_HAS_FREENECT_SYSTEM)
		target_link_libraries(hwdrivers PRIVATE mrpt_freenect)
	endif()
endif()

if(NOT "${LIBUSB10_LIBS}" STREQUAL "")
	target_link_libraries(hwdrivers PRIVATE ${LIBUSB10_LIBS})
endif()

# build with link on Phidget21 lib
if(CMAKE_MRPT_HAS_PHIDGET)
	include_directories(${PHIDGETS_INCLUDE_DIR})
	target_link_libraries(hwdrivers PRIVATE ${PHIDGETS_LIBRARIES})
endif()

if(CMAKE_MRPT_HAS_LIBDC1394_2)
	target_link_libraries(hwdrivers PRIVATE ${LIBDC1394_2_LIBRARIES})
endif()

# Link to National Instrument libraries:
if(MRPT_HAS_NI_DAQmxBASE)
	# link_directories() was added in the configuration script under /cmakemodules/*_ni.cmake
	include_directories(${NI_DAQmxBASE_INCLUDE_DIR})
	target_link_libraries(hwdrivers PRIVATE ${NI_DAQmxBASE_LIB_FILE})
endif()
if(MRPT_HAS_NI_DAQmx)
	# link_directories() was added in the configuration script under /cmakemodules/*_ni.cmake
	include_directories(${NI_DAQmx_INCLUDE_DIR})
	target_link_libraries(hwdrivers PRIVATE ${NI_DAQmx_LIB_FILE})
endif()

# Link to PGR FlyCapture2 libraries:
if(CMAKE_MRPT_HAS_FLYCAPTURE2)
	# link_directories() was added in the configuration script under /cmakemodules/*_ni.cmake
	include_directories(${PGR_FLYCAPTURE2_INCLUDE_DIR})
	target_link_libraries(hwdrivers PRIVATE ${PGR_FLYCAPTURE2_LIB_FILE_RELEASE})
	# Windows: Set dll as DELAYLOAD, so users without that driver can use precompiled binaries:
	if(MSVC)
		set_property(TARGET hwdrivers APPEND_STRING PROPERTY LINK_FLAGS " /DELAYLOAD:\"FLYCAPTURE2.DLL\"")
	endif()
endif()

# Link to PGR Triclops libraries:
if(CMAKE_MRPT_HAS_TRICLOPS)
	# link_directories() was added in the configuration script under /cmakemodules/*_ni.cmake
	include_directories(${PGR_TRICLOPS_INCLUDE_DIR})
	target_link_libraries(hwdrivers PRIVATE ${PGR_TRICLOPS_LIB_FILE_RELEASE})
	target_link_libraries(hwdrivers PRIVATE ${PGR_FC2BRIDGE_LIB_FILE_RELEASE})
	# Windows: Set dll as DELAYLOAD, so users without that driver can use precompiled binaries:
	if(MSVC)
		get_filename_component(PGR_TRICLOPS_LIBNAME ${PGR_TRICLOPS_LIB_FILE_RELEASE} NAME_WE)
		get_filename_component(PGR_FC2BRIDGE_LIBNAME ${PGR_FC2BRIDGE_LIB_FILE_RELEASE} NAME_WE)
		set_property(TARGET hwdrivers APPEND_STRING PROPERTY LINK_FLAGS " /DELAYLOAD:\"${PGR_TRICLOPS_LIBNAME}.DLL\"")
		set_property(TARGET hwdrivers APPEND_STRING PROPERTY LINK_FLAGS " /DELAYLOAD:\"${PGR_FC2BRIDGE_LIBNAME}.DLL\"")
	endif()
endif()

if(CMAKE_MRPT_HAS_ROBOPEAK_LIDAR)
	target_link_libraries(hwdrivers PRIVATE mrpt_rplidar)
endif()

# Build with link on SVS lib
if(CMAKE_MRPT_HAS_SVS)
	target_link_libraries(hwdrivers PRIVATE dcap svs svscalc svscap)
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
			${MRPT_LIBS_ROOT}/hwdrivers/src/CFFMPEG_InputStream.cpp
			PROPERTIES COMPILE_FLAGS ${EXTRA_ISYSTEMS}
			)
	endif ()
endif()

if (CMAKE_MRPT_HAS_FFMPEG)
  if (NOT TARGET imp_ffmpeg)
	  message(ERROR "imp_ffmpeg should be defined!")
  endif()
  target_link_libraries(hwdrivers PRIVATE imp_ffmpeg)
endif()

if(CMAKE_MRPT_HAS_WXWIDGETS)
	if (NOT TARGET imp_wxwidgets)
		message(ERROR "imp_wxwidgets should be defined!")
	endif()
	target_link_libraries(hwdrivers PRIVATE imp_wxwidgets)
endif()
