# Lists of directories with source files:
#  See "DeclareMRPTLib.cmake" for explanations
# -------------------------------------------------

# Build xSens?
IF(CMAKE_MRPT_HAS_xSENS)
	LIST(APPEND hwdrivers_EXTRA_SRCS
		"${MRPT_LIBS_ROOT}/hwdrivers/src/xSens/*.cpp"
		"${MRPT_LIBS_ROOT}/hwdrivers/src/xSens/*.h"
		)
	LIST(APPEND hwdrivers_EXTRA_SRCS_NAME 
		"xSens"
		"xSens"
		)
ENDIF(CMAKE_MRPT_HAS_xSENS)

# Build Xbox Kinect?
IF(CMAKE_MRPT_HAS_FREENECT)
	IF (CMAKE_MRPT_HAS_FREENECT_SYSTEM)
		# No need to build if it's already built in the system:
		INCLUDE_DIRECTORIES("${OPENKINECT_FREENECT_INCLUDE_DIR}")
	ELSE (CMAKE_MRPT_HAS_FREENECT_SYSTEM)
		# Embedded version:
		INCLUDE_DIRECTORIES("${MRPT_LIBS_ROOT}/hwdrivers/src/libfreenect/")
		LIST(APPEND hwdrivers_EXTRA_SRCS
			"${MRPT_LIBS_ROOT}/hwdrivers/src/libfreenect/*.c"
			"${MRPT_LIBS_ROOT}/hwdrivers/src/libfreenect/*.h"
			)
		LIST(APPEND hwdrivers_EXTRA_SRCS_NAME 
			"kinect"
			"kinect"
			)
			
		# Special stuff for windows:
		if (WIN32)
			LIST(APPEND hwdrivers_EXTRA_SRCS
				"${MRPT_LIBS_ROOT}/hwdrivers/src/libfreenect/platform/windows/libusb10emu/libusb-1.0/libusbemu.cpp"
				"${MRPT_LIBS_ROOT}/hwdrivers/src/libfreenect/platform/windows/libusb10emu/libusb-1.0/failguard.cpp"
				)
			LIST(APPEND hwdrivers_EXTRA_SRCS_NAME 
				"kinect"
				"kinect"
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

# Macro declared in "DeclareMRPTLib.cmake":
define_mrpt_lib(
	# Lib name
	hwdrivers 	
	# Dependencies
	mrpt-base
	mrpt-obs 
	mrpt-gui  # used to camera preview, etc.
	mrpt-opengl  # needed by mrpt-gui
	)
	
IF(CMAKE_MRPT_HAS_xSENS)
	# Ignore precompiled headers in xSens sources:
	IF(MRPT_ENABLE_PRECOMPILED_HDRS AND MSVC)
		set_source_files_properties(${xSens_FILES} PROPERTIES COMPILE_FLAGS "/Y-")
	ENDIF(MRPT_ENABLE_PRECOMPILED_HDRS AND MSVC)

	# Avoid some annoying warnings:
	IF (CMAKE_COMPILER_IS_GNUCXX)
		set_source_files_properties(${xSens_FILES} PROPERTIES COMPILE_FLAGS "-Wno-variadic-macros")
	ENDIF (CMAKE_COMPILER_IS_GNUCXX)

	ADD_DEFINITIONS(-D_LOG_TO_STDOUT)
ENDIF(CMAKE_MRPT_HAS_xSENS)

IF(CMAKE_MRPT_HAS_FREENECT)
	# Add linking against these libs:
	IF(NOT "${FREENECT_LIBS}" STREQUAL "")
		TARGET_LINK_LIBRARIES(mrpt-hwdrivers ${FREENECT_LIBS})
	ENDIF(NOT "${FREENECT_LIBS}" STREQUAL "")
		
	if (NOT CMAKE_MRPT_HAS_FREENECT_SYSTEM)
		# Using embedded version:
		# Ignore precompiled headers:
		IF(MRPT_ENABLE_PRECOMPILED_HDRS AND MSVC)
			set_source_files_properties(${kinect_FILES} PROPERTIES COMPILE_FLAGS "/Y-")
		ENDIF(MRPT_ENABLE_PRECOMPILED_HDRS AND MSVC)
		
		if (MSVC)
			set_source_files_properties(${kinect_FILES} PROPERTIES LANGUAGE CXX) 			# Build as C++
		endif(MSVC)
	endif (NOT CMAKE_MRPT_HAS_FREENECT_SYSTEM)
ENDIF(CMAKE_MRPT_HAS_FREENECT)

# build with link on Phidget21 lib
IF(CMAKE_MRPT_HAS_PHIDGET)
	TARGET_LINK_LIBRARIES(mrpt-hwdrivers phidget21)
ENDIF(CMAKE_MRPT_HAS_PHIDGET)

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

	set_source_files_properties(
		${MRPT_LIBS_ROOT}/hwdrivers/src/CFFMPEG_InputStream.cpp 
		PROPERTIES COMPILE_FLAGS ${EXTRA_ISYSTEMS}
		)
ENDIF(CMAKE_COMPILER_IS_GNUCXX AND CMAKE_MRPT_HAS_FFMPEG_SYSTEM)

