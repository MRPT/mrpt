
include(../cmakemodules/AssureCMakeRootFile.cmake) # Avoid user mistake in CMake source directory

# ----------------------------------
# wxThings
# ----------------------------------
if(CMAKE_MRPT_HAS_WXWIDGETS)
	add_subdirectory(wxThings)
endif()

# ----------------------------------
# libfreenect: Embedded version
# ----------------------------------
if(CMAKE_MRPT_HAS_FREENECT AND NOT CMAKE_MRPT_HAS_FREENECT_SYSTEM)

	# Special stuff for windows:
	set(SRCS
		libfreenect/cameras.c
		libfreenect/core.c
		libfreenect/tilt.c
		libfreenect/usb_libusb10.c
	)

	if (WIN32)
		list(APPEND SRCS
			libfreenect/platform/windows/libusb10emu/libusb-1.0/libusbemu.cpp
			libfreenect/platform/windows/libusb10emu/libusb-1.0/failguard.cpp
			)
	endif()

	add_library(mrpt_freenect STATIC ${SRCS})

	if (WIN32)
		target_include_directories(mrpt_freenect SYSTEM PUBLIC
			${MRPT_SOURCE_DIR}/3rdparty/libfreenect/platform/windows
			${MRPT_SOURCE_DIR}/3rdparty/libfreenect/platform/windows/libusb10emu
			${LIBUSB_1_INCLUDE_DIRS} # libfreenect.h includes libusb.h:
		)
	endif()

	if (MSVC)
		set_source_files_properties(${SRCS} PROPERTIES LANGUAGE CXX) # Build as C++
	endif()

	if (MRPT_COMPILER_IS_GCC_OR_CLANG)
		target_compile_options(mrpt_freenect PRIVATE
			-fPIC
			-Wno-ignored-qualifiers
			-Wno-unused-function
			-Wno-unused-const-variable
		)
	endif()

	# Embedded version:
	target_include_directories(mrpt_freenect SYSTEM PUBLIC
		${MRPT_SOURCE_DIR}/3rdparty/libfreenect/
	)
	unset(SRCS)
endif()

# ----------------------------------
# Build RPLIDAR?
# ----------------------------------
if(CMAKE_MRPT_HAS_ROBOPEAK_LIDAR)
	set(SRCS_LIST
		rplidar/src/*.cpp
		rplidar/src/hal/*.cpp
		rplidar/include/*.h
	)
	if (WIN32)
		list(APPEND SRCS_LIST rplidar/src/arch/win32/*.cpp)
	else()
		list(APPEND SRCS_LIST rplidar/src/arch/linux/*.cpp)
	endif()
	file(GLOB SRCS ${SRCS_LIST})

	add_library(mrpt_rplidar STATIC ${SRCS})

	if (MRPT_COMPILER_IS_GCC_OR_CLANG)
		target_compile_options(mrpt_rplidar PRIVATE
			-fPIC
		)
	endif()
	if(MSVC)
		target_compile_options(mrpt_rplidar PRIVATE WIN64)
	endif()

	# Embedded version:
	target_include_directories(mrpt_rplidar SYSTEM PUBLIC
		${MRPT_SOURCE_DIR}/3rdparty/rplidar/include/
		${MRPT_SOURCE_DIR}/3rdparty/rplidar/src/
		#${MRPT_SOURCE_DIR}/3rdparty
	)

	unset(SRCS_LIST)
	unset(SRCS)
endif()
