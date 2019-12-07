
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
	set_target_properties(mrpt_freenect PROPERTIES FOLDER "3rd party")

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
		target_compile_options(mrpt_rplidar PRIVATE /DWIN64)
	endif()

	# Embedded version:
	target_include_directories(mrpt_rplidar SYSTEM PUBLIC
		${MRPT_SOURCE_DIR}/3rdparty/rplidar/include/
		${MRPT_SOURCE_DIR}/3rdparty/rplidar/src/
	)
	set_target_properties(mrpt_rplidar PROPERTIES FOLDER "3rd party")

	target_link_libraries(mrpt_rplidar PRIVATE mrpt::core) # To enforce c++17

	unset(SRCS_LIST)
	unset(SRCS)
endif()

# ----------------------------------
# Build xSens MT3
# ----------------------------------
if(CMAKE_MRPT_HAS_xSENS_MT3)
	file(GLOB SRCS
		xSens_MT3/*.cpp
		xSens_MT3/*.h
	)

	add_library(mrpt_xsens_mt3 STATIC ${SRCS})

	if (MRPT_COMPILER_IS_GCC_OR_CLANG)
		target_compile_options(mrpt_xsens_mt3 PRIVATE
			-fPIC
			-Wno-variadic-macros
			-Wno-parentheses
			-Wno-ignored-qualifiers
		)
	endif()
	target_compile_definitions(mrpt_xsens_mt3 PRIVATE
		_LOG_TO_STDOUT
	)
	# Embedded version:
	target_include_directories(mrpt_xsens_mt3 SYSTEM PUBLIC
		${MRPT_SOURCE_DIR}/3rdparty
	)

	target_link_libraries(mrpt_xsens_mt3 PRIVATE mrpt::core)
	set_target_properties(mrpt_xsens_mt3 PROPERTIES FOLDER "3rd party")
	unset(SRCS)
endif()

# ----------------------------------
# Build xSens MT4
# ----------------------------------
if(CMAKE_MRPT_HAS_xSENS_MT4)
	file(GLOB SRCS
		xSens_MT4/xcommunication/src/*.cpp
		xSens_MT4/xcommunication/src/*.c
		xSens_MT4/xcommunication/include/xsens/*.h
		xSens_MT4/xstypes/src/*.c
		xSens_MT4/xstypes/include/xsens/*.h
	)

	add_library(mrpt_xsens_mt4 STATIC ${SRCS})

	if (MRPT_COMPILER_IS_GCC_OR_CLANG)
		target_compile_options(mrpt_xsens_mt4 PRIVATE
			-fPIC
			-fms-extensions  # Unnamed structs in unions
			-Wno-variadic-macros
			-Wno-parentheses
		)
		target_compile_definitions(mrpt_xsens_mt4 PRIVATE
			_POSIX_C_SOURCE=200809L  # Needed to have CLOCK_REALTIME,... in gcc
			_LOG_TO_STDOUT
		)
	endif()

	# Embedded version:
	target_include_directories(mrpt_xsens_mt4 SYSTEM PUBLIC
		${MRPT_SOURCE_DIR}/3rdparty/xSens_MT4/xcommunication/include
		${MRPT_SOURCE_DIR}/3rdparty/xSens_MT4/xcommunication/include/xsens
		${MRPT_SOURCE_DIR}/3rdparty/xSens_MT4/xstypes/include
		${MRPT_SOURCE_DIR}/3rdparty/xSens_MT4/xstypes/include/xsens
	)

	target_link_libraries(mrpt_xsens_mt4 PRIVATE mrpt::core)
	set_target_properties(mrpt_xsens_mt4 PROPERTIES FOLDER "3rd party")
	unset(SRCS)
endif()
