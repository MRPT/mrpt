
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

	find_package(libusb-1.0)

	# Special stuff for windows:
	set(SRCS
		libfreenect/src/cameras.c
		libfreenect/src/core.c
		libfreenect/src/flags.c
		libfreenect/src/loader.c
		libfreenect/src/registration.c
		libfreenect/src/audio.c
		libfreenect/src/tilt.c
		libfreenect/src/usb_libusb10.c
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
		${MRPT_SOURCE_DIR}/3rdparty/libfreenect/include
	)
	# libfreenect.h includes libusb.h:
	if (LIBUSB_1_INCLUDE_DIRS)
		target_include_directories(mrpt_freenect SYSTEM PUBLIC
			${LIBUSB_1_INCLUDE_DIRS}
			${LIBUSB_1_INCLUDE_DIRS}/libusb-1.0
		)
	endif()
	if (LIBUSB_1_LIBRARIES)
		message(STATUS "LIBUSB_1_LIBRARIES: ${LIBUSB_1_LIBRARIES}")
		target_link_libraries(mrpt_freenect PUBLIC ${LIBUSB_1_LIBRARIES})
	endif()
	set_target_properties(mrpt_freenect PROPERTIES FOLDER "3rd party")

	unset(SRCS)
endif()

# ----------------------------------
# Build RPLIDAR?
# ----------------------------------
if(CMAKE_MRPT_HAS_ROBOPEAK_LIDAR)
	set(RPLIDAR_ROOT ${MRPT_SOURCE_DIR}/3rdparty/rplidar_sdk/sdk/sdk/)
	set(SRCS_LIST
		${RPLIDAR_ROOT}/src/*.cpp
		${RPLIDAR_ROOT}/src/hal/*.cpp
		${RPLIDAR_ROOT}/include/*.h
	)
	if (WIN32)
		list(APPEND SRCS_LIST ${RPLIDAR_ROOT}/src/arch/win32/*.cpp)
	else()
		list(APPEND SRCS_LIST ${RPLIDAR_ROOT}/src/arch/linux/*.cpp)
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
	if (MRPT_COMPILER_IS_CLANG)
		target_compile_options(mrpt_rplidar PRIVATE -Wno-c++11-narrowing)
	endif()
	if (CMAKE_COMPILER_IS_GNUCXX)
		target_compile_options(mrpt_rplidar PRIVATE -Wno-narrowing)
	endif()

	# Embedded version:
	target_include_directories(mrpt_rplidar SYSTEM PUBLIC
		${RPLIDAR_ROOT}/include/
		${RPLIDAR_ROOT}/src/
	)
	set_target_properties(mrpt_rplidar PROPERTIES FOLDER "3rd party")

	target_link_libraries(mrpt_rplidar PRIVATE mrpt::core) # To enforce c++17

	unset(SRCS_LIST)
	unset(SRCS)
	unset(RPLIDAR_ROOT)
endif()

# ----------------------------------
# Build xSens IMU library
# ----------------------------------
if(CMAKE_MRPT_HAS_xSENS)
	file(GLOB_RECURSE SRCS
		xspublic/*.cpp
		xspublic/*.c
		xspublic/*.h
	)

	add_library(mrpt_xsens STATIC ${SRCS})

	if (MRPT_COMPILER_IS_GCC_OR_CLANG)
		target_compile_options(mrpt_xsens PRIVATE
			-fPIC
			-fms-extensions  # Unnamed structs in unions
			-Wno-variadic-macros
			-Wno-parentheses
		)
		target_compile_definitions(mrpt_xsens PRIVATE
			_POSIX_C_SOURCE=200809L  # Needed to have CLOCK_REALTIME,... in gcc
			_LOG_TO_STDOUT
		)
	endif()

	# Embedded version:
	target_include_directories(mrpt_xsens SYSTEM PUBLIC
		${MRPT_SOURCE_DIR}/3rdparty/xspublic/
		${MRPT_SOURCE_DIR}/3rdparty/xspublic/xstypes/
		${MRPT_SOURCE_DIR}/3rdparty/xspublic/xscommon/
		${MRPT_SOURCE_DIR}/3rdparty/xspublic/xscontroller/
	)

	# Couldn't make this to work with a regular target_compile_options() cmd:
	if (MSVC)
		set(xsens_flag "/FI ${MRPT_SOURCE_DIR}/3rdparty/xspublic/msvc_include.h ")
	else()
		set(xsens_flag "-include xscommon_config.h -include xsens_compat.h -include xscontrollerconfig.h ")
	endif()
	set_property(TARGET mrpt_xsens APPEND_STRING
		PROPERTY COMPILE_FLAGS ${xsens_flag}
	)

	if (MSVC)
		target_compile_definitions(mrpt_xsens PRIVATE XDA_STATIC_LIB)
	endif()


  # Feature should be "cxx_std_11", but it's unknown to cmake 3.5.1 in Xenial:
	target_compile_features(mrpt_xsens PRIVATE cxx_constexpr)
	set_target_properties(mrpt_xsens PROPERTIES FOLDER "3rd party")
	unset(SRCS)
endif()



# ----------------------------------
# Built-in version of freeglut?
# ----------------------------------
if(CMAKE_MRPT_HAS_OPENGL_GLUT AND NOT CMAKE_MRPT_HAS_OPENGL_GLUT_SYSTEM)
	file(GLOB SRCS
		freeglut/*.c
		freeglut/GL/*.h
	)
	add_library(mrpt_freeglut STATIC ${SRCS})

	if (MRPT_COMPILER_IS_GCC_OR_CLANG)
		target_compile_options(mrpt_freeglut PRIVATE
			-fPIC
		)
	endif()
	CHECK_INCLUDE_FILE("sys/time.h" HAVE_SYS_TIME_H)
	CHECK_INCLUDE_FILE("sys/ioctl.h" HAVE_SYS_IOCTL_H)
	CHECK_INCLUDE_FILE("fcntl.h" HAVE_FCNTL_H)
	CHECK_INCLUDE_FILE("unistd.h" HAVE_UNISTD_H)

	target_compile_definitions(mrpt_freeglut PUBLIC
		FREEGLUT_STATIC # or /DFREEGLUT_EXPORTS
		HAVE_SYS_TIME_H=$<BOOL:${HAVE_SYS_TIME_H}>
		HAVE_SYS_IOCTL_H=$<BOOL:${HAVE_SYS_IOCTL_H}>
		HAVE_FCNTL_H=$<BOOL:${HAVE_FCNTL_H}>
		HAVE_UNISTD_H=$<BOOL:${HAVE_UNISTD_H}>
		)

	# Embedded version:
	target_include_directories(mrpt_freeglut SYSTEM PUBLIC
		${MRPT_SOURCE_DIR}/3rdparty/freeglut/
	)
	set_target_properties(mrpt_freeglut PROPERTIES FOLDER "3rd party")

	# Depend on X11 lib:
	if (UNIX)
		target_link_libraries(mrpt_freeglut PUBLIC
			X11
			${OPENGL_glu_LIBRARY}
		)
	endif()
	unset(SRCS)
endif()

# ----------------------------------
# Built-in version of Glew?
# ----------------------------------
if(WIN32)
	file(GLOB SRCS
		glew/src/glew.c
		glew/include/GL/*.h
	)
	add_library(mrpt_glew STATIC ${SRCS})

	if (MRPT_COMPILER_IS_GCC_OR_CLANG)
		target_compile_options(mrpt_glew PRIVATE
			-fPIC
		)
	endif()

	target_compile_definitions(mrpt_glew PUBLIC
		GLEW_STATIC
	)

	# Embedded version:
	target_include_directories(mrpt_glew SYSTEM PUBLIC
		${MRPT_SOURCE_DIR}/3rdparty/glew/include/
		${MRPT_SOURCE_DIR}/3rdparty/glext/include/
	)
	set_target_properties(mrpt_glew PROPERTIES FOLDER "3rd party")
	unset(SRCS)
endif()

# ----------------------------------
# nanogui:
# ----------------------------------
# Already handled in cmakemodules/script_nanogui.cmake

# ----------------------------------
# libfyaml
# ----------------------------------
# already handled in cmakemodules/script_libfyaml.cmake

# ----------------------------------
# simpleini
# ----------------------------------
# already handled in cmakemodules/script_simpleini.cmake

# ----------------------------------
# nanoflann:
# ----------------------------------
# Already handled in cmakemodules/script_nanoflann.cmake

# ----------------------------------
# googletest
# ----------------------------------
if (NOT CMAKE_MRPT_HAS_GTEST_SYSTEM)
	add_library(mrptgtest
		STATIC
		${MRPT_SOURCE_DIR}/3rdparty/googletest/googletest/src/gtest-all.cc
		)

	target_include_directories(mrptgtest PRIVATE
		${MRPT_SOURCE_DIR}/3rdparty/googletest/googletest/src/
		${MRPT_SOURCE_DIR}/3rdparty/googletest/googletest/
	)
	target_include_directories(mrptgtest PUBLIC
		${MRPT_SOURCE_DIR}/3rdparty/googletest/googletest/include/
	)

	mrpt_lib_target_requires_cpp17(mrptgtest)

	if (UNIX AND MRPT_COMPILER_IS_GCC_OR_CLANG)
		target_compile_options(mrptgtest PRIVATE -fPIC)
	endif()
	# Fix a "bug" in VS11 (MSVC 2012):
	if(MSVC)
		target_compile_definitions(mrptgtest PUBLIC _VARIADIC_MAX=10)
	endif()
	if(MRPT_COMPILER_IS_CLANG AND APPLE)
		target_compile_definitions(mrptgtest PUBLIC GTEST_USE_OWN_TR1_TUPLE=1)
	endif()

	set_target_properties(mrptgtest PROPERTIES FOLDER "3rd party")
endif()
