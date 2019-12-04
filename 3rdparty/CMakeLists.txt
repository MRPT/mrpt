
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
		target_compile_options(mrpt_freenect PRIVATE -fPIC)
		target_compile_options(mrpt_freenect PRIVATE
			-Wno-ignored-qualifiers
			-Wno-unused-function
			-Wno-unused-const-variable
		)
	endif()

	# Embedded version:
	target_include_directories(mrpt_freenect SYSTEM PUBLIC
		${MRPT_SOURCE_DIR}/3rdparty/libfreenect/
	)
endif()
