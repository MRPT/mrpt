# Check for system jpeglib:
# ===================================================
set(CMAKE_MRPT_HAS_JPEG 0)

if(MSVC OR APPLE)
	set(CMAKE_MRPT_HAS_JPEG 1)
	set(CMAKE_MRPT_HAS_JPEG_SYSTEM 0)
else()
	find_package(JPEG)
	if(JPEG_FOUND)
		#message(STATUS "Found library: jpeg  - Include: ${JPEG_INCLUDE_DIR}")
		include_directories("${JPEG_INCLUDE_DIR}")

		set(CMAKE_MRPT_HAS_JPEG 1)
		set(CMAKE_MRPT_HAS_JPEG_SYSTEM 1)
	else()
		set(CMAKE_MRPT_HAS_JPEG_SYSTEM 0)
	endif()
endif()

if((NOT CMAKE_MRPT_HAS_JPEG_SYSTEM) AND NOT ("${CMAKE_SYSTEM_NAME}" STREQUAL "Emscripten"))
	include(ExternalProject)

	set(CMAKE_MRPT_HAS_JPEG 1)

	find_program(NASM_PATH nasm)
	if(NASM_PATH)
		set(JPEG_ENABLE_SIMD 1)
	else(NASM_PATH)
		set(JPEG_ENABLE_SIMD 0)
	endif(NASM_PATH)

	ExternalProject_Add(EP_JPEG
		URL               "https://github.com/libjpeg-turbo/libjpeg-turbo/archive/1.5.90.tar.gz"
		URL_MD5           "85f7f9c377b70cbf48e61726097d4efa"
		SOURCE_DIR        "${MRPT_BINARY_DIR}/3rdparty/libjpeg-turbo/"
		CMAKE_ARGS
			-DENABLE_SHARED=OFF
			-DCMAKE_POSITION_INDEPENDENT_CODE=ON
			-DWITH_SIMD=${JPEG_ENABLE_SIMD}
			-DWITH_CRT_DLL=ON
			-DCMAKE_TOOLCHAIN_FILE=${CMAKE_TOOLCHAIN_FILE}
		INSTALL_COMMAND   ""
	)
	
	ExternalProject_Get_Property(EP_JPEG BINARY_DIR)
	set(JPEG_LIBRARY_NAME jpeg)
	if(MSVC)
		set(JPEG_LIBRARY_NAME jpeg-static)
		set(OBJDIR "$<CONFIG>/")
	endif()

	set(JPEG_INCLUDE_DIRS
		"${MRPT_BINARY_DIR}/3rdparty/libjpeg-turbo/"
		"${BINARY_DIR}")
	set(JPEG_LIBRARIES ${BINARY_DIR}/${OBJDIR}${CMAKE_STATIC_LIBRARY_PREFIX}${JPEG_LIBRARY_NAME}${CMAKE_STATIC_LIBRARY_SUFFIX})

	add_library(JPEG STATIC IMPORTED)
	set_property(TARGET JPEG PROPERTY IMPORTED_LOCATION ${JPEG_LIBRARIES})
	add_dependencies(JPEG EP_JPEG)

	include_directories("${JPEG_INCLUDE_DIRS}")
endif()
