cmake_minimum_required(VERSION 3.0)
project(CVD)
enable_testing()

include(TestBigEndian)

set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

# Dependencies that can be automatically found by CMake.

find_package(JPEG)
find_package(PNG)
find_package(TIFF)
find_package(TooN)
find_package(X11)
find_package(dc1394v2)
find_package(OpenGL)

# Detect endianness

TEST_BIG_ENDIAN(CVD_INTERNAL_ARCH_BIG_ENDIAN)
if(CVD_INTERNAL_ARCH_BIG_ENDIAN)
	set(CVD_INTERNAL_ARCH_LITTLE_ENDIAN OFF)
else()
	set(CVD_INTERNAL_ARCH_LITTLE_ENDIAN ON)
endif()

option(CVD_SSE2 "Enable SSE2" OFF)
option(CVD_SSE "Enable SSE" OFF)
option(CVD_FAST "Enable FAST" ON)
option(CVD_TESTS "Enable tests" ON)
option(CVD_PROGS "Enable programs" ON)
option(CVD_EXAMPLES "Enable examples" ON)
option(CVD_FAST_OBSCURE "Enable obscure fast algorithms" ON)

if(CVD_SSE2)
	set(CVD_SSE ON)
endif()

if(NOT MSVC)
	if(CVD_SSE2)
		add_compile_options(-O3)
		add_compile_options(-funroll-loops)
		add_compile_options(-msse2)
	endif()

	if(CVD_SSE)
		add_compile_options(-mfpmath=sse)
	endif()
endif()

# Basic source files and headers for all platforms and options.

set(SRCS
	cvd_src/bayer.cxx
	cvd_src/colourspace_convert.cxx
	cvd_src/connected_components.cc
	cvd_src/convolution.cc
	cvd_src/cvd_timer.cc
	cvd_src/deinterlacebuffer.cc
	cvd_src/diskbuffer2.cc
	cvd_src/draw.cc
	cvd_src/exceptions.cc
	cvd_src/faster_corner_utilities.h
	cvd_src/image_io.cc
	cvd_src/morphology.cc
	cvd_src/nonmax_suppression.cxx
	cvd_src/quartic.cpp
	cvd_src/timeddiskbuffer.cc
	cvd_src/videofilebuffer_exceptions.cc
	cvd_src/videosource.cpp
	cvd_src/yuv411_to_stuff.cxx
	cvd_src/yuv420.cpp
	cvd_src/fast_corner.cxx
	cvd_src/image_io/bmp.cxx
	cvd_src/image_io/bmp_read.cc
	cvd_src/image_io/bmp_write.cc
	cvd_src/image_io/cvdimage.cxx
	cvd_src/image_io/fits.cc
	cvd_src/image_io/fitswrite.cc
	cvd_src/image_io/pnm_grok.cxx
	cvd_src/image_io/save_postscript.cxx
	cvd_src/image_io/text.cxx
	cvd_src/image_io/text_write.cc
	cvd_src/noarch/convert_rgb_to_y.cc
	cvd_src/noarch/yuv422.cpp
	)


if(CVD_FAST)
	if(CVD_SSE2)
		list(APPEND SRCS
			cvd_src/SSE2/faster_corner_9.cxx
			cvd_src/SSE2/faster_corner_10.cxx
			cvd_src/SSE2/faster_corner_12.cxx
			)
	else()
		list(APPEND SRCS
			cvd_src/noarch/slower_corner_9.cxx
			cvd_src/noarch/slower_corner_10.cxx
			cvd_src/noarch/slower_corner_12.cxx
			)
	endif()

	list(APPEND SRCS
		cvd_src/fast/fast_9_detect.cxx
		cvd_src/fast/fast_9_score.cxx
		cvd_src/fast/fast_corner_9_nonmax.cxx

		cvd_src/fast/fast_10_detect.cxx
		cvd_src/fast/fast_10_score.cxx

		cvd_src/fast/fast_12_detect.cxx
		cvd_src/fast/fast_12_score.cxx
		)
endif()

if(CVD_FAST_OBSCURE)
	list(APPEND SRCS
		cvd_src/fast/fast_7_detect.cxx
		cvd_src/fast/fast_7_score.cxx
		cvd_src/fast/slower_corner_7.cxx
		cvd_src/fast/fast_8_detect.cxx
		cvd_src/fast/fast_8_score.cxx
		cvd_src/fast/slower_corner_8.cxx
		cvd_src/fast/fast_11_detect.cxx
		cvd_src/fast/fast_11_score.cxx
		cvd_src/fast/slower_corner_11.cxx
		)
endif()

if(CVD_SSE2)
	list(APPEND SRCS
		cvd_src/SSE2/faster_corner_utilities.h
		cvd_src/SSE2/gradient.cc
		cvd_src/SSE2/half_sample.cc
		cvd_src/SSE2/median_3x3.cc
		cvd_src/SSE2/two_thirds_sample.cc
		cvd_src/SSE2/utility_double_int.cc
		)
else()
	list(APPEND SRCS
		cvd_src/noarch/gradient.cc
		cvd_src/noarch/half_sample.cc
		cvd_src/noarch/median_3x3.cc
		cvd_src/noarch/two_thirds_sample.cc
		cvd_src/noarch/utility_double_int.cc
	)
endif()

if(CVD_SSE)
	list(APPEND SRCS
		cvd_src/SSE/convolve_gaussian.cc
		cvd_src/SSE/utility_float.cc
		)
else()
	list(APPEND SRCS
		cvd_src/noarch/convolve_gaussian.cc
		cvd_src/noarch/utility_float.cc
	)
endif()

if(CVD_MMX)
	list(APPEND SRCS
		cvd_src/MMX/utility_byte_differences.cc
	)
else()
	list(APPEND SRCS
		cvd_src/noarch/utility_byte_differences.cc
	)
endif()

set(HEADERS
	cvd/argb.h
	cvd/bgrx.h
	cvd/bresenham.h
	cvd/byte.h
	cvd/camera.h
	cvd/canny.h
	cvd/colourmap.h
	cvd/colourspace.h
	cvd/colourspacebuffer.h
	cvd/colourspaces.h
	cvd/colourspace_convert.h
	cvd/colourspace_frame.h
	cvd/connected_components.h
	cvd/convolution.h
	cvd/deinterlacebuffer.h
	cvd/deinterlaceframe.h
	cvd/diskbuffer2.h
	cvd/diskbuffer2_frame.h
	cvd/distance_transform.h
	cvd/documentation.h
	cvd/draw.h
	cvd/esm.h
	cvd/exceptions.h
	cvd/fast_corner.h
	cvd/gles1_helpers.h
	cvd/glwindow.h
	cvd/gl_helpers.h
	cvd/haar.h
	cvd/harris_corner.h
	cvd/helpers.h
	cvd/image.h
	cvd/image_convert.h
	cvd/image_convert_fwd.h
	cvd/image_interpolate.h
	cvd/image_io.h
	cvd/image_ref.h
	cvd/integral_image.h
	cvd/interpolate.h
	cvd/la.h
	cvd/localvideobuffer.h
	cvd/localvideoframe.h
	cvd/morphology.h
	cvd/nonmax_suppression.h
	cvd/opencv.h
	cvd/readaheadvideobuffer.h
	cvd/rgb.h
	cvd/rgb8.h
	cvd/rgba.h
	cvd/serverpushjpegbuffer.h
	cvd/serverpushjpegframe.h
	cvd/timeddiskbuffer.h
	cvd/timer.h
	cvd/utility.h
	cvd/vector_image_ref.h
	cvd/videobuffer.h
	cvd/videobufferflags.h
	cvd/videobufferwithdata.h
	cvd/videofilebuffer.h
	cvd/videofilebuffer_frame.h
	cvd/videoframe.h
	cvd/videosource.h
	cvd/vision.h
	cvd/vision_exceptions.h
	cvd/yc.h
	cvd/video/skipbuffer.h
	cvd/internal/builtin_components.h
	cvd/internal/concurrency_utilities.h
	cvd/internal/convert_pixel_types.h
	cvd/internal/gles1_types.h
	cvd/internal/gl_types.h
	cvd/internal/image_ref_implementation.hh
	cvd/internal/io/bmp.h
	cvd/internal/io/cvdimage.h
	cvd/internal/io/fits.h
	cvd/internal/io/jpeg.h
	cvd/internal/io/parameter.h
	cvd/internal/io/png.h
	cvd/internal/io/pnm_grok.h
	cvd/internal/io/save_postscript.h
	cvd/internal/io/text.h
	cvd/internal/io/tiff.h
	cvd/internal/load_and_save.h
	cvd/internal/name_builtin_types.h
	cvd/internal/name_CVD_rgb_types.h
	cvd/internal/pixel_operations.h
	cvd/internal/pixel_traits.h
	cvd/internal/rgb_components.h
	cvd/internal/scalar_convert.h
	cvd/internal/win.h)


# Library-specific source files, headers and definitions.
if(dc1394_FOUND)
	list(APPEND SRCS
		cvd_src/videosource_dvbuffer.cc
		cvd_src/Linux/dvbuffer3_dc1394v2.cc)
	list(APPEND HEADERS
		cvd/Linux/dvbuffer.h
		cvd/Linux/dvbuffer3.h
		cvd/Linux/dvframe.h)
	link_libraries(${dc1394_LIBRARIES})
else()
	list(APPEND SRCS
		cvd_src/videosource_nodvbuffer.cc)
endif()

if(OpenGL_FOUND)
	list(APPEND SRCS
		cvd_src/gltext.cpp)

	if(X11_FOUND)
		list(APPEND SRCS
			cvd_src/glwindow.cc
			cvd_src/videodisplay.cc)
		list(APPEND HEADERS cvd/videodisplay.h)

		set(CVD_HAVE_GLWINDOW ON)
		set(CVD_HAVE_VIDEODISPLAY ON)
		link_libraries(${X11_LIBRARIES})
	endif()
	link_libraries(${OPENGL_LIBRARIES})
endif()

# Platform-specific source files, headers and definitions.
if(WIN32)
	list(APPEND SRCS
		cvd_src/Win32/win32.cpp
		cvd_src/Win32/glwindow.cpp)
	set(CVD_HAVE_GLOB OFF)
	set(CVD_HAVE_GLWINDOW ON)
	add_definitions(-D_CRT_SECURE_NO_WARNINGS)
else()
    #I'm not really sure this is the correct solution?
	#This will always force the compiler into c++14 mode
	#even if it's in C++17 mode, I think.
	add_compile_options(--std=c++14)

	list(APPEND SRCS
		cvd_src/globlist.cxx)

	set(CVD_HAVE_GLOB ON)
	link_libraries(pthread)

endif()

if(USE_V4L)
	list(APPEND SRCS
		cvd_src/Linux/v4lbuffer.cc
		cvd_src/Linux/v4lcontrol.cc
		cvd_src/Linux/videosource_v4lbuffer.cc)
	list(APPEND HEADERS
		cvd/Linux/v4lbuffer.h
		cvd/Linux/v4lcontrol.h)
else()
	list(APPEND SRCS cvd_src/Linux/videosource_nov4lbuffer.cc)
endif()

if(USE_UVC)
	list(APPEND SRCS
		cvd_src/uvcbuffer.cc
		cvd_src/videosource_uvcbuffer.cc)
	list(APPEND HEADERS cvd/video/uvcbuffer.h)
else()
	list(APPEND SRCS
		cvd_src/videosource_nouvcbuffer.cc)
endif()

if(USE_AVCODEC)
	list(APPEND SRCS cvd_src/videofilebuffer2.cc
		cvd_src/videosource_videofilebuffer.cc)
else()
	list(APPEND SRCS
		cvd_src/videosource_novideofilebuffer.cc)
endif()


if(TooN_FOUND)
	include_directories("${TooN_INCLUDE_DIRS}")
	set(CVD_HAVE_TOON ON)
	list(APPEND SRCS
		cvd_src/brezenham.cc
		cvd_src/draw_toon.cc
		cvd_src/tensor_voting.cc
		cvd_src/threepointpose.cpp)
	list(APPEND HEADERS
		cvd/brezenham.h
		cvd/tensor_voting.h
		cvd/geometry/threepointpose.h)
else()
	set(CVD_HAVE_TOON OFF)
endif()

if(JPEG_INCLUDE_DIR)
	include_directories("${JPEG_INCLUDE_DIR}")
	link_libraries(${JPEG_LIBRARIES})
	set(CVD_HAVE_JPEG ON)
	set(CVD_INTERNAL_JPEG_BUFFER_SIZE 1)
    list(APPEND SRCS cvd_src/image_io/jpeg.cxx)
else()
	set(CVD_HAVE_JPEG OFF)
	set(CVD_INTERNAL_JPEG_BUFFER_SIZE OFF)
endif()

if(TIFF_INCLUDE_DIRS)
	include_directories("${TIFF_INCLUDE_DIRS}")
	link_libraries(${TIFF_LIBRARIES})
	set(CVD_HAVE_TIFF ON)
    list(APPEND SRCS
		cvd_src/image_io/tiff.cxx
		cvd_src/image_io/tiffwrite.cc)
else()
	set(CVD_HAVE_TIFF OFF)
endif()

if(PNG_INCLUDE_DIRS)
	include_directories("${PNG_INCLUDE_DIRS}")
	link_libraries(${PNG_LIBRARIES})
	set(CVD_HAVE_PNG ON)
    list(APPEND SRCS cvd_src/image_io/png.cc)
else()
	set(CVD_HAVE_PNG OFF)
endif()

configure_file(cmake/config.h.in include/cvd/config.h)
configure_file(cmake/config_internal.h.in include/cvd_src/config_internal.h)

set(CMAKE_DEBUG_POSTFIX _debug)
include_directories("${PROJECT_SOURCE_DIR}" "${CMAKE_CURRENT_BINARY_DIR}/include")
add_library(${PROJECT_NAME} ${SRCS} ${HEADERS})

install(TARGETS ${PROJECT_NAME} ARCHIVE DESTINATION lib LIBRARY DESTINATION lib)
foreach(file ${HEADERS})
	get_filename_component(dir ${file} DIRECTORY)
	install(FILES ${file} DESTINATION include/${dir})
endforeach()
install(DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/include/cvd DESTINATION include)
install(FILES ${PROJECT_SOURCE_DIR}/cmake/CVDConfig.cmake DESTINATION cmake)

if(CVD_TESTS)
add_subdirectory(tests)
endif()
if(CVD_PROGS)
add_subdirectory(progs)
endif()
if(CVD_EXAMPLES)
add_subdirectory(examples)
endif()
