# Check for the OpenGL libraries (OpenGL, EGL, GLES)
# ===================================================

option(DISABLE_OPENGL "Disable the OpenGL library" "OFF")
mark_as_advanced(DISABLE_OPENGL)

set(CMAKE_MRPT_HAS_OPENGL 0)
set(CMAKE_MRPT_HAS_OPENGL_SYSTEM 0)

set(CMAKE_MRPT_HAS_EGL 0)
set(CMAKE_MRPT_HAS_EGL_SYSTEM 0)

set(CMAKE_MRPT_HAS_GLES 0)
set(CMAKE_MRPT_HAS_GLES_SYSTEM 0)

if(DISABLE_OPENGL)
	return()
endif()

# Start with an empty list
unset(MRPT_OPENGL_LIBS)


# Read: https://cmake.org/cmake/help/latest/module/FindOpenGL.html
if (POLICY CMP0072)
	# prefer non-legacy opengl libs
	cmake_policy(SET CMP0072 NEW)
endif()
find_package(OpenGL)

set(MRPT_GL_LIB OpenGL::GL)

# EGL:
find_package(EGL)

if (EGL_FOUND)
	set(CMAKE_MRPT_HAS_EGL 1)
	set(CMAKE_MRPT_HAS_EGL_SYSTEM 1)
	list(APPEND MRPT_GL_LIB EGL::EGL)
endif()

# GLES:
pkg_check_modules(GLESV2 QUIET IMPORTED_TARGET glesv2)

if (GLESV2_FOUND)
	CHECK_INCLUDE_FILE("GLES/gl.h"       HAVE_GLES_GL_H)
	CHECK_INCLUDE_FILE("GLES/glext.h"    HAVE_GLES_GLEXT_H)
	CHECK_INCLUDE_FILE("GLES2/gl2.h"     HAVE_GLES2_GL2_H)
	CHECK_INCLUDE_FILE("GLES2/gl2ext.h"  HAVE_GLES2_GL2EXT_H)
	CHECK_INCLUDE_FILE("GLES3/gl3.h"     HAVE_GLES3_GL3_H)
	CHECK_INCLUDE_FILE("GLES3/gl3ext.h"  HAVE_GLES3_GL3EXT_H)

	set(CMAKE_MRPT_HAS_GLES 1)
	set(CMAKE_MRPT_HAS_GLES_SYSTEM 1)
	list(APPEND MRPT_GL_LIB PkgConfig::GLESV2)
endif()

if(OpenGL_FOUND)
	add_library(imp_opengl INTERFACE IMPORTED)
	set_target_properties(imp_opengl
		PROPERTIES
		INTERFACE_INCLUDE_DIRECTORIES "${OPENGL_INCLUDE_DIR}"
		INTERFACE_LINK_LIBRARIES "${MRPT_GL_LIB}"
		)
	list(APPEND MRPT_OPENGL_LIBS imp_opengl)

	set(CMAKE_MRPT_HAS_OPENGL 1)
	set(CMAKE_MRPT_HAS_OPENGL_SYSTEM 1)
endif()
