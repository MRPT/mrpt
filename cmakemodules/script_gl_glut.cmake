# Check for the GL,GLUT libraries (Linux only, in Windows use the build in .h & .lib)
# ===================================================

option(DISABLE_OPENGL "Disable the OpenGL library" "OFF")
mark_as_advanced(DISABLE_OPENGL)

set(CMAKE_MRPT_HAS_OPENGL_GLUT 0)
set(CMAKE_MRPT_HAS_OPENGL_GLUT_SYSTEM 0)

set(CMAKE_MRPT_HAS_GLUT 0)
set(CMAKE_MRPT_HAS_GLUT_SYSTEM 0)

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

if (NOT "${CMAKE_VERSION}" VERSION_LESS "3.8.2")
	set(MRPT_GL_LIB OpenGL::GL)
else()
	set(MRPT_GL_LIB ${OPENGL_gl_LIBRARY})
endif()

# EGL:
if(CMAKE_VERSION VERSION_LESS 3.16.0)
	message(WARNING "You need cmake >=3.16.0 to detect EGL. Disabling it, some features (FBO off-screen rendering) will not be enabled in MRPT.")
else()
	find_package(EGL)
endif()

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

# glut:
if(UNIX)
	find_package(GLUT)
endif()

if(OpenGL_FOUND)
	add_library(imp_opengl INTERFACE IMPORTED)
	set_target_properties(imp_opengl
		PROPERTIES
		INTERFACE_INCLUDE_DIRECTORIES "${OPENGL_INCLUDE_DIR}"
		INTERFACE_LINK_LIBRARIES "${MRPT_GL_LIB}"
		)
	list(APPEND MRPT_OPENGL_LIBS imp_opengl)
endif()

if(UNIX AND GLUT_FOUND AND OpenGL_FOUND AND OPENGL_glu_LIBRARY AND GLUT_glut_LIBRARY)
	set(CMAKE_MRPT_HAS_OPENGL_GLUT 1)
	set(CMAKE_MRPT_HAS_OPENGL_GLUT_SYSTEM 1)

	add_library(imp_glut INTERFACE IMPORTED)
	set_target_properties(imp_glut
		PROPERTIES
		INTERFACE_INCLUDE_DIRECTORIES "${GLUT_INCLUDE_DIR}"
		INTERFACE_LINK_LIBRARIES "${OPENGL_glu_LIBRARY};${GLUT_glut_LIBRARY}"
		)
	list(APPEND MRPT_OPENGL_LIBS imp_glut)

	set(CMAKE_MRPT_HAS_GLUT 1)
	set(CMAKE_MRPT_HAS_GLUT_SYSTEM 1)
endif()

# GLUT: Windows or Linux w/o system OpenGL packages: embedded source version.
if (WIN32 OR (OpenGL_FOUND AND NOT CMAKE_MRPT_HAS_OPENGL_GLUT_SYSTEM))
	if (NOT WIN32) # In Windows, this is the expected behavior!
		message(STATUS "**Warning**: System GLUT libraries not found! Using built-in version:")
		message(STATUS "  OPENGL_gl_LIBRARY: ${OPENGL_gl_LIBRARY}")
		message(STATUS "  OPENGL_glu_LIBRARY: ${OPENGL_glu_LIBRARY}")
		message(STATUS "  GLUT_glut_LIBRARY: ${GLUT_glut_LIBRARY}")
		message(STATUS "  OPENGL_INCLUDE_DIR: ${OPENGL_INCLUDE_DIR}")
	endif()

	set(CMAKE_MRPT_HAS_OPENGL_GLUT 1)
	set(CMAKE_MRPT_HAS_OPENGL_GLUT_SYSTEM 0)

	set(CMAKE_MRPT_HAS_GLUT 1)
	set(CMAKE_MRPT_HAS_GLUT_SYSTEM 0)

	# glut from built-in lib:
	list(APPEND MRPT_OPENGL_LIBS mrpt_freeglut)
endif()
