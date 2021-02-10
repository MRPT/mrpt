# Check for the GL,GLUT libraries (Linux only, in Windows use the build in .h & .lib)
# ===================================================

option(DISABLE_OPENGL "Disable the OpenGL library" "OFF")
mark_as_advanced(DISABLE_OPENGL)

set(CMAKE_MRPT_HAS_OPENGL_GLUT 0)
set(CMAKE_MRPT_HAS_OPENGL_GLUT_SYSTEM 0)

set(CMAKE_MRPT_HAS_GLUT 0)
set(CMAKE_MRPT_HAS_GLUT_SYSTEM 0)

if(DISABLE_OPENGL)
	return()
endif()

# Start with an empty list
unset(MRPT_OPENGL_LIBS)


# Read: https://cmake.org/cmake/help/latest/module/FindOpenGL.html
#set(OpenGL_GL_PREFERENCE "LEGACY")
find_package(OpenGL)

if(UNIX)
	find_package(GLUT)
endif()

if(OpenGL_FOUND)
	add_library(imp_opengl INTERFACE IMPORTED)
	set_target_properties(imp_opengl
		PROPERTIES
		INTERFACE_INCLUDE_DIRECTORIES "${OPENGL_INCLUDE_DIR}"
		INTERFACE_LINK_LIBRARIES "OpenGL::GL"
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
