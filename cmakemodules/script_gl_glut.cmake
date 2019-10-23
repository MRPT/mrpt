# Check for the GL,GLUT libraries (Linux only, in Windows use the build in .h & .lib)
# ===================================================

option(DISABLE_OPENGL "Disable the OpenGL library" "OFF")
mark_as_advanced(DISABLE_OPENGL)

if(DISABLE_OPENGL)
	set(CMAKE_MRPT_HAS_OPENGL_GLUT 0)
	set(CMAKE_MRPT_HAS_OPENGL_GLUT_SYSTEM 0)

	set(CMAKE_MRPT_HAS_GLUT 0)
	set(CMAKE_MRPT_HAS_GLUT_SYSTEM 0)
	return()
endif()


# Read: https://cmake.org/cmake/help/latest/module/FindOpenGL.html
set(OpenGL_GL_PREFERENCE "LEGACY")
find_package(OpenGL)

if(UNIX)
	find_package(GLUT)
endif()

if(OpenGL_FOUND)
	add_library(imp_opengl INTERFACE IMPORTED)
	set_target_properties(imp_opengl
		PROPERTIES
		INTERFACE_INCLUDE_DIRECTORIES "${OPENGL_INCLUDE_DIR}"
		INTERFACE_LINK_LIBRARIES "${OPENGL_gl_LIBRARY}"
		)
endif()

if(UNIX)
	if (GLUT_FOUND AND OPENGL_gl_LIBRARY AND OPENGL_glu_LIBRARY AND GLUT_glut_LIBRARY)
		set(CMAKE_MRPT_HAS_OPENGL_GLUT 1)
		set(CMAKE_MRPT_HAS_OPENGL_GLUT_SYSTEM 1)

		add_library(imp_glut INTERFACE IMPORTED)
		set_target_properties(imp_glut
			PROPERTIES
			INTERFACE_INCLUDE_DIRECTORIES "${GLUT_INCLUDE_DIR}"
			INTERFACE_LINK_LIBRARIES "${OPENGL_glu_LIBRARY};${GLUT_glut_LIBRARY}"
			)

		set(CMAKE_MRPT_HAS_GLUT 1)
		set(CMAKE_MRPT_HAS_GLUT_SYSTEM 1)
	else()
		message(STATUS "**Warning**: OpenGL and/or GLUT not found! OpenGL capabilities will be disabled.")
		message(STATUS "              	OPENGL_gl_LIBRARY: ${OPENGL_gl_LIBRARY}")
		message(STATUS "              	OPENGL_glu_LIBRARY: ${OPENGL_glu_LIBRARY}")
		message(STATUS "              	GLUT_glut_LIBRARY: ${GLUT_glut_LIBRARY}")
		message(STATUS "              	OPENGL_INCLUDE_DIR: ${OPENGL_INCLUDE_DIR}")

		set(CMAKE_MRPT_HAS_OPENGL_GLUT 0)
		set(CMAKE_MRPT_HAS_OPENGL_GLUT_SYSTEM 0)

		set(CMAKE_MRPT_HAS_GLUT 0)
		set(CMAKE_MRPT_HAS_GLUT_SYSTEM 0)
	endif(GLUT_FOUND AND OPENGL_gl_LIBRARY AND OPENGL_glu_LIBRARY AND GLUT_glut_LIBRARY)
else()
	# Windows: embedded source versions:
	set(CMAKE_MRPT_HAS_OPENGL_GLUT 1)
	set(CMAKE_MRPT_HAS_OPENGL_GLUT_SYSTEM 1)

	set(CMAKE_MRPT_HAS_GLUT 1)
	set(CMAKE_MRPT_HAS_GLUT_SYSTEM 0)
endif(UNIX)

if(NOT MSVC)
	# GL, GLU, glut:
	if(CMAKE_MRPT_HAS_OPENGL_GLUT)
		set(MRPT_OPENGL_LIBS
			imp_opengl
			imp_glut
			)
	endif()
endif()
