# Check for the GL,GLUT libraries (Linux only, in Windows use the build in .h & .lib)
# ===================================================

option(DISABLE_OPENGL "Disable the OpenGL library" "OFF")
mark_as_advanced(DISABLE_OPENGL)

if (DISABLE_OPENGL)
	set(CMAKE_MRPT_HAS_OPENGL_GLUT 0)
	set(CMAKE_MRPT_HAS_OPENGL_GLUT_SYSTEM 0)

	set(CMAKE_MRPT_HAS_GLUT 0)
	set(CMAKE_MRPT_HAS_GLUT_SYSTEM 0)

else (DISABLE_OPENGL)

	find_package(OpenGL)
	if(OPENGL_FOUND AND "${OPENGL_INCLUDE_DIR}")
		include_directories("${OPENGL_INCLUDE_DIR}")
	endif(OPENGL_FOUND AND "${OPENGL_INCLUDE_DIR}")

	if(UNIX)
		find_package(GLUT)

		if (GLUT_FOUND AND OPENGL_gl_LIBRARY AND OPENGL_glu_LIBRARY AND GLUT_glut_LIBRARY)
			include_directories("${GLUT_INCLUDE_DIR}")
			set(CMAKE_MRPT_HAS_OPENGL_GLUT 1)
			set(CMAKE_MRPT_HAS_OPENGL_GLUT_SYSTEM 1)

			set(CMAKE_MRPT_HAS_GLUT 1)
			set(CMAKE_MRPT_HAS_GLUT_SYSTEM 1)
		else(GLUT_FOUND AND OPENGL_gl_LIBRARY AND OPENGL_glu_LIBRARY AND GLUT_glut_LIBRARY)
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
	else(UNIX)
		set(CMAKE_MRPT_HAS_OPENGL_GLUT 1)
		set(CMAKE_MRPT_HAS_OPENGL_GLUT_SYSTEM 1)

		set(CMAKE_MRPT_HAS_GLUT 1)
		set(CMAKE_MRPT_HAS_GLUT_SYSTEM 0)
	endif(UNIX)

	if(NOT MSVC)
		# GL, GLU, glut:
		if(CMAKE_MRPT_HAS_OPENGL_GLUT)
			#APPEND_MRPT_LIBS(${OPENGL_gl_LIBRARY} ${OPENGL_glu_LIBRARY} ${GLUT_glut_LIBRARY})
			set(MRPT_OPENGL_LIBS ${OPENGL_gl_LIBRARY} ${OPENGL_glu_LIBRARY} ${GLUT_glut_LIBRARY})
		endif(CMAKE_MRPT_HAS_OPENGL_GLUT)
	endif(NOT MSVC)
endif (DISABLE_OPENGL)

