
# Lists of directories with source files:
#  See "DeclareMRPTLib.cmake" for explanations
# -------------------------------------------------


# Embedded freeglut? Built-in into mrpt-opengl, and enable DLL-exporting it to other client libs.
IF(CMAKE_MRPT_HAS_OPENGL_GLUT AND NOT CMAKE_MRPT_HAS_GLUT_SYSTEM)
	LIST(APPEND opengl_EXTRA_SRCS		"${MRPT_LIBS_ROOT}/opengl/src/freeglut-2.4.0/*.c" "${MRPT_LIBS_ROOT}/opengl/src/freeglut-2.4.0/*.h")
	LIST(APPEND opengl_EXTRA_SRCS_NAME 	"freeglut" "freeglut")
	#ADD_DEFINITIONS(/DFREEGLUT_STATIC)
	ADD_DEFINITIONS(/DFREEGLUT_EXPORTS)
	INCLUDE_DIRECTORIES("${MRPT_LIBS_ROOT}/opengl/include/otherlibs/freeglut/")
ENDIF(CMAKE_MRPT_HAS_OPENGL_GLUT AND NOT CMAKE_MRPT_HAS_GLUT_SYSTEM)

# Embedded version of lib3ds: We have it embedded because the API
#  changes between versions and it'd be a mess between Lin/Win builds...
#  even worse since lib3ds does NOT have any VERSION #define...
LIST(APPEND opengl_EXTRA_SRCS		"${MRPT_LIBS_ROOT}/opengl/src/lib3ds/*.c" "${MRPT_LIBS_ROOT}/opengl/src/lib3ds/*.h")
LIST(APPEND opengl_EXTRA_SRCS_NAME 	"lib3ds" "lib3ds")
INCLUDE_DIRECTORIES("${MRPT_SOURCE_DIR}/libs/opengl/src/")

# Embedded version of Glew:
IF(WIN32)
	LIST(APPEND opengl_EXTRA_SRCS		"${MRPT_SOURCE_DIR}/libs/opengl/src/glew/src/*.c" "${MRPT_SOURCE_DIR}/libs/opengl/src/glew/include/*.h")
	LIST(APPEND opengl_EXTRA_SRCS_NAME 	"glew" "glew")
	INCLUDE_DIRECTORIES("${MRPT_SOURCE_DIR}/libs/opengl/src/glew/include")
ENDIF(WIN32)

# Glext headers only (for Windows)
INCLUDE_DIRECTORIES("${MRPT_SOURCE_DIR}/libs/opengl/src/glext")


#---------------------------------------------
# Macro declared in "DeclareMRPTLib.cmake":
#---------------------------------------------
define_mrpt_lib(
	# Lib name
	opengl 		
	# Dependencies: 
	mrpt-base		
	)
	
# Ignore precompiled headers in some sources:
IF(MRPT_ENABLE_PRECOMPILED_HDRS)
	IF (MSVC)
		set_source_files_properties(
			${lib3ds_FILES}
			${freeglut_FILES}
			${glew_FILES}
			PROPERTIES COMPILE_FLAGS "/Y-"
			)
		# Params for glew:	
		set_source_files_properties(${glew_FILES} PROPERTIES COMPILE_FLAGS "/Y- /DGLEW_STATIC")
ENDIF (MSVC)
ENDIF(MRPT_ENABLE_PRECOMPILED_HDRS)

