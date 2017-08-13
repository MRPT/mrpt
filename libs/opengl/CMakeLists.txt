
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
ENDIF()

# Embedded version of Glew:
IF(WIN32)
	LIST(APPEND opengl_EXTRA_SRCS		"${MRPT_SOURCE_DIR}/libs/opengl/src/glew/src/*.c" "${MRPT_SOURCE_DIR}/libs/opengl/src/glew/include/*.h")
	LIST(APPEND opengl_EXTRA_SRCS_NAME 	"glew" "glew")
	INCLUDE_DIRECTORIES("${MRPT_SOURCE_DIR}/libs/opengl/src/glew/include")
ENDIF()

# Glext headers only (for Windows)
INCLUDE_DIRECTORIES("${MRPT_SOURCE_DIR}/libs/opengl/src/glext")

# Assimp:
IF (CMAKE_MRPT_HAS_ASSIMP)
	# ASSIMP_ROOT_DIR - the root directory where the installation can be found
	# ASSIMP_CXX_FLAGS - extra flags for compilation
	# ASSIMP_LINK_FLAGS - extra flags for linking
	# ASSIMP_INCLUDE_DIRS - include directories
	# ASSIMP_LIBRARY_DIRS - link directories
	# ASSIMP_LIBRARIES - libraries to link plugins with
	IF (NOT ${ASSIMP_CXX_FLAGS} STREQUAL "")
		ADD_DEFINITIONS("${ASSIMP_CXX_FLAGS}")
	ENDIF()
	IF (NOT "${ASSIMP_INCLUDE_DIRS}" STREQUAL "")
		INCLUDE_DIRECTORIES("${ASSIMP_INCLUDE_DIRS}")
	ENDIF()
	IF(NOT "${ASSIMP_VERSION}" STREQUAL "")
		string(SUBSTRING "${ASSIMP_VERSION}" 0 1 MRPT_ASSIMP_VERSION_MAJOR)
		ADD_DEFINITIONS(-DMRPT_ASSIMP_VERSION_MAJOR=${MRPT_ASSIMP_VERSION_MAJOR})
	ENDIF()
ENDIF ()

#---------------------------------------------
# Macro declared in "DeclareMRPTLib.cmake":
#---------------------------------------------
define_mrpt_lib(
	# Lib name
	opengl
	# Dependencies:
	mrpt-base
	)

IF(BUILD_mrpt-opengl)
	# Ignore precompiled headers in some sources:
	IF(MRPT_ENABLE_PRECOMPILED_HDRS)
		IF (MSVC)
			set_source_files_properties(
				${freeglut_FILES}
				${glew_FILES}
				PROPERTIES COMPILE_FLAGS "/Y-"
				)
			# Params for glew:
			set_source_files_properties(${glew_FILES} PROPERTIES COMPILE_FLAGS "/Y- /DGLEW_STATIC")
	ENDIF ()
	ENDIF()

	# Link against gl, glu, glut:
	TARGET_LINK_LIBRARIES(mrpt-opengl ${MRPT_OPENGL_LIBS} ${MRPT_3DS_LIBS})

	# Link against assimp:
	IF (CMAKE_MRPT_HAS_ASSIMP)
		IF(TARGET "EP_assimp")
			ADD_DEPENDENCIES(mrpt-opengl EP_assimp)
		ENDIF()
		TARGET_LINK_LIBRARIES(mrpt-opengl ${ASSIMP_LIBRARIES})
	ENDIF ()
ENDIF()
