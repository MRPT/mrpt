
# Lists of directories with source files:
#  See "DeclareMRPTLib.cmake" for explanations
# -------------------------------------------------


# Embedded freeglut? Built-in into mrpt-opengl, and enable DLL-exporting it to other client libs.
if(CMAKE_MRPT_HAS_OPENGL_GLUT AND NOT CMAKE_MRPT_HAS_GLUT_SYSTEM)
	list(APPEND opengl_EXTRA_SRCS		"${MRPT_LIBS_ROOT}/opengl/src/freeglut-2.4.0/*.c" "${MRPT_LIBS_ROOT}/opengl/src/freeglut-2.4.0/*.h")
	list(APPEND opengl_EXTRA_SRCS_NAME 	"freeglut" "freeglut")
	#add_definitions(/DFREEGLUT_STATIC)
	add_definitions(/DFREEGLUT_EXPORTS)
	include_directories("${MRPT_LIBS_ROOT}/opengl/include/otherlibs/freeglut/")
endif()

# Embedded version of Glew:
if(WIN32)
	list(APPEND opengl_EXTRA_SRCS		"${MRPT_SOURCE_DIR}/libs/opengl/src/glew/src/*.c" "${MRPT_SOURCE_DIR}/libs/opengl/src/glew/include/*.h")
	list(APPEND opengl_EXTRA_SRCS_NAME 	"glew" "glew")
	include_directories("${MRPT_SOURCE_DIR}/libs/opengl/src/glew/include")
endif()

# Glext headers only (for Windows)
include_directories(${CVD_DIR})

include_directories("${MRPT_SOURCE_DIR}/libs/opengl/src/glext")

# Assimp:
if (CMAKE_MRPT_HAS_ASSIMP)
	# ASSIMP_ROOT_DIR - the root directory where the installation can be found
	# ASSIMP_CXX_FLAGS - extra flags for compilation
	# ASSIMP_LINK_FLAGS - extra flags for linking
	# ASSIMP_INCLUDE_DIRS - include directories
	# ASSIMP_LIBRARY_DIRS - link directories
	# ASSIMP_LIBRARIES - libraries to link plugins with
	if (NOT ${ASSIMP_CXX_FLAGS} STREQUAL "")
		add_definitions("${ASSIMP_CXX_FLAGS}")
	endif()
	if (NOT "${ASSIMP_INCLUDE_DIRS}" STREQUAL "")
		include_directories("${ASSIMP_INCLUDE_DIRS}")
	endif()
	if(NOT "${ASSIMP_VERSION}" STREQUAL "")
		string(SUBSTRING "${ASSIMP_VERSION}" 0 1 MRPT_ASSIMP_VERSION_MAJOR)
		add_definitions(-DMRPT_ASSIMP_VERSION_MAJOR=${MRPT_ASSIMP_VERSION_MAJOR})
	endif()
endif ()

#---------------------------------------------
# Macro declared in "DeclareMRPTLib.cmake":
#---------------------------------------------
define_mrpt_lib(
	# Lib name
	opengl
	# Dependencies:
	mrpt-poses
	mrpt-img
	)

if(BUILD_mrpt-opengl)
	add_dependencies(mrpt-opengl libcvd)
	include_directories(${CVD_INCLUDE_DIRS})

	# Ignore precompiled headers in some sources:
	if(MRPT_ENABLE_PRECOMPILED_HDRS)
		if (MSVC)
			set_source_files_properties(
				${freeglut_FILES}
				${glew_FILES}
				PROPERTIES COMPILE_FLAGS "/Y-"
				)
			# Params for glew:
			set_source_files_properties(${glew_FILES} PROPERTIES COMPILE_FLAGS "/Y- /DGLEW_STATIC")
	endif ()
	endif()

	if (MSVC)
		# Avoid duplicated export symbol warnings in MSVC SRC glut version
		set_target_properties(mrpt-opengl PROPERTIES LINK_FLAGS "/ignore:4197")
	endif()

	# Link against gl, glu, glut:
	target_link_libraries(mrpt-opengl PUBLIC ${MRPT_OPENGL_LIBS} ${MRPT_3DS_LIBS} ${CVD_LIB})

	# Link against assimp:
	if (CMAKE_MRPT_HAS_ASSIMP)
		if(TARGET "EP_assimp")
			add_dependencies(mrpt-opengl EP_assimp)
		endif()
		target_link_libraries(mrpt-opengl PRIVATE ${ASSIMP_LIBRARIES})
	endif ()
endif()
