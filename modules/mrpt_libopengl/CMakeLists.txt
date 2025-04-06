
# Lists of directories with source files:
#  See "DeclareMRPTLib.cmake" for explanations
# -------------------------------------------------

include(${MRPT_SOURCE_DIR}/cmakemodules/script_assimp.cmake REQUIRED)      # Check for system assimp lib (3D models)
include(${MRPT_SOURCE_DIR}/cmakemodules/script_gl_glut.cmake REQUIRED)     # Check for the GL,GLUT libraries

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

option(MRPT_OPENGL_PROFILER OFF)

#---------------------------------------------
# Macro declared in "DeclareMRPTLib.cmake":
#---------------------------------------------
define_mrpt_lib(
	# Lib name
	opengl
	# Dependencies:
	mrpt-viz
	mrpt-poses
	mrpt-img
	)

if(NOT BUILD_mrpt-opengl)
  return()
endif()

if (MRPT_OPENGL_PROFILER)
	target_compile_definitions(opengl PRIVATE MRPT_OPENGL_PROFILER)
endif()

# https://developer.apple.com/macos/whats-new/#deprecationofopenglandopencl
if (APPLE)
	target_compile_definitions(opengl PRIVATE GL_SILENCE_DEPRECATION)
endif()

# Link against gl, glu, glut:
target_link_libraries(opengl PRIVATE
	${MRPT_OPENGL_LIBS}
	)
if (TARGET mrpt_glew) # Basically, for Windows
	target_link_libraries(opengl PRIVATE mrpt_glew)
endif()

if(CMAKE_MRPT_HAS_OPENCV)
	target_link_libraries(opengl PRIVATE imp_opencv)
endif()

# Link against assimp:
if (CMAKE_MRPT_HAS_ASSIMP)
	if(TARGET "EP_assimp")
		add_dependencies(opengl EP_assimp)
	endif()
	target_link_libraries(opengl PRIVATE ${ASSIMP_LIBRARIES})
endif ()
if (MINGW)
	target_link_libraries(opengl PRIVATE opengl32 GlU32)
endif()
if(CMAKE_MRPT_HAS_OPENCV)
	target_link_libraries(opengl PRIVATE imp_opencv)
endif()

