# Lists of directories with source files:
#  See "DeclareMRPTLib.cmake" for explanations
# -------------------------------------------------
LIST(APPEND maps_EXTRA_SRCS			"${MRPT_SOURCE_DIR}/libs/maps/src/maps/*.cpp" "${MRPT_SOURCE_DIR}/libs/maps/include/mrpt/maps/*.h" "${MRPT_SOURCE_DIR}/libs/maps/include/mrpt/slam/*.h")
LIST(APPEND maps_EXTRA_SRCS_NAME 	"maps" "maps" "back-compat-hdrs")

LIST(APPEND maps_EXTRA_SRCS			"${MRPT_SOURCE_DIR}/libs/maps/src/opengl/*.cpp" "${MRPT_SOURCE_DIR}/libs/maps/include/mrpt/opengl/*.h")
LIST(APPEND maps_EXTRA_SRCS_NAME 	"opengl" "opengl")

# Include OctoMap (by Kai M. Wurm and Armin Hornung, University of Freiburg, Copyright (C) 2009-2012)
# which is stored in <MRPT>/otherlibs/octomap
#
LIST(APPEND maps_EXTRA_SRCS			"${MRPT_SOURCE_DIR}/otherlibs/octomap/src/*.cpp" "${MRPT_SOURCE_DIR}/otherlibs/octomap/src/math/*.cpp")
LIST(APPEND maps_EXTRA_SRCS_NAME 	"octomap" "octomath")
#INCLUDE_DIRECTORIES("${MRPT_SOURCE_DIR}/otherlibs/octomap/include/")
# Headers are publicly exposed in <mrpt/otherlibs/octomap/*.h>

IF(CMAKE_MRPT_HAS_LIBLAS)
	LINK_DIRECTORIES(${LAS_LINK_DIRS})
	INCLUDE_DIRECTORIES(${LAS_INCLUDE_DIRS})
ENDIF()

#---------------------------------------------
# Macro declared in "DeclareMRPTLib.cmake":
#---------------------------------------------
define_mrpt_lib(
	# Lib name
	maps
	# Dependencies:
	mrpt-obs
	mrpt-graphs
	# Deps of deps:
	mrpt-opengl
	mrpt-base
	)

IF(BUILD_mrpt-maps)
	IF (CMAKE_MRPT_HAS_PCL)
		target_link_libraries(mrpt-maps ${PCL_LIBRARIES})
	ENDIF()

	IF(CMAKE_MRPT_HAS_LIBLAS)
		target_link_libraries(mrpt-maps ${LAS_LIBS})
	ENDIF()

	IF (CMAKE_MRPT_HAS_VTK)
		target_link_libraries(mrpt-maps ${VTK_LIBRARIES})
	ENDIF()

	# Ignore precompiled headers in some sources:
	IF(MRPT_ENABLE_PRECOMPILED_HDRS AND MSVC)
		set_source_files_properties(
			${octomap_FILES}
			${octomath_FILES}
			PROPERTIES COMPILE_FLAGS "/Y-"
			)
	ENDIF()

ENDIF()
