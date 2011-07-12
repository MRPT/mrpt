# Lists of directories with source files:
#  See "DeclareMRPTLib.cmake" for explanations
# -------------------------------------------------
LIST(APPEND maps_EXTRA_SRCS			"${MRPT_SOURCE_DIR}/libs/maps/src/maps/*.cpp" "${MRPT_SOURCE_DIR}/libs/maps/include/mrpt/slam/*.h")
LIST(APPEND maps_EXTRA_SRCS_NAME 	"maps" "maps")

LIST(APPEND maps_EXTRA_SRCS			"${MRPT_SOURCE_DIR}/libs/maps/src/opengl/*.cpp" "${MRPT_SOURCE_DIR}/libs/maps/include/mrpt/opengl/*.h")
LIST(APPEND maps_EXTRA_SRCS_NAME 	"opengl" "opengl")

# for ann:
INCLUDE_DIRECTORIES("${MRPT_LIBS_ROOT}/base/include/mrpt/otherlibs/")
#LIST(APPEND maps_EXTRA_SRCS			"${MRPT_SOURCE_DIR}/libs/maps/src/maps/ann/*.cpp" "${MRPT_SOURCE_DIR}/libs/maps/include/mrpt/otherlibs/ann/*.h")
#LIST(APPEND maps_EXTRA_SRCS_NAME 	"ann" "ann")
#INCLUDE_DIRECTORIES("${MRPT_SOURCE_DIR}/libs/maps/include/mrpt/otherlibs/")


#---------------------------------------------
# Macro declared in "DeclareMRPTLib.cmake":
#---------------------------------------------
define_mrpt_lib(
	# Lib name
	maps
	# Dependencies:
	mrpt-obs
	# Deps of deps:
	mrpt-opengl
	mrpt-base
	)

IF (CMAKE_MRPT_HAS_PCL)
        target_link_libraries(mrpt-maps ${PCL_LIBRARIES})
ENDIF(CMAKE_MRPT_HAS_PCL)


# Ignore precompiled headers in some sources:
IF(MRPT_ENABLE_PRECOMPILED_HDRS AND MSVC)
	#...
ENDIF(MRPT_ENABLE_PRECOMPILED_HDRS AND MSVC)

