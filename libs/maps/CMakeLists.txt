# Lists of directories with source files:
#  See "DeclareMRPTLib.cmake" for explanations
# -------------------------------------------------
LIST(APPEND maps_EXTRA_SRCS			"${MRPT_SOURCE_DIR}/libs/maps/src/maps/*.cpp" "${MRPT_SOURCE_DIR}/libs/maps/include/mrpt/slam/*.h")
LIST(APPEND maps_EXTRA_SRCS_NAME 	"maps" "maps")

LIST(APPEND maps_EXTRA_SRCS			"${MRPT_SOURCE_DIR}/libs/maps/src/opengl/*.cpp" "${MRPT_SOURCE_DIR}/libs/maps/include/mrpt/opengl/*.h")
LIST(APPEND maps_EXTRA_SRCS_NAME 	"opengl" "opengl")

LIST(APPEND maps_EXTRA_SRCS			"${MRPT_SOURCE_DIR}/libs/maps/src/maps/ann/*.cpp" "${MRPT_SOURCE_DIR}/libs/maps/include/mrpt/otherlibs/ann/*.h")
LIST(APPEND maps_EXTRA_SRCS_NAME 	"ann" "ann")
INCLUDE_DIRECTORIES("${MRPT_SOURCE_DIR}/libs/maps/include/mrpt/otherlibs/")

#---------------------------------------------
# Macro declared in "DeclareMRPTLib.cmake":
#---------------------------------------------
define_mrpt_lib(
	# Lib name
	maps
	# Dependencies:
	mrpt-obs
	mrpt-opengl  # Dep of the ones above.
	mrpt-base  # Dep of the ones above.
	)

# Ignore precompiled headers in some sources:
IF(MRPT_ENABLE_PRECOMPILED_HDRS AND MSVC)
	set_source_files_properties(
		${ann_FILES}
		PROPERTIES COMPILE_FLAGS "/Y-"
		)
ENDIF(MRPT_ENABLE_PRECOMPILED_HDRS AND MSVC)

