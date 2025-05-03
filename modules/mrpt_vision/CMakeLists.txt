# Lists of directories with source files:
#  See "DeclareMRPTLib.cmake" for explanations
# -------------------------------------------------
list(APPEND vision_EXTRA_SRCS		"${MRPT_SOURCE_DIR}/libs/vision/src/maps/*.cpp" "${MRPT_SOURCE_DIR}/libs/vision/include/mrpt/maps/*.h" "${MRPT_SOURCE_DIR}/libs/vision/include/mrpt/slam/*.h")
list(APPEND vision_EXTRA_SRCS_NAME 	"maps" "maps" "maps-bcks-compat")

list(APPEND vision_EXTRA_SRCS		"${MRPT_SOURCE_DIR}/libs/vision/src/obs/*.cpp" "${MRPT_SOURCE_DIR}/libs/vision/include/mrpt/slam/CObservation*.h")
list(APPEND vision_EXTRA_SRCS_NAME 	"observations" "observations")

list(APPEND vision_EXTRA_SRCS		"${MRPT_SOURCE_DIR}/libs/vision/src/pnp/*.cpp" "${MRPT_SOURCE_DIR}/libs/vision/include/mrpt/vision/pnp_algos.h")
list(APPEND vision_EXTRA_SRCS_NAME 	"pnp" "pnp")

#---------------------------------------------
# Macro declared in "DeclareMRPTLib.cmake":
#---------------------------------------------
define_mrpt_lib(
	# Lib name
	vision
	# Dependencies:
	mrpt-obs
	)

