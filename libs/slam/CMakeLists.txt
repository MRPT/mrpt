# Lists of directories with source files:
#  See "DeclareMRPTLib.cmake" for explanations
# -------------------------------------------------

LIST(APPEND slam_EXTRA_SRCS
	"${MRPT_LIBS_ROOT}/slam/src/maps/*.cpp"
	"${MRPT_LIBS_ROOT}/slam/src/slam/*.cpp"
	"${MRPT_LIBS_ROOT}/slam/include/mrpt/slam/*.h"
	)
LIST(APPEND slam_EXTRA_SRCS_NAME
	"slam" "slam" "slam-headers")

#---------------------------------------------
# Macro declared in "DeclareMRPTLib.cmake":
#---------------------------------------------
define_mrpt_lib(
	# Lib name
	slam
	# Dependencies
	mrpt-vision
	mrpt-maps
	)

IF(BUILD_mrpt-slam)

ENDIF()
