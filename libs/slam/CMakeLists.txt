# Lists of directories with source files:
#  See "DeclareMRPTLib.cmake" for explanations
# -------------------------------------------------

list(APPEND slam_EXTRA_SRCS
	"${MRPT_LIBS_ROOT}/slam/src/maps/*.cpp"
	"${MRPT_LIBS_ROOT}/slam/src/slam/*.cpp"
	"${MRPT_LIBS_ROOT}/slam/include/mrpt/slam/*.h"
	)
list(APPEND slam_EXTRA_SRCS_NAME
	"slam" "slam" "slam-headers")

if(CMAKE_MRPT_HAS_TBB)
	set(tbb_dep TBB)
else()
	set(tbb_dep "")
endif()

#---------------------------------------------
# Macro declared in "DeclareMRPTLib.cmake":
#---------------------------------------------
define_mrpt_lib(
	# Lib name
	slam
	# Dependencies
	mrpt-vision
	mrpt-maps
	# Other imported targets:
	${tbb_dep} # find_package() lib name
	)

if(BUILD_mrpt-slam)

	if (CMAKE_MRPT_HAS_TBB)
		target_link_libraries(slam PUBLIC TBB::tbb)
	endif()
endif()
