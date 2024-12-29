# Lists of directories with source files:
#  See "DeclareMRPTLib.cmake" for explanations
# -------------------------------------------------

#---------------------------------------------
# Macro declared in "DeclareMRPTLib.cmake":
#---------------------------------------------
define_mrpt_lib(
	# Lib name
	viz
	# Dependencies:
	mrpt-poses
	mrpt-img
	)

if(NOT BUILD_mrpt-viz)
  return()
endif()

