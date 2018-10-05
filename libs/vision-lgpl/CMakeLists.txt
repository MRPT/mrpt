# Lists of directories with source files:
#  See "DeclareMRPTLib.cmake" for explanations
# -------------------------------------------------

#---------------------------------------------
# Macro declared in "DeclareMRPTLib.cmake":
#---------------------------------------------
if(MRPT_ALLOW_LGPLV3)
	define_mrpt_lib(
		# Lib name
		vision-lgpl
		# Dependencies:
		mrpt-vision
		)
endif()
