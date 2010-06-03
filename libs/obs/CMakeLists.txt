# Lists of directories with source files:
#  See "DeclareMRPTLib.cmake" for explanations
# -------------------------------------------------

LIST(APPEND obs_EXTRA_SRCS
	"${MRPT_LIBS_ROOT}/obs/src/*.cpp" 
	"${MRPT_LIBS_ROOT}/obs/include/mrpt/slam/*.h")
	
LIST(APPEND obs_EXTRA_SRCS_NAME 
	"obs sources" 
	"obs headers")

#---------------------------------------------
# Macro declared in "DeclareMRPTLib.cmake":
#---------------------------------------------
define_mrpt_lib(
	# Lib name
	obs
	# Dependencies
	mrpt-opengl
	mrpt-base
	)
