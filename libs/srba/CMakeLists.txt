# Lists of directories with source files:
#  See "DeclareMRPTLib.cmake" for explanations
# -------------------------------------------------

LIST(APPEND srba_EXTRA_SRCS
	"${MRPT_LIBS_ROOT}/srba/include/mrpt/srba/impl/*.h"
	"${MRPT_LIBS_ROOT}/srba/include/mrpt/srba/models/*.h")
	
LIST(APPEND srba_EXTRA_SRCS_NAME 
	"Template implementations" 
	"Model implementations")

#---------------------------------------------
# Macro declared in "DeclareMRPTLib.cmake":
#---------------------------------------------
define_mrpt_lib_header_only(
	# Lib name
	srba
	# Dependencies
	mrpt-opengl
	mrpt-base
	)

IF(BUILD_mrpt-srba) 

ENDIF(BUILD_mrpt-srba) 

