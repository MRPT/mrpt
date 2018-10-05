# extra dependencies required by unit tests in this module:
set_property(GLOBAL PROPERTY mrpt_obs_UNIT_TEST_EXTRA_DEPS mrpt-maps)

# Lists of directories with source files:
#  See "DeclareMRPTLib.cmake" for explanations
# -------------------------------------------------

list(APPEND obs_EXTRA_SRCS
	"${MRPT_LIBS_ROOT}/obs/src/*.cpp"
	"${MRPT_LIBS_ROOT}/obs/include/mrpt/obs/*.h"
	"${MRPT_LIBS_ROOT}/obs/include/mrpt/maps/*.h"
	"${MRPT_LIBS_ROOT}/obs/include/mrpt/slam/*.h"
	)

list(APPEND obs_EXTRA_SRCS_NAME
	"obs sources"
	"obs headers"
	"maps headers"
	"back-compat-hdrs")

# Minimal XMLParser:
include_directories("${MRPT_LIBS_ROOT}/db/src/")  # Notice the "src", not "include": these are private hdrs
# No need to compile XML sources here (already in mrpt-db), will be exported and linked from mrpt-db, 
# even if their interface is in a private .h file.

#---------------------------------------------
# Macro declared in "DeclareMRPTLib.cmake":
#---------------------------------------------
define_mrpt_lib(
	# Lib name
	obs
	# Dependencies
	mrpt-opengl
	mrpt-poses
	mrpt-db # xmlparser
	mrpt-tfest
	)

if(BUILD_mrpt-obs)
endif(BUILD_mrpt-obs)
