# Lists of directories with source files:
#  See "DeclareMRPTLib.cmake" for explanations
# -------------------------------------------------

LIST(APPEND obs_EXTRA_SRCS
	"${MRPT_LIBS_ROOT}/obs/src/*.cpp"
	"${MRPT_LIBS_ROOT}/obs/include/mrpt/obs/*.h"
	"${MRPT_LIBS_ROOT}/obs/include/mrpt/maps/*.h"
	"${MRPT_LIBS_ROOT}/obs/include/mrpt/slam/*.h"
	)

LIST(APPEND obs_EXTRA_SRCS_NAME
	"obs sources"
	"obs headers"
	"maps headers"
	"back-compat-hdrs")

# Minimal XMLParser:
INCLUDE_DIRECTORIES("${MRPT_LIBS_ROOT}/db/src/")  # Notice the "src", not "include": these are private hdrs
IF (WIN32 AND BUILD_SHARED_LIBS)  # No need to reinclude these sources (already in mrpt-db) if building static libs or if in GCC
	LIST(APPEND obs_EXTRA_SRCS		"${MRPT_LIBS_ROOT}/db/src/xmlparser/*.cpp" "${MRPT_LIBS_ROOT}/db/src/xmlparser/*.h")
	LIST(APPEND obs_EXTRA_SRCS_NAME 	"db/xmlparser" "db/xmlparser")
	SET(OBS_HAS_LOCAL_XMLPARSER 1)
ENDIF()

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

IF(BUILD_mrpt-obs)
	IF(MRPT_ENABLE_PRECOMPILED_HDRS AND MSVC AND OBS_HAS_LOCAL_XMLPARSER)
			set_source_files_properties(${utils/xmlparser_FILES} PROPERTIES COMPILE_FLAGS "/Y-")
	ENDIF()
ENDIF(BUILD_mrpt-obs)
