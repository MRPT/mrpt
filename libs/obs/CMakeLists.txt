# extra dependencies required by unit tests in this module:
set_property(GLOBAL PROPERTY mrpt_obs_UNIT_TEST_EXTRA_DEPS mrpt-maps)

# Lists of directories with source files:
#  See "DeclareMRPTLib.cmake" for explanations
# -------------------------------------------------

list(APPEND obs_EXTRA_SRCS
	"${MRPT_LIBS_ROOT}/obs/src/*.cpp"
	"${MRPT_LIBS_ROOT}/obs/include/mrpt/obs/*.h"
	"${MRPT_LIBS_ROOT}/obs/include/mrpt/maps/*.h"
	)

list(APPEND obs_EXTRA_SRCS_NAME
	"obs sources"
	"obs headers"
	"maps headers"
	)

# Built-in libtinyxml2?
if(CMAKE_MRPT_HAS_TINYXML2 AND NOT CMAKE_MRPT_HAS_TINYXML2_SYSTEM)
	list(APPEND obs_EXTRA_SRCS
		"${MRPT_BINARY_DIR}/3rdparty/tinyxml2/*.cpp"
		"${MRPT_BINARY_DIR}/3rdparty/tinyxml2/*.h"
		)
	list(APPEND obs_EXTRA_SRCS_NAME
		"tinyxml2" "tinyxml2"
		)
	include_directories("${MRPT_BINARY_DIR}/3rdparty/tinyxml2/")
endif()

#---------------------------------------------
# Macro declared in "DeclareMRPTLib.cmake":
#---------------------------------------------
define_mrpt_lib(
	# Lib name
	obs
	# Dependencies
	mrpt-opengl
	mrpt-poses
	mrpt-tfest
	)

if(BUILD_mrpt-obs)
	# tinyxml2 as system lib:
	if (TARGET imp_tinyxml2)
		target_link_libraries(obs PRIVATE imp_tinyxml2)
	endif()
	#
	# Ignore precompiled headers in built-in tinyxml2 (for Windows, etc.):
	if(CMAKE_MRPT_HAS_TINYXML2 AND NOT CMAKE_MRPT_HAS_TINYXML2_SYSTEM)
		if(MRPT_ENABLE_PRECOMPILED_HDRS AND MSVC)
			set_source_files_properties(${tinyxml2_FILES} PROPERTIES COMPILE_FLAGS "/Y-")
		endif()
	endif()

	# Windows only: the mxplus lib internally requires linking with opencv (sigh...)
	# Update Sep 2019: mrpt-obs now also uses OpenCV directly, so let's just link against opencv anyway:
	target_link_libraries(obs PRIVATE imp_opencv)

endif(BUILD_mrpt-obs)
