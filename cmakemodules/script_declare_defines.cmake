# ----------------------------------------------------------------------------
# Transform the variables MRPT_XXX="ON/OFF" to CMAKE_MRPT_XXX="1/0"
# ----------------------------------------------------------------------------
macro(CREATE_CMAKE_MRPT_DEFINE defName)
	if(${defName} MATCHES "ON")
		set(CMAKE_${defName} "1")
	else(${defName} MATCHES "ON")
		set(CMAKE_${defName} "0")
	endif(${defName} MATCHES "ON")
endmacro(CREATE_CMAKE_MRPT_DEFINE)

CREATE_CMAKE_MRPT_DEFINE(MRPT_ALWAYS_CHECKS_DEBUG)
CREATE_CMAKE_MRPT_DEFINE(MRPT_ALWAYS_CHECKS_DEBUG_MATRICES)
CREATE_CMAKE_MRPT_DEFINE(MRPT_HAS_BUMBLEBEE)
CREATE_CMAKE_MRPT_DEFINE(MRPT_HAS_SVS)
CREATE_CMAKE_MRPT_DEFINE(MRPT_HAS_ASIAN_FONTS)
CREATE_CMAKE_MRPT_DEFINE(CMAKE_MRPT_HAS_INOTIFY)
