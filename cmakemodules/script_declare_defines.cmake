# ----------------------------------------------------------------------------
# Transform the variables MRPT_XXX="ON/OFF" to CMAKE_MRPT_XXX="1/0"
# ----------------------------------------------------------------------------
macro(CREATE_CMAKE_MRPT_DEFINE defName)
	if(${defName} MATCHES "ON")
		set(CMAKE_${defName} "1" CACHE INTERNAL "")
	else()
		set(CMAKE_${defName} "0" CACHE INTERNAL "")
	endif()
endmacro()

CREATE_CMAKE_MRPT_DEFINE(MRPT_HAS_BUMBLEBEE)
CREATE_CMAKE_MRPT_DEFINE(MRPT_HAS_ASIAN_FONTS)
CREATE_CMAKE_MRPT_DEFINE(CMAKE_MRPT_HAS_INOTIFY)
