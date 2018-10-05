
set(CMAKE_MRPT_HAS_JSONCPP 0)
set(CMAKE_MRPT_HAS_JSONCPP_SYSTEM 0)

option(DISABLE_JSONCPP "Forces NOT using JSONCPP, even if it could be found by CMake" "OFF")
mark_as_advanced(DISABLE_JSONCPP)

if(NOT DISABLE_JSONCPP)
	find_package(jsoncpp QUIET)
	if(jsoncpp_FOUND)
		# cmake imported target: "jsoncpp_lib"
		set(CMAKE_MRPT_HAS_JSONCPP 1)
		set(CMAKE_MRPT_HAS_JSONCPP_SYSTEM 1)
	endif()
endif()
