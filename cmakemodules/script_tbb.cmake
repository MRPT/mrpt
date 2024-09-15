
set(CMAKE_MRPT_HAS_TBB 0)
set(CMAKE_MRPT_HAS_TBB_SYSTEM 0)

option(DISABLE_TBB "Force not using TBB" "OFF")
mark_as_advanced(DISABLE_TBB)
if(DISABLE_TBB)
	return()
endif()

find_package(TBB QUIET)

if(TBB_FOUND)
	set(CMAKE_MRPT_HAS_TBB 1)
	set(CMAKE_MRPT_HAS_TBB_SYSTEM 1)
endif()
