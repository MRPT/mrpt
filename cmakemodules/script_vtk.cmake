# Check forVTK
# ===================================================
set(CMAKE_MRPT_HAS_VTK 0 PARENT_SCOPE)

option(DISABLE_VTK "Force not using VTK libs" "OFF")
mark_as_advanced(DISABLE_VTK)
if(NOT DISABLE_VTK)
	find_package(VTK QUIET)
	if (VTK_FOUND)
		set(CMAKE_MRPT_HAS_VTK 1  PARENT_SCOPE)
	endif()
endif()
