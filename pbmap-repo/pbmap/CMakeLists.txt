
# This library is especial: it *really* needs PCL, so don't even declare it if we don't have PCL:
if(NOT CMAKE_MRPT_HAS_PCL)
	set(BUILD_mrpt-pbmap "OFF" CACHE BOOL "Build the library mrpt-pbmap" FORCE)
	return()
endif()

find_package(VTK QUIET)
if(NOT VTK_FOUND)
	message(STATUS "pbmap: Requires PCL (found) & VTK (not found), disabling build.")
	set(BUILD_mrpt-pbmap "OFF" CACHE BOOL "Build the library mrpt-pbmap" FORCE)
	return()
endif()

#---------------------------------------------
# Macro declared in "DeclareMRPTLib.cmake":
#---------------------------------------------
define_mrpt_lib(
	# Lib name
	pbmap
	# Dependencies
	mrpt-graphs
	)

if(BUILD_mrpt-pbmap)
	GET_PROPERTY(PCL_LIBRARIES GLOBAL PROPERTY PCL_LIBRARIES)
	if(NOT TARGET imp_pcl)
		message(ERROR "Target impl_pcl should be defined!")
	endif()
	target_link_libraries(pbmap PRIVATE imp_pcl  ${PCL_LIBRARIES})
endif()
