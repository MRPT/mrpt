
# This library is especial: it *really* needs PCL, so don't even declare it if we don't have PCL:
if(CMAKE_MRPT_HAS_PCL)

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
		target_link_libraries(mrpt-pbmap PRIVATE ${PCL_LIBRARIES})
	endif()

else(CMAKE_MRPT_HAS_PCL)
	set(BUILD_mrpt-pbmap "OFF" CACHE BOOL "Build the library mrpt-pbmap" FORCE)
endif()
