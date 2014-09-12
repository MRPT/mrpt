
# This library is especial: it *really* needs PCL, so don't even declare it if we don't have PCL:
IF(CMAKE_MRPT_HAS_PCL)

	#---------------------------------------------
	# Macro declared in "DeclareMRPTLib.cmake":
	#---------------------------------------------
	define_mrpt_lib(
		# Lib name
		pbmap
		# Dependencies
		mrpt-graphs
		mrpt-base
		)

	target_link_libraries(mrpt-pbmap ${PCL_LIBRARIES})

ELSE(CMAKE_MRPT_HAS_PCL)
	SET(BUILD_mrpt-pbmap "OFF" CACHE BOOL "Build the library mrpt-pbmap" FORCE)
ENDIF(CMAKE_MRPT_HAS_PCL)

