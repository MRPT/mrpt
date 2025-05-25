
include(./cmake/script_gridmap_options.cmake REQUIRED) # Gridmap options
include(./cmake/script_octomap.cmake REQUIRED)     # Check for the octomap library

#---------------------------------------------
# Macro declared in "DeclareMRPTLib.cmake":
#---------------------------------------------
define_mrpt_lib(
	# Lib name
	maps
	# Dependencies:
	mrpt-obs
	mrpt-graphs
	)

if(BUILD_mrpt-maps)
	if (CMAKE_MRPT_HAS_OCTOMAP)
		if (NOT "${OCTOMAP_LIBRARY_DIRS}" STREQUAL "")
			link_directories("${OCTOMAP_LIBRARY_DIRS}")
		endif()
		if(TARGET "EP_octomap")
			add_dependencies(maps EP_octomap)
		endif()
		target_link_libraries(maps PRIVATE ${OCTOMAP_LIBRARIES})
	endif()

	target_link_libraries(maps PRIVATE ${MRPT_OPENGL_LIBS})


	# Windows only: the mxplus lib internally requires linking with opencv (sigh...)
	# Perhaps there is a better way to solve this, someday.
	if (MRPT_WITH_MATLAB_WRAPPER)
		target_link_libraries(maps PRIVATE imp_opencv)
	endif()
endif()
