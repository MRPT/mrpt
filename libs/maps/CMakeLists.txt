# Lists of directories with source files:
#  See "DeclareMRPTLib.cmake" for explanations
# -------------------------------------------------
list(APPEND maps_EXTRA_SRCS			"${MRPT_SOURCE_DIR}/libs/maps/src/maps/*.cpp" "${MRPT_SOURCE_DIR}/libs/maps/include/mrpt/maps/*.h" "${MRPT_SOURCE_DIR}/libs/maps/include/mrpt/slam/*.h")
list(APPEND maps_EXTRA_SRCS_NAME 	"maps" "maps" "back-compat-hdrs")

list(APPEND maps_EXTRA_SRCS			"${MRPT_SOURCE_DIR}/libs/maps/src/opengl/*.cpp" "${MRPT_SOURCE_DIR}/libs/maps/include/mrpt/opengl/*.h")
list(APPEND maps_EXTRA_SRCS_NAME 	"opengl" "opengl")

list(APPEND maps_EXTRA_SRCS			"${MRPT_SOURCE_DIR}/libs/maps/src/obs/*.cpp" "${MRPT_SOURCE_DIR}/libs/maps/include/mrpt/obs/*.h")
list(APPEND maps_EXTRA_SRCS_NAME 	"obs" "obs")

if(CMAKE_MRPT_HAS_LIBLAS)
	link_directories(${LAS_LINK_DIRS})
	include_directories(${LAS_INCLUDE_DIRS})
endif()

#---------------------------------------------
# Macro declared in "DeclareMRPTLib.cmake":
#---------------------------------------------
define_mrpt_lib(
	# Lib name
	maps
	# Dependencies:
	mrpt-obs
	mrpt-graphs
	# Deps of deps:
	mrpt-opengl
	)

if(BUILD_mrpt-maps)
	if (CMAKE_MRPT_HAS_PCL)
		target_link_libraries(mrpt-maps PRIVATE ${PCL_LIBRARIES})
	endif()

	if(CMAKE_MRPT_HAS_LIBLAS)
		target_link_libraries(mrpt-maps PRIVATE ${LAS_LIBS})
	endif()

	if (CMAKE_MRPT_HAS_VTK)
        include(${VTK_USE_FILE})
		target_link_libraries(mrpt-maps PRIVATE ${VTK_LIBRARIES})
	endif()

	if (CMAKE_MRPT_HAS_OCTOMAP)
		if (NOT "${OCTOMAP_LIBRARY_DIRS}" STREQUAL "")
			link_directories("${OCTOMAP_LIBRARY_DIRS}")
		endif()
		if(TARGET "EP_octomap")
			add_dependencies(mrpt-maps EP_octomap)
		endif()
		target_link_libraries(mrpt-maps PUBLIC ${OCTOMAP_LIBRARIES})
	endif()

endif()
