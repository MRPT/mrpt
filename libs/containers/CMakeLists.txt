#---------------------------------------------
# Macro declared in "DeclareMRPTLib.cmake":
#---------------------------------------------
define_mrpt_lib(
	# Lib name
	containers
	# Dependencies
	mrpt-core
	)

# extra dependencies required by unit tests in this module:
set_property(GLOBAL PROPERTY mrpt_containers_UNIT_TEST_EXTRA_DEPS mrpt-random mrpt-poses)

if(BUILD_mrpt-containers)

	target_link_libraries(containers PRIVATE ${LIBYAMLCPP_LIBRARIES})
	target_include_directories(containers PRIVATE ${LIBYAMLCPP_INCLUDE_DIRS})

	if (TARGET mrpt_libfyaml)
		target_link_libraries(containers PRIVATE mrpt_libfyaml)
	endif()

endif( )
