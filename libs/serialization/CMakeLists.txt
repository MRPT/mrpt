# Extra dependencies required by unit tests in this module:
# Include jsoncpp only if present (optional)
if (CMAKE_MRPT_HAS_JSONCPP)
	if(TARGET "jsoncpp_lib")
		set(tst_json_dep "jsoncpp_lib")
	else()
		set(tst_json_dep "jsoncpp_lib_static")
	endif()
else()
	set(tst_json_dep "")
endif()
# define those deps:
set_property(GLOBAL PROPERTY mrpt_serialization_UNIT_TEST_EXTRA_DEPS mrpt-io mrpt-poses ${tst_json_dep})

#---------------------------------------------
# Macro declared in "DeclareMRPTLib.cmake":
#---------------------------------------------
define_mrpt_lib(
	# Lib name
	serialization
	# Dependencies
	mrpt-rtti
	)

if(BUILD_mrpt-serialization)
	target_link_libraries(mrpt-serialization PRIVATE ${tst_json_dep})
endif( )
