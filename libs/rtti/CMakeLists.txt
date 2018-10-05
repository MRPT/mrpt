#---------------------------------------------
# Macro declared in "DeclareMRPTLib.cmake":
#---------------------------------------------
define_mrpt_lib(
	# Lib name
	rtti
	# Dependencies
	mrpt-core
	mrpt-typemeta
	)

# extra dependencies required by unit tests in this module:
#set_property(GLOBAL PROPERTY mrpt_rtti_UNIT_TEST_EXTRA_DEPS mrpt-random)

if(BUILD_mrpt-rtti)


endif( )
