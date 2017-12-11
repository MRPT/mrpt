# extra dependencies required by unit tests in this module:
set_property(GLOBAL PROPERTY mrpt_serialization_UNIT_TEST_EXTRA_DEPS mrpt-io)

#---------------------------------------------
# Macro declared in "DeclareMRPTLib.cmake":
#---------------------------------------------
define_mrpt_lib(
	# Lib name
	serialization
	# Dependencies
	mrpt-typemeta
	mrpt-rtti
	mrpt-core
	)

IF(BUILD_mrpt-serialization)

ENDIF( )
