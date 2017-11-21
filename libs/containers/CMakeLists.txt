#---------------------------------------------
# Macro declared in "DeclareMRPTLib.cmake":
#---------------------------------------------
define_mrpt_lib_header_only(
	# Lib name
	containers
	# Dependencies
	#mrpt-core
	)

# extra dependencies required by unit tests in this module:
set_property(GLOBAL PROPERTY mrpt_containers_UNIT_TEST_EXTRA_DEPS mrpt-random)

IF(BUILD_mrpt-containers)


ENDIF( )
