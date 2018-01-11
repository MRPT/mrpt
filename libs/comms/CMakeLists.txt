# extra dependencies required by unit tests in this module:
set_property(GLOBAL PROPERTY mrpt_comms_UNIT_TEST_EXTRA_DEPS mrpt-poses)

#---------------------------------------------
# Macro declared in "DeclareMRPTLib.cmake":
#---------------------------------------------
define_mrpt_lib(
	# Lib name
	comms
	# Dependencies
	mrpt-io
	)

IF(CMAKE_MRPT_HAS_FTDI_SYSTEM)
	TARGET_LINK_LIBRARIES(mrpt-comms PRIVATE ${FTDI_LIBS})
ENDIF()
