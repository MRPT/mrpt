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

if(NOT BUILD_mrpt-comms)
  return()
endif()

target_link_libraries(comms PRIVATE Threads::Threads)

if(CMAKE_MRPT_HAS_FTDI_SYSTEM)
    target_link_libraries(comms PRIVATE imp_ftdi)
endif()
if (MINGW)
	target_link_libraries(comms PRIVATE WS2_32)
endif()
