#  See "DeclareMRPTLib.cmake"

list(APPEND system_EXTRA_SRCS		"${MRPT_LIBS_ROOT}/system/*.cpp" "${MRPT_LIBS_ROOT}/system/*.h")
list(APPEND system_EXTRA_SRCS_NAME 	"system" "system")

define_mrpt_lib(
	system 		# Lib name
	# Dependencies:
	mrpt-typemeta
	mrpt-containers
	)

# extra dependencies required by unit tests in this module:
set_property(GLOBAL PROPERTY mrpt_system_UNIT_TEST_EXTRA_DEPS mrpt-random)

if(BUILD_mrpt-system)
	if(WIN32)
		target_link_libraries(mrpt-system PRIVATE Dbghelp) # For mrpt::system::getCallStackBackTrace()
	else()
		target_link_libraries(mrpt-system PRIVATE ${CMAKE_DL_LIBS}) # For mrpt::system::getCallStackBackTrace()
	endif()
endif()
