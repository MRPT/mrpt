#  See "DeclareMRPTLib.cmake"

LIST(APPEND system_EXTRA_SRCS		"${MRPT_LIBS_ROOT}/system/*.cpp" "${MRPT_LIBS_ROOT}/system/*.h")
LIST(APPEND system_EXTRA_SRCS_NAME 	"system" "system")

define_mrpt_lib(
	system 		# Lib name
	# Dependencies:
	mrpt-core
	)

# extra dependencies required by unit tests in this module:
set_property(GLOBAL PROPERTY mrpt_system_UNIT_TEST_EXTRA_DEPS mrpt-random)

IF(BUILD_mrpt-system)
	IF(WIN32)
		target_link_libraries(mrpt-system PRIVATE Dbghelp) # For mrpt::system::getCallStackBackTrace()
	ELSE()
		target_link_libraries(mrpt-system PRIVATE ${CMAKE_DL_LIBS}) # For mrpt::system::getCallStackBackTrace()
	ENDIF()
ENDIF()
