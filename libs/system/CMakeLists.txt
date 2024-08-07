#  See "DeclareMRPTLib.cmake"

# Use C++17 filesystem lib:
find_package(Filesystem REQUIRED)

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
	target_link_libraries(system PRIVATE
	Threads::Threads
	CXX::Filesystem  # the C++>=17 std lib
	)

	# -ldl required by the plugin load mechanism loadPluginModule()
	if(UNIX)
		target_link_libraries(system PRIVATE ${CMAKE_DL_LIBS})
	endif()
endif()
