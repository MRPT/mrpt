#  See "DeclareMRPTLib.cmake"
list(APPEND random_EXTRA_SRCS		"${MRPT_LIBS_ROOT}/random/*.cpp" "${MRPT_LIBS_ROOT}/random/*.h")
list(APPEND random_EXTRA_SRCS_NAME 	"random" "random")

set_property(GLOBAL PROPERTY mrpt_random_UNIT_TEST_EXTRA_DEPS mrpt-math)

define_mrpt_lib(
	random  # Lib name
	# Dependencies:
	)

if(BUILD_mrpt-random)
	# Enforce C++17 in all dependent projects:
	mrpt_lib_target_requires_cpp17(random)
endif()
