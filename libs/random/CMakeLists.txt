#  See "DeclareMRPTLib.cmake"
LIST(APPEND random_EXTRA_SRCS		"${MRPT_LIBS_ROOT}/random/*.cpp" "${MRPT_LIBS_ROOT}/random/*.h")
LIST(APPEND random_EXTRA_SRCS_NAME 	"random" "random")

define_mrpt_lib(
	random  # Lib name
	# Dependencies:
	mrpt-core
	)

IF(BUILD_mrpt-random)

ENDIF()
