#  See "DeclareMRPTLib.cmake"
LIST(APPEND io_EXTRA_SRCS		"${MRPT_LIBS_ROOT}/io/*.cpp" "${MRPT_LIBS_ROOT}/io/*.h")
LIST(APPEND io_EXTRA_SRCS_NAME 	"io" "io")

define_mrpt_lib(
	io 		# Lib name
	# Dependencies:
	mrpt-core
	mrpt-system
	)

IF(BUILD_mrpt-io)

ENDIF()
