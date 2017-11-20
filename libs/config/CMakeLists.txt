#  See "DeclareMRPTLib.cmake"

LIST(APPEND config_EXTRA_SRCS		"${MRPT_LIBS_ROOT}/config/*.cpp" "${MRPT_LIBS_ROOT}/config/*.h")
LIST(APPEND config_EXTRA_SRCS_NAME 	"config" "config")

LIST(APPEND config_EXTRA_SRCS       "${MRPT_LIBS_ROOT}/config/src/simpleini/*.h")
LIST(APPEND config_EXTRA_SRCS_NAME  "config/simpleini")

define_mrpt_lib(
	config 		# Lib name
	# Dependencies:
	mrpt-core
	mrpt-system
	mrpt-expr
	)

IF(BUILD_mrpt-config)

ENDIF()
