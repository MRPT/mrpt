#  See "DeclareMRPTLib.cmake"

list(APPEND expr_EXTRA_SRCS		"${MRPT_LIBS_ROOT}/expr/*.cpp" "${MRPT_LIBS_ROOT}/expr/*.h")
list(APPEND expr_EXTRA_SRCS_NAME 	"expr" "expr")

define_mrpt_lib(
	expr 		# Lib name
	# Dependencies:
	mrpt-system
	mrpt-core
	)

if(BUILD_mrpt-expr)
	# Don't export ALL symbols for the huge exprtk lib
	set_target_properties(mrpt-expr PROPERTIES WINDOWS_EXPORT_ALL_SYMBOLS 0)
endif()
