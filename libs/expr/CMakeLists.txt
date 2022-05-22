#  See "DeclareMRPTLib.cmake"

list(APPEND expr_EXTRA_SRCS		"${MRPT_LIBS_ROOT}/expr/*.cpp" "${MRPT_LIBS_ROOT}/expr/*.h")
list(APPEND expr_EXTRA_SRCS_NAME 	"expr" "expr")

define_mrpt_lib(
	expr 		# Lib name
	# Dependencies:
	mrpt-system
	)

if(BUILD_mrpt-expr)
	# Don't export ALL symbols for the huge exprtk lib
	set_target_properties(expr PROPERTIES WINDOWS_EXPORT_ALL_SYMBOLS 0)

	# Minimize debug info for this module:
	mrpt_reduced_debug_symbols(expr)
endif()
