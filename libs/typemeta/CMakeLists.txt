#---------------------------------------------
# Macro declared in "DeclareMRPTLib.cmake":
#---------------------------------------------
define_mrpt_lib_header_only(
	# Lib name
	typemeta
	# Dependencies
	)

if(BUILD_mrpt-typemeta)
	# Enforce C++17 in all dependent projects:
	mrpt_lib_target_requires_cpp17(mrpt-typemeta)

endif( )
