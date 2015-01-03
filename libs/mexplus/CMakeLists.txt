#---------------------------------------------
# Macro declared in "DeclareMRPTLib.cmake":
#---------------------------------------------
# Set OFF as default value for BUILD_mrpt-mexplus
set(DEFAULT_BUILD_mrpt-mexplus "OFF")

define_mrpt_lib_header_only(
	# Lib name
        mexplus
	# Dependencies
        # None
	)

IF(BUILD_mrpt-mexplus)

ENDIF(BUILD_mrpt-mexplus)
