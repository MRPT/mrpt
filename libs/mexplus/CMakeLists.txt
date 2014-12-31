#---------------------------------------------
# Macro declared in "DeclareMRPTLib.cmake":
#---------------------------------------------
define_mrpt_lib_header_only(
	# Lib name
        mexplus
	# Dependencies
        # None
	)

IF(BUILD_mrpt-mexplus)
    INCLUDE_DIRECTORIES("${MRPT_LIBS_ROOT}/mexplus/include")
    message("mexplus include directories is: ${MRPT_LIBS_ROOT}/mexplus/include")
ENDIF(BUILD_mrpt-mexplus)
