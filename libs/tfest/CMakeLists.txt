
SET(TFESTDIR "${MRPT_SOURCE_DIR}/libs/tfest")

LIST(APPEND tfest_EXTRA_SRCS_NAME   
	"bckwrds_compatibility"
	"bckwrds_compatibility"
	"bckwrds_compatibility"
	)

#---------------------------------------------
# Macro declared in "DeclareMRPTLib.cmake":
#---------------------------------------------
define_mrpt_lib(
	# Lib name
	tfest
	# Dependencies
	mrpt-base
	)

IF(BUILD_mrpt-tfest)
	# Declare "meta-lib" for backwards compatibility:
	define_mrpt_metalib(
		# Lib name
		scanmatching
		# Deps:
		mrpt-tfest
		# Deps of the deps:
		mrpt-base
		)

ENDIF(BUILD_mrpt-tfest)
