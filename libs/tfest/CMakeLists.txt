
SET(TFESTDIR "${MRPT_SOURCE_DIR}/libs/tfest")

# Compatibility with MRPT <1.3.0
LIST(APPEND tfest_EXTRA_SRCS	      
	"${TFESTDIR}/include/mrpt/scanmatching/*.h" 
	"${TFESTDIR}/include/mrpt/scanmatching.h"
	"${TFESTDIR}/src/scanmatching_backwards_compat.cpp"
	)
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

ENDIF(BUILD_mrpt-tfest) 

