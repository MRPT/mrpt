# Modified version of the TCLAP library. Search for JLBC in the sources to see the changes

include(${MRPT_SOURCE_DIR}/cmakemodules/script_libtclap.cmake REQUIRED)    # Check for system libtclap

#---------------------------------------------
# Macro declared in "DeclareMRPTLib.cmake":
#---------------------------------------------
define_mrpt_lib_header_only(
	# Lib name
	tclap
	# Dependencies
	)

if(BUILD_mrpt-tclap)

endif()
