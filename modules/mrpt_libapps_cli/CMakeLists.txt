#---------------------------------------------
# Macro declared in "DeclareMRPTLib.cmake":
#---------------------------------------------
define_mrpt_lib(
	# Lib name
	apps-cli
	# Dependencies
	mrpt-hwdrivers
	mrpt-slam
	mrpt-tclap
	mrpt-topography
	)

if(NOT BUILD_mrpt-apps-cli)
  return()
endif()

target_link_libraries(apps-cli PRIVATE Threads::Threads)
