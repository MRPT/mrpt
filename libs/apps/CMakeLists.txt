#---------------------------------------------
# Macro declared in "DeclareMRPTLib.cmake":
#---------------------------------------------
define_mrpt_lib(
	# Lib name
	apps
	# Dependencies
	mrpt-hwdrivers
	mrpt-gui
	mrpt-slam
	mrpt-tclap
	mrpt-topography
	)

if(NOT BUILD_mrpt-apps)
  return()
endif()

target_link_libraries(apps PRIVATE Threads::Threads)
