#---------------------------------------------
# Macro declared in "DeclareMRPTLib.cmake":
#---------------------------------------------
define_mrpt_lib(
	# Lib name
	apps
	# Dependencies
	mrpt-bayes
	mrpt-hwdrivers
	mrpt-gui
	mrpt-obs
	mrpt-slam
	mrpt-graphslam
	mrpt-hmtslam
	mrpt-tclap
	)

if(NOT BUILD_mrpt-apps)
  return()
endif()

target_link_libraries(apps PRIVATE Threads::Threads)

