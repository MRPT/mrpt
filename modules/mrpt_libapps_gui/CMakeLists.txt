#---------------------------------------------
# Macro declared in "DeclareMRPTLib.cmake":
#---------------------------------------------
define_mrpt_lib(
	# Lib name
	apps-gui
	# Dependencies
	mrpt-apps-cli
	mrpt-gui
	)

if(NOT BUILD_mrpt-apps-gui)
  return()
endif()

target_link_libraries(apps-gui PRIVATE Threads::Threads)
