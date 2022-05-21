# Mandatory modules for this module:
set(lst_modules
	mrpt-hwdrivers
	mrpt-gui
	mrpt-slam
	mrpt-tclap
	mrpt-topography
)

# Optional modules:
if (BUILD_mrpt-hmtslam)
	list(APPEND lst_modules mrpt-hmtslam)
endif()
if (BUILD_mrpt-graphslam)
	list(APPEND lst_modules mrpt-graphslam)
endif()

#---------------------------------------------
# Macro declared in "DeclareMRPTLib.cmake":
#---------------------------------------------
define_mrpt_lib(
	# Lib name
	apps
	# Dependencies
	${lst_modules}
	)

if(NOT BUILD_mrpt-apps)
  return()
endif()

target_link_libraries(apps PRIVATE Threads::Threads)
