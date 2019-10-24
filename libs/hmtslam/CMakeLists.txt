# Macro declared in "DeclareMRPTLib.cmake":
define_mrpt_lib(
	# Lib name
	hmtslam
	# Dependencies
	mrpt-graphslam
	)

if(BUILD_mrpt-hmtslam)
    target_link_libraries(hmtslam PRIVATE Threads::Threads)
endif()
