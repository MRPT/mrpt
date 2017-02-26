# Macro declared in "DeclareMRPTLib.cmake":
define_mrpt_lib(
	# Lib name
	hmtslam
	# Dependencies
	mrpt-slam
	mrpt-graphslam
	# Deps of deps
	mrpt-graphs
	mrpt-vision
	mrpt-maps
	mrpt-base
	mrpt-obs
	mrpt-opengl
  mrpt-gui
	)

IF(BUILD_mrpt-hmtslam)

ENDIF() 
