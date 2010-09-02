#---------------------------------------------
# Macro declared in "DeclareMRPTLib.cmake":
#---------------------------------------------
define_mrpt_lib(
	# Lib name
	detectors
	# Deps:
	mrpt-vision
	mrpt-maps
	mrpt-gui
	mrpt-slam
	# And deps of the deps:
	mrpt-base
	mrpt-opengl
	mrpt-obs	
	)
