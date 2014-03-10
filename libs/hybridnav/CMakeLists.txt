#---------------------------------------------
# Macro declared in "DeclareMRPTLib.cmake":
#---------------------------------------------
define_mrpt_lib(
	# Lib name
	hybridnav
	# Dependencies
	mrpt-maps        #just for now to have fast compiling 
	mrpt-graphs
	# Dependencies of deps
	mrpt-obs
	mrpt-opengl
	mrpt-base
	)
