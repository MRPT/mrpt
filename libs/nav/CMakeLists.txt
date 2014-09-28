# Lists of directories with source files:
#  See "DeclareMRPTLib.cmake" for explanations
# -------------------------------------------------
LIST(APPEND nav_EXTRA_SRCS	
	"${MRPT_SOURCE_DIR}/libs/nav/include/mrpt/reactivenav/*.h")
LIST(APPEND nav_EXTRA_SRCS_NAME
	"nav")


#---------------------------------------------
# Macro declared in "DeclareMRPTLib.cmake":
#---------------------------------------------
define_mrpt_lib(
	# Lib name
	nav    # Includes old "reactivenav" + new stuff
	# Dependencies
	mrpt-graphs
	mrpt-maps
	# Depts of the deps:
	mrpt-obs
	mrpt-opengl
	mrpt-base
	)

IF(BUILD_mrpt-nav)

	# Declare "meta-lib" for backwards compatibility:
	define_mrpt_metalib(
		# Lib name
		reactivenav
		# Deps:
		mrpt-nav
		# Depts of the deps:
		mrpt-graphs
		mrpt-maps
		mrpt-obs
		mrpt-opengl
		mrpt-base
		)
		
ENDIF(BUILD_mrpt-nav) 

