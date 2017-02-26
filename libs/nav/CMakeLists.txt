# Lists of directories with source files:
#  See "DeclareMRPTLib.cmake" for explanations
# -------------------------------------------------
SET(NAVDIR "${MRPT_SOURCE_DIR}/libs/nav")

LIST(APPEND nav_EXTRA_SRCS	      "${NAVDIR}/src/tpspace/*.cpp" "${NAVDIR}/include/mrpt/nav/tpspace/*.h")
LIST(APPEND nav_EXTRA_SRCS_NAME   "PTGs_TPSpace" "PTGs_TPSpace")

LIST(APPEND nav_EXTRA_SRCS	      "${NAVDIR}/src/holonomic/*.cpp" "${NAVDIR}/include/mrpt/nav/holonomic/*.h")
LIST(APPEND nav_EXTRA_SRCS_NAME   "HolonomicNav" "HolonomicNav")

LIST(APPEND nav_EXTRA_SRCS	      "${NAVDIR}/src/reactive/*.cpp" "${NAVDIR}/include/mrpt/nav/reactive/*.h")
LIST(APPEND nav_EXTRA_SRCS_NAME   "ReactiveNav" "ReactiveNav")

LIST(APPEND nav_EXTRA_SRCS	      "${NAVDIR}/src/planners/*.cpp" "${NAVDIR}/include/mrpt/nav/planners/*.h")
LIST(APPEND nav_EXTRA_SRCS_NAME   "Planners" "Planners")

# Compatibility with MRPT <1.3.0
LIST(APPEND nav_EXTRA_SRCS	      "${NAVDIR}/include/mrpt/reactivenav/*.h")
LIST(APPEND nav_EXTRA_SRCS_NAME   "bckwrds_compat_hdrs")

#---------------------------------------------
# Macro declared in "DeclareMRPTLib.cmake":
#---------------------------------------------
define_mrpt_lib(
	# Lib name
	nav    # Includes old "reactivenav" + new stuff
	# Dependencies
	mrpt-graphs
	mrpt-maps
	mrpt-kinematics
	# Depts of the deps:
	mrpt-graphs  # dep of mrpt-maps
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
ENDIF() 

