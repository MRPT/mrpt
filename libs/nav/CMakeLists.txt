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

#---------------------------------------------
# Macro declared in "DeclareMRPTLib.cmake":
#---------------------------------------------
define_mrpt_lib(
	# Lib name
	nav
	# Dependencies
	mrpt-maps
	mrpt-kinematics
	)
