# Lists of directories with source files:
#  See "DeclareMRPTLib.cmake" for explanations
# -------------------------------------------------
set(NAVDIR "${MRPT_SOURCE_DIR}/libs/nav")

list(APPEND nav_EXTRA_SRCS	      "${NAVDIR}/src/tpspace/*.cpp" "${NAVDIR}/include/mrpt/nav/tpspace/*.h")
list(APPEND nav_EXTRA_SRCS_NAME   "PTGs_TPSpace" "PTGs_TPSpace")

list(APPEND nav_EXTRA_SRCS	      "${NAVDIR}/src/holonomic/*.cpp" "${NAVDIR}/include/mrpt/nav/holonomic/*.h")
list(APPEND nav_EXTRA_SRCS_NAME   "HolonomicNav" "HolonomicNav")

list(APPEND nav_EXTRA_SRCS	      "${NAVDIR}/src/reactive/*.cpp" "${NAVDIR}/include/mrpt/nav/reactive/*.h")
list(APPEND nav_EXTRA_SRCS_NAME   "ReactiveNav" "ReactiveNav")

list(APPEND nav_EXTRA_SRCS	      "${NAVDIR}/src/planners/*.cpp" "${NAVDIR}/include/mrpt/nav/planners/*.h")
list(APPEND nav_EXTRA_SRCS_NAME   "Planners" "Planners")

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
