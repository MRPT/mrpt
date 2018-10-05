# Build (or not) some apps:
# ===================================================
set(BUILD_MONOSLAM OFF CACHE BOOL "Build library mrpt-monoslam")
set(BUILD_STEREOSLAM OFF CACHE BOOL "Build library mrpt-stereoslam")
mark_as_advanced(BUILD_MONOSLAM)
mark_as_advanced(BUILD_STEREOSLAM )

# If some app is not in this package, do NOT build it:
# =====================================================
if (NOT EXISTS "${MRPT_SOURCE_DIR}/libs/monoslam/src")
	set(BUILD_MONOSLAM OFF CACHE INTERNAL "" FORCE)
endif (NOT EXISTS "${MRPT_SOURCE_DIR}/libs/monoslam/src")

if (NOT EXISTS "${MRPT_SOURCE_DIR}/libs/stereoslam/src")
	set(BUILD_STEREOSLAM OFF CACHE INTERNAL "" FORCE)
endif (NOT EXISTS "${MRPT_SOURCE_DIR}/libs/stereoslam/src")
