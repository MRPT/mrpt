# Build (or not) some apps:
# ===================================================
SET(BUILD_MONOSLAM OFF CACHE BOOL "Build library mrpt-monoslam")
SET(BUILD_STEREOSLAM OFF CACHE BOOL "Build library mrpt-stereoslam")
MARK_AS_ADVANCED(BUILD_MONOSLAM)
MARK_AS_ADVANCED(BUILD_STEREOSLAM )

# If some app is not in this package, do NOT build it:
# =====================================================
IF (NOT EXISTS "${MRPT_SOURCE_DIR}/libs/monoslam/src")
	SET(BUILD_MONOSLAM OFF CACHE INTERNAL "" FORCE)
ENDIF (NOT EXISTS "${MRPT_SOURCE_DIR}/libs/monoslam/src")

IF (NOT EXISTS "${MRPT_SOURCE_DIR}/libs/stereoslam/src")
	SET(BUILD_STEREOSLAM OFF CACHE INTERNAL "" FORCE)
ENDIF (NOT EXISTS "${MRPT_SOURCE_DIR}/libs/stereoslam/src")
