# SSE{2,3,4} extensions?
# ===================================================
IF(EXISTS "/proc/cpuinfo")
	SET(MRPT_AUTODETECT_SSE ON CACHE BOOL "Check /proc/cpuinfo to determine if SSE{2,3,4} optimizations are available")
	MARK_AS_ADVANCED(MRPT_AUTODETECT_SSE)
	FILE(READ "/proc/cpuinfo" MRPT_CPU_INFO)

	SET(CMAKE_MRPT_HAS_SSE2 0)
	IF ("${MRPT_CPU_INFO}" MATCHES ".*sse2.*")
		SET(CMAKE_MRPT_HAS_SSE2 1)
	ENDIF("${MRPT_CPU_INFO}" MATCHES ".*sse2.*")

	SET(CMAKE_MRPT_HAS_SSE3 0)
	IF ("${MRPT_CPU_INFO}" MATCHES ".*sse3.*")
		SET(CMAKE_MRPT_HAS_SSE3 1)
	ENDIF("${MRPT_CPU_INFO}" MATCHES ".*sse3.*")

	SET(CMAKE_MRPT_HAS_SSE4 0)
	IF ("${MRPT_CPU_INFO}" MATCHES ".*sse4.*")
		SET(CMAKE_MRPT_HAS_SSE4 1)
	ENDIF("${MRPT_CPU_INFO}" MATCHES ".*sse4.*")

	MESSAGE(STATUS "CPU supported SIMD extensions: SSE2=${CMAKE_MRPT_HAS_SSE2} SSE3=${CMAKE_MRPT_HAS_SSE3} SSE4=${CMAKE_MRPT_HAS_SSE4} ")
ELSE(EXISTS "/proc/cpuinfo")
	# By default: Leave all optimizations enabled. If the compiler doesn't support them it will be ignored anyway.
	SET(CMAKE_MRPT_HAS_SSE2 1)
	SET(CMAKE_MRPT_HAS_SSE3 1)
	SET(CMAKE_MRPT_HAS_SSE4 1)
ENDIF(EXISTS "/proc/cpuinfo")

