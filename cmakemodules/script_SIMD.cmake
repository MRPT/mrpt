# SSE{2,3,4} extensions?
# ===================================================
SET(MRPT_AUTODETECT_SSE ON CACHE BOOL "Check /proc/cpuinfo to determine if SSE{2,3,4} optimizations are available")
MARK_AS_ADVANCED(MRPT_AUTODETECT_SSE)

# Read info about CPUs:
SET(DO_SSE_AUTODETECT 0)
IF(MRPT_AUTODETECT_SSE AND EXISTS "/proc/cpuinfo")
	SET(DO_SSE_AUTODETECT 1)
ENDIF(MRPT_AUTODETECT_SSE AND EXISTS "/proc/cpuinfo")

IF (DO_SSE_AUTODETECT)
	FILE(READ "/proc/cpuinfo" MRPT_CPU_INFO)
ENDIF (DO_SSE_AUTODETECT)

# Macro for each SSE* var: Invoke with name in uppercase:
macro(DEFINE_SSE_VAR  _setname)
	string(TOLOWER ${_setname} _set)

	IF (DO_SSE_AUTODETECT)
		# Automatic detection:
		SET(CMAKE_MRPT_HAS_${_setname} 0)
		IF (${MRPT_CPU_INFO} MATCHES ".*${_set}.*")
			SET(CMAKE_MRPT_HAS_${_setname} 1)
		ENDIF()
	ELSE (DO_SSE_AUTODETECT)
		# Manual:
		SET("DISABLE_${_setname}" OFF CACHE BOOL "Forces compilation WITHOUT ${_setname} extensions")
		MARK_AS_ADVANCED("DISABLE_${_setname}")
		SET(CMAKE_MRPT_HAS_${_setname} 0)
		IF (NOT DISABLE_${_setname})
			SET(CMAKE_MRPT_HAS_${_setname} 1)
		ENDIF (NOT DISABLE_${_setname})	
	ENDIF (DO_SSE_AUTODETECT)
endmacro(DEFINE_SSE_VAR)

# SSE optimizations:
DEFINE_SSE_VAR(SSE2)
DEFINE_SSE_VAR(SSE3)
DEFINE_SSE_VAR(SSE4_1)
DEFINE_SSE_VAR(SSE4_2)
DEFINE_SSE_VAR(SSE4_A)
