# SSE{2,3,4} extensions?
# ===================================================
set(MRPT_AUTODETECT_SSE ON CACHE BOOL "Check /proc/cpuinfo to determine if SSE{2,3,4} optimizations are available")
mark_as_advanced(MRPT_AUTODETECT_SSE)

# Read info about CPUs:
set(DO_SSE_AUTODETECT 0)
if(MRPT_AUTODETECT_SSE AND EXISTS "/proc/cpuinfo" AND
	("${CMAKE_MRPT_ARCH}" STREQUAL "x86_64" OR
	"${CMAKE_MRPT_ARCH}" STREQUAL "i686") )
	set(DO_SSE_AUTODETECT 1)
endif()

if (DO_SSE_AUTODETECT)
	file(READ "/proc/cpuinfo" MRPT_CPU_INFO)
endif (DO_SSE_AUTODETECT)

# Macro for each SSE* var: Invoke with name in uppercase:
macro(DEFINE_SSE_VAR  _setname)
	string(TOLOWER ${_setname} _set)

	if (DO_SSE_AUTODETECT)
		# Automatic detection:
		set(CMAKE_MRPT_HAS_${_setname} 0)
		if (${MRPT_CPU_INFO} MATCHES ".*${_set}.*")
			set(CMAKE_MRPT_HAS_${_setname} 1)
		endif()
	else (DO_SSE_AUTODETECT)
		# Manual:
		set("DISABLE_${_setname}" OFF CACHE BOOL "Forces compilation WITHOUT ${_setname} extensions")
		mark_as_advanced("DISABLE_${_setname}")
		set(CMAKE_MRPT_HAS_${_setname} 0)
		if (NOT DISABLE_${_setname})
			set(CMAKE_MRPT_HAS_${_setname} 1)
		endif (NOT DISABLE_${_setname})
	endif (DO_SSE_AUTODETECT)
endmacro(DEFINE_SSE_VAR)

# SSE optimizations:
DEFINE_SSE_VAR(SSE2)
DEFINE_SSE_VAR(SSE3)
DEFINE_SSE_VAR(SSE4_1)
DEFINE_SSE_VAR(SSE4_2)
DEFINE_SSE_VAR(SSE4_A)
