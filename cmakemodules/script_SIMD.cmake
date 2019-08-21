# SIMD extensions
# ===================================================
set(AUTODETECT_SIMD_DEFAULT OFF)
if(EXISTS "/proc/cpuinfo" AND
	("${CMAKE_MRPT_ARCH}" STREQUAL "x86_64" OR
	"${CMAKE_MRPT_ARCH}" STREQUAL "i686") )
	set(AUTODETECT_SIMD_DEFAULT ON)
endif()

set(MRPT_AUTODETECT_SIMD ${AUTODETECT_SIMD_DEFAULT} CACHE BOOL "Check /proc/cpuinfo to determine if SSE{2,3,4} optimizations are available")
mark_as_advanced(MRPT_AUTODETECT_SIMD)

# Read info about CPUs:
if (MRPT_AUTODETECT_SIMD)
	file(READ "/proc/cpuinfo" MRPT_CPU_INFO)
endif()

# Macro for each SSE* var: Invoke with name in uppercase:
macro(DEFINE_SIMD_VAR  _setname)
	string(TOLOWER ${_setname} _set)

	if (MRPT_AUTODETECT_SIMD)
		# Automatic detection:
		set(CMAKE_MRPT_HAS_${_setname} 0)
		if (${MRPT_CPU_INFO} MATCHES ".*${_set}.*")
			set(CMAKE_MRPT_HAS_${_setname} 1)
		endif()
	else()
		# Manual:
		set("ENABLE_${_setname}" OFF CACHE BOOL "Forces compilation WITH ${_setname} extensions")
		mark_as_advanced("ENABLE_${_setname}")
		set(CMAKE_MRPT_HAS_${_setname} 0)
		if (ENABLE_${_setname})
			set(CMAKE_MRPT_HAS_${_setname} 1)
		endif()
	endif()
endmacro()

# SSE optimizations:
DEFINE_SIMD_VAR(SSE2)
DEFINE_SIMD_VAR(SSE3)
DEFINE_SIMD_VAR(SSE4_1)
DEFINE_SIMD_VAR(SSE4_2)
DEFINE_SIMD_VAR(SSE4_A)
DEFINE_SIMD_VAR(AVX)
DEFINE_SIMD_VAR(AVX2)
DEFINE_SIMD_VAR(NEON)
