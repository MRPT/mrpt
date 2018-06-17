# SSE{2,3,4} extensions?
# ===================================================
include(CheckCXXCompilerFlag)

SET(MRPT_AUTODETECT_SSE ON CACHE BOOL "Check /proc/cpuinfo to determine if SSE{2,3,4} optimizations are available")
MARK_AS_ADVANCED(MRPT_AUTODETECT_SSE)

# Read info about CPUs:
SET(DO_SSE_AUTODETECT 0)
IF(MRPT_AUTODETECT_SSE AND EXISTS "/proc/cpuinfo" AND
	("${CMAKE_MRPT_ARCH}" STREQUAL "x86_64" OR
	"${CMAKE_MRPT_ARCH}" STREQUAL "i686") )
	SET(DO_SSE_AUTODETECT 1)
ENDIF()

# Macro for each SSE* var: Invoke with name in uppercase:
macro(DEFINE_SSE_VAR _flag  _setname)
	string(TOLOWER ${_setname} _set)

	IF (DO_SSE_AUTODETECT)
		# Automatic detection:
        CHECK_CXX_COMPILER_FLAG(${_flag} CMAKE_MRPT_HAS_${_setname})
        IF(NOT CMAKE_MRPT_HAS_${_setname})
		    SET(CMAKE_MRPT_HAS_${_setname} 0)
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
DEFINE_SSE_VAR("-msse2" SSE2)
DEFINE_SSE_VAR("-msse3" SSE3)
DEFINE_SSE_VAR("-msse4.1" SSE4_1)
DEFINE_SSE_VAR("-msse4.2" SSE4_2)
DEFINE_SSE_VAR("-msse4a" SSE4_A)
