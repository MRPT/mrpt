# ----------------------------------------------------------------------------
# Helper macro to append space separeated value to string variable
# similar to LIST(APPEND ...) but uses strings+space instead of semicolon+list
# AUTHOR Jan Woetzel
#  Grabbed by JLBlanco from: 
#    http://www.mip.informatik.uni-kiel.de/~jw/cmake/CMakeModules/DefineFlags.cmake
# ----------------------------------------------------------------------------
MACRO(STRING_APPEND  _VAR _VALUE )
  IF (${_VAR})
    # not empty, add space and value
    SET(${_VAR} "${${_VAR}} ${_VALUE}")
  ELSE(${_VAR})
    # empty, no space required.
    SET(${_VAR} "${_VALUE}")
  ENDIF (${_VAR})
ENDMACRO(STRING_APPEND)

# ------------------------------------------------------------------------
# For usage below. Checks whether we have to add the "general" prefix.
# ------------------------------------------------------------------------
MACRO(APPEND_MRPT_LIBS )
	IF(NOT ARGV STREQUAL "")  # Do nothing for an empty string
		IF(${ARGV0} STREQUAL "debug" OR ${ARGV0} STREQUAL "optimized")
			set(_libs ${ARGV})
		ELSE(${ARGV0} STREQUAL "debug" OR ${ARGV0} STREQUAL "optimized")
			set(_libs general ${ARGV})
		ENDIF(${ARGV0} STREQUAL "debug" OR ${ARGV0} STREQUAL "optimized")
		LIST(APPEND MRPT_LINKER_LIBS ${_libs})
	ENDIF(NOT ARGV STREQUAL "")
ENDMACRO(APPEND_MRPT_LIBS)
