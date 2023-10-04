# ----------------------------------------------------------------------------
# Helper macro to append space separeated value to string variable
# similar to list(APPEND ...) but uses strings+space instead of semicolon+list
# AUTHOR Jan Woetzel
#  Grabbed by JLBlanco from:
#    http://www.mip.informatik.uni-kiel.de/~jw/cmake/CMakeModules/DefineFlags.cmake
# ----------------------------------------------------------------------------
macro(STRING_APPEND  _VAR _VALUE )
  if (${_VAR})
    # not empty, add space and value
    set(${_VAR} "${${_VAR}} ${_VALUE}")
  else(${_VAR})
    # empty, no space required.
    set(${_VAR} "${_VALUE}")
  endif (${_VAR})
endmacro(STRING_APPEND)

# Based on: http://www.cmake.org/pipermail/cmake/2008-February/020114.html
# Usage: list_subdirectories(the_list_is_returned_here C:/cwd)
macro(list_subdirectories retval curdir)
  file(GLOB sub_dir RELATIVE ${curdir} *)
  set(list_of_dirs "")
  foreach(dir ${sub_dir})
    string(SUBSTRING ${dir} 0 1 dir1st)
    if(IS_DIRECTORY ${curdir}/${dir} AND NOT ${dir1st} STREQUAL "." AND NOT ${dir} STREQUAL "CMakeFiles")
        set(list_of_dirs ${list_of_dirs} ${dir})
    endif(IS_DIRECTORY ${curdir}/${dir} AND NOT ${dir1st} STREQUAL "." AND NOT ${dir} STREQUAL "CMakeFiles")
  endforeach(dir)
  set(${retval} ${list_of_dirs})
endmacro(list_subdirectories)


# Parse a list of "pkg-config"-like flags and split them into INCLUDE_DIRS,etc.
# Example: pkgconfig_parse("-Ldir1 -llib1 -Idir2" "FOO")
#  --> FOO_INCLUDE_DIRS = "dir2"
#  --> FOO_LINK_DIRS = "dir1"
#  --> FOO_LIBS = "lib1"
macro(pkgconfig_parse _FLAGS _OUT_PREFIX)
	string(REPLACE " " ";" _FLAGS_LST ${_FLAGS})
	set(${_OUT_PREFIX}_INCLUDE_DIRS "")
	set(${_OUT_PREFIX}_LINK_DIRS "")
	set(${_OUT_PREFIX}_LIBS "")
	foreach(str ${_FLAGS_LST})
		string(LENGTH ${str} _LEN)
		if (_LEN GREATER 2)
			string(SUBSTRING ${str} 0 2 _START)
			math( EXPR _LEN2 "${_LEN}-2" )
			string(SUBSTRING ${str} 2 ${_LEN2} _REST)
			if (${_START} STREQUAL "-L")
				list(APPEND ${_OUT_PREFIX}_LINK_DIRS ${_REST})
			elseif (${_START} STREQUAL "-l")
				list(APPEND ${_OUT_PREFIX}_LIBS ${_REST})
			elseif (${_START} STREQUAL "-I")
				list(APPEND ${_OUT_PREFIX}_INCLUDE_DIRS ${_REST})
			endif (${_START} STREQUAL "-L")
		endif (_LEN GREATER 2)
	endforeach(str)
endmacro(pkgconfig_parse )

# Converts a version like "1.2.3" into a string "0x10203",
# or "3.4.19" into "0x30413".
# Usage: VERSION_TO_HEXADECIMAL(TARGET_VAR "1.2.3")
macro(VERSION_TO_HEXADECIMAL OUT_VAR IN_VERSION)
  string(REGEX MATCHALL "[0-9]+" VERSION_PARTS "${IN_VERSION}")
  list(GET VERSION_PARTS 0 VERSION_NUMBER_MAJOR)
  list(GET VERSION_PARTS 1 VERSION_NUMBER_MINOR)
  list(GET VERSION_PARTS 2 VERSION_NUMBER_PATCH)

  # Convert each part to hex:
  math(EXPR ${OUT_VAR}
  "(${VERSION_NUMBER_MAJOR} << 16) + \
    (${VERSION_NUMBER_MINOR} << 8) + \
    (${VERSION_NUMBER_PATCH})" OUTPUT_FORMAT HEXADECIMAL )
endmacro()


# GOOD & BAD are single strings, INPUT is a list wrapped in string
# OUTPUT is a name for a list
# Based on: https://stackoverflow.com/a/30680445
macro(mrpt_split_lib_list INPUT OUTPUT GOOD BAD)
  set(LST ${${INPUT}})   # can we avoid this?
  set(PICKME YES)
  foreach(ELEMENT IN LISTS LST)
    if(${ELEMENT} STREQUAL general OR ${ELEMENT} STREQUAL ${GOOD})
      set(PICKME YES)
    elseif(${ELEMENT} STREQUAL ${BAD})
      set(PICKME NO)
    elseif(PICKME)
      list(APPEND ${OUTPUT} ${ELEMENT})
    endif()
  endforeach()
endmacro()
