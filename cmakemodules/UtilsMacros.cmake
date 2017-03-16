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
	IF(NOT "${ARGV}" STREQUAL "")  # Do nothing for an empty string
		IF(${ARGV0} STREQUAL "debug" OR ${ARGV0} STREQUAL "optimized")
			set(_libs ${ARGV})
		ELSE(${ARGV0} STREQUAL "debug" OR ${ARGV0} STREQUAL "optimized")
			set(_libs general ${ARGV})
		ENDIF(${ARGV0} STREQUAL "debug" OR ${ARGV0} STREQUAL "optimized")
		LIST(APPEND MRPT_LINKER_LIBS ${_libs})
	ENDIF(NOT "${ARGV}" STREQUAL "")
ENDMACRO(APPEND_MRPT_LIBS)

# Add a path to "-isystem", filtering out "/usr/include", without checking for the right compiler:
MACRO(INTERNAL_ADD_ISYSTEM   DIR)
	get_filename_component(PATH_DIR "${DIR}" ABSOLUTE)
	if (NOT "${PATH_DIR}" STREQUAL "/usr/include")
	    IF($ENV{VERBOSE})
		    MESSAGE(STATUS "isystem: Path added: ${PATH_DIR}")
	    ENDIF()
	    INCLUDE_DIRECTORIES(SYSTEM ${PATH_DIR})
	endif()
ENDMACRO()

# Only if GNU GCC is used, add one "-isystem" flag for each include directory.
# Useful to discard -pedantic errors in system libraries not prepared to be so... well, pedantic.
MACRO(ADD_DIRECTORIES_AS_ISYSTEM INCLUDE_DIRS)
	IF(CMAKE_COMPILER_IS_GNUCXX OR ${CMAKE_CXX_COMPILER_ID} STREQUAL "Clang")
		FOREACH(DIR ${${INCLUDE_DIRS}})
			INTERNAL_ADD_ISYSTEM(${DIR})
		ENDFOREACH(DIR)
	ENDIF()
ENDMACRO(ADD_DIRECTORIES_AS_ISYSTEM)

MACRO(ADD_DIRECTORIES_AS_INCLUDE_AND_ISYSTEM INCLUDE_DIRS)
	IF(CMAKE_COMPILER_IS_GNUCXX OR ${CMAKE_CXX_COMPILER_ID} STREQUAL "Clang")
		FOREACH(DIR ${${INCLUDE_DIRS}})
			SET(CMAKE_CXX_FLAGS "-I ${DIR} ${CMAKE_CXX_FLAGS}")
			SET(CMAKE_C_FLAGS "-I ${DIR} ${CMAKE_C_FLAGS}")
			INTERNAL_ADD_ISYSTEM(${DIR})
		ENDFOREACH(DIR)
	ENDIF()
ENDMACRO(ADD_DIRECTORIES_AS_INCLUDE_AND_ISYSTEM)


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
	SET(${_OUT_PREFIX}_INCLUDE_DIRS "")
	SET(${_OUT_PREFIX}_LINK_DIRS "")
	SET(${_OUT_PREFIX}_LIBS "")
	foreach(str ${_FLAGS_LST})
		string(LENGTH ${str} _LEN)
		if (_LEN GREATER 2)
			string(SUBSTRING ${str} 0 2 _START)
			MATH( EXPR _LEN2 "${_LEN}-2" )
			string(SUBSTRING ${str} 2 ${_LEN2} _REST)
			IF (${_START} STREQUAL "-L")
				LIST(APPEND ${_OUT_PREFIX}_LINK_DIRS ${_REST})
			ELSEIF (${_START} STREQUAL "-l")
				LIST(APPEND ${_OUT_PREFIX}_LIBS ${_REST})
			ELSEIF (${_START} STREQUAL "-I")
				LIST(APPEND ${_OUT_PREFIX}_INCLUDE_DIRS ${_REST})
			ENDIF (${_START} STREQUAL "-L")
		endif (_LEN GREATER 2)
	endforeach(str)
endmacro(pkgconfig_parse )

# Convert a decimal value [0,15] to hexadecimal
# From: http://stackoverflow.com/questions/26182289/convert-from-decimal-to-hexadecimal-in-cmake
macro(DECCHAR2HEX VAR VAL)
        if (${VAL} LESS 10)
            SET(${VAR} ${VAL})
        elseif(${VAL} EQUAL 10)
            SET(${VAR} "A")
        elseif(${VAL} EQUAL 11)
            SET(${VAR} "B")
        elseif(${VAL} EQUAL 12)
            SET(${VAR} "C")
        elseif(${VAL} EQUAL 13)
            SET(${VAR} "D")
        elseif(${VAL} EQUAL 14)
            SET(${VAR} "E")
        elseif(${VAL} EQUAL 15)
            SET(${VAR} "F")
        else(${VAL} LESS 10)
            MESSAGE(FATAL_ERROR "Invalid format for hexidecimal character")
        endif(${VAL} LESS 10)
endmacro(DECCHAR2HEX)

# Converts a version like "1.2.3" into a string "0x123"
# Usage: VERSION_TO_HEXADECIMAL(TARGET_VAR "1.2.3")
MACRO(VERSION_TO_HEXADECIMAL  OUT_VAR IN_VERSION)
	STRING(REGEX MATCHALL "[0-9]+" VERSION_PARTS "${IN_VERSION}")
	LIST(GET VERSION_PARTS 0 VERSION_NUMBER_MAJOR)
	LIST(GET VERSION_PARTS 1 VERSION_NUMBER_MINOR)
	LIST(GET VERSION_PARTS 2 VERSION_NUMBER_PATCH)
	# Convert each part to hex:
	DECCHAR2HEX(VERSION_NUMBER_MAJOR_HEX ${VERSION_NUMBER_MAJOR})
	DECCHAR2HEX(VERSION_NUMBER_MINOR_HEX ${VERSION_NUMBER_MINOR})
	DECCHAR2HEX(VERSION_NUMBER_PATCH_HEX ${VERSION_NUMBER_PATCH})
	# Concat version string:
	SET(${OUT_VAR} "0x${VERSION_NUMBER_MAJOR_HEX}${VERSION_NUMBER_MINOR_HEX}${VERSION_NUMBER_PATCH_HEX}")
ENDMACRO(VERSION_TO_HEXADECIMAL)
