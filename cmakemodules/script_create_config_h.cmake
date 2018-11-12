# ----------------------------------------------------------------------------
#   				UPDATE CONFIG FILES & SCRIPTS:
#
#  configure_file(InputFile OutputFile [COPYONLY] [ESCAPE_QUOTES] [@ONLY])
# If @ONLY is specified, only variables of the form @VAR@ will be
#  replaces and ${VAR} will be ignored.
#
#  A directory will be created for each platform so the "config.h" file is
#   not overwritten if cmake generates code in the same path.
# ----------------------------------------------------------------------------
set(MRPT_CONFIG_FILE_INCLUDE_DIR "${CMAKE_BINARY_DIR}/include/mrpt-configuration/" CACHE PATH "Where to create the platform-dependant config.h")
if(UNIX)
	set(MRPT_CONFIG_FILE_INCLUDE_DIR "${CMAKE_BINARY_DIR}/include/mrpt-configuration/unix/" )
endif(UNIX)
if (WIN32)
	set(MRPT_CONFIG_FILE_INCLUDE_DIR "${CMAKE_BINARY_DIR}/include/mrpt-configuration/win32/")
endif(WIN32)

file(MAKE_DIRECTORY  "${MRPT_CONFIG_FILE_INCLUDE_DIR}")
file(MAKE_DIRECTORY  "${MRPT_CONFIG_FILE_INCLUDE_DIR}/mrpt")

configure_file("${CMAKE_SOURCE_DIR}/parse-files/config.h.in" "${MRPT_CONFIG_FILE_INCLUDE_DIR}/mrpt/config.h")
