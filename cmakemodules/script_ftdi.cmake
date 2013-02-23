# Check for the FTDI headers (Linux only, in win32
#  we use built-in header & dynamic DLL load):
# ===================================================
IF(UNIX)
	FIND_FILE(FTDI_CONFIG_FILE libftdi-config)
	IF(FTDI_CONFIG_FILE)
		MARK_AS_ADVANCED(FTDI_CONFIG_FILE)

		SET(CMAKE_MRPT_HAS_FTDI 1)
		SET(CMAKE_MRPT_HAS_FTDI_SYSTEM 1)

		# Get the config params:
		EXECUTE_PROCESS(COMMAND ${FTDI_CONFIG_FILE} --libs
			RESULT_VARIABLE CMAKE_FTDI_CONFIG_RES
			OUTPUT_VARIABLE CMAKE_FTDI_LIBS
			OUTPUT_STRIP_TRAILING_WHITESPACE
			)
		IF(${CMAKE_FTDI_CONFIG_RES})
			MESSAGE("Error invoking FTDI config file:\n ${FTDI_CONFIG_FILE} --libs")
		ENDIF(${CMAKE_FTDI_CONFIG_RES})

		pkgconfig_parse(${CMAKE_FTDI_LIBS} "FTDI")
		IF($ENV{VERBOSE})
			MESSAGE(STATUS "libftdi configuration:")
			MESSAGE(STATUS "  FTDI_INCLUDE_DIRS: ${FTDI_INCLUDE_DIRS}")
			MESSAGE(STATUS "  FTDI_LINK_DIRS: ${FTDI_LINK_DIRS}")
			MESSAGE(STATUS "  FTDI_LIBS: ${FTDI_LIBS}")
		ENDIF($ENV{VERBOSE})

		APPEND_MRPT_LIBS(${FTDI_LIBS})
		LINK_DIRECTORIES(${FTDI_LINK_DIRS})
		INCLUDE_DIRECTORIES(${FTDI_INCLUDE_DIRS})


	ELSE(FTDI_CONFIG_FILE)
		SET(CMAKE_MRPT_HAS_FTDI 0)
	ENDIF(FTDI_CONFIG_FILE)
	# This option will be available only on Linux, hence it's declared here:
	OPTION(DISABLE_FTDI "Do not use the USB driver for FTDI chips" 0)
	MARK_AS_ADVANCED(DISABLE_FTDI)
ELSE(UNIX)
	# In windows we always have FTDI support (at compile time at least...)
	SET(CMAKE_MRPT_HAS_FTDI 1)
ENDIF(UNIX)


