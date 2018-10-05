# Check for the FTDI headers (Linux only, in win32
#  we use built-in header & dynamic DLL load):
# ===================================================
set(CMAKE_MRPT_HAS_FTDI 0)

# This option will be available only on Linux, hence it's declared here:
option(DISABLE_FTDI "Do not use the USB driver for FTDI chips" 0)
mark_as_advanced(DISABLE_FTDI)

if (NOT DISABLE_FTDI)
if(UNIX)
	# 1st: try to find LIBFTDI1 (1.2+)
	find_package(LibFTDI1 QUIET)
	if (LibFTDI1_FOUND)
		set(CMAKE_MRPT_HAS_FTDI 1)
		set(CMAKE_MRPT_HAS_FTDI_SYSTEM 1)

		set(FTDI_INCLUDE_DIRS ${LIBFTDI_INCLUDE_DIRS})
		set(FTDI_LINK_DIRS ${LIBFTDI_LIBRARY_DIRS})
		set(FTDI_LIBS ${LIBFTDI_LIBRARIES})
	else()
		# 2nd: Find old libftdi
		find_file(FTDI_CONFIG_FILE libftdi-config)
		if(FTDI_CONFIG_FILE)
			mark_as_advanced(FTDI_CONFIG_FILE)

			set(CMAKE_MRPT_HAS_FTDI 1)
			set(CMAKE_MRPT_HAS_FTDI_SYSTEM 1)

			set(LIBFTDI_VERSION_MAJOR 1)
			set(LIBFTDI_VERSION_MINOR 0)

			# Get the config params:
			execute_process(COMMAND ${FTDI_CONFIG_FILE} --libs
				RESULT_VARIABLE CMAKE_FTDI_CONFIG_RES
				OUTPUT_VARIABLE CMAKE_FTDI_LIBS
				OUTPUT_STRIP_TRAILING_WHITESPACE
				)
			if(${CMAKE_FTDI_CONFIG_RES})
				message("Error invoking FTDI config file:\n ${FTDI_CONFIG_FILE} --libs")
			endif(${CMAKE_FTDI_CONFIG_RES})

			pkgconfig_parse(${CMAKE_FTDI_LIBS} "FTDI")
		endif(FTDI_CONFIG_FILE)
	endif (LibFTDI1_FOUND)

	if(CMAKE_MRPT_HAS_FTDI)
		if($ENV{VERBOSE})
			message(STATUS "libftdi configuration:")
			message(STATUS "  FTDI_INCLUDE_DIRS: ${FTDI_INCLUDE_DIRS}")
			message(STATUS "  FTDI_LINK_DIRS: ${FTDI_LINK_DIRS}")
			message(STATUS "  FTDI_LIBS: ${FTDI_LIBS}")
		endif($ENV{VERBOSE})

		#APPEND_MRPT_LIBS(${FTDI_LIBS})
		link_directories(${FTDI_LINK_DIRS})
		include_directories(${FTDI_INCLUDE_DIRS})
	endif()

else(UNIX)
	# In windows we always have FTDI support (at compile time at least...)
	set(CMAKE_MRPT_HAS_FTDI 1)

	set(LIBFTDI_VERSION_MAJOR 1)
	set(LIBFTDI_VERSION_MINOR 0)
endif(UNIX)
endif (NOT DISABLE_FTDI)


