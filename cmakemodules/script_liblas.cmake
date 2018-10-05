# Check for the LAS (LiDAR format) library
# ===================================================
if(UNIX)
	find_file(LAS_CONFIG_FILE liblas-config)
	if(LAS_CONFIG_FILE)
		mark_as_advanced(LAS_CONFIG_FILE)

		set(CMAKE_MRPT_HAS_LIBLAS 1)
		set(CMAKE_MRPT_HAS_LIBLAS_SYSTEM 1)

		# Get the config params:
		execute_process(COMMAND ${LAS_CONFIG_FILE} --libs
			RESULT_VARIABLE CMAKE_LAS_CONFIG_RES
			OUTPUT_VARIABLE CMAKE_LAS_LIBS
			OUTPUT_STRIP_TRAILING_WHITESPACE
			)
		if(${CMAKE_LAS_CONFIG_RES})
			message("Error invoking LAS config file:\n ${LAS_CONFIG_FILE} --libs")
		endif(${CMAKE_LAS_CONFIG_RES})

		execute_process(COMMAND ${LAS_CONFIG_FILE} --includes
			RESULT_VARIABLE CMAKE_LAS_CONFIG_RES
			OUTPUT_VARIABLE CMAKE_LAS_INCLUDES
			OUTPUT_STRIP_TRAILING_WHITESPACE
			)
		if(${CMAKE_LAS_CONFIG_RES})
			message("Error invoking LAS config file:\n ${LAS_CONFIG_FILE} --includes")
		endif(${CMAKE_LAS_CONFIG_RES})

		# Join all flags and parse to separate them:
		set(CMAKE_LAS_CFGS "${CMAKE_LAS_LIBS} ${CMAKE_LAS_INCLUDES}")

		pkgconfig_parse(${CMAKE_LAS_CFGS} "LAS")

		# For some reason, "liblas-config --libs" return all other libs, except liblas itself:
		list(APPEND LAS_LIBS "las")

		if($ENV{VERBOSE})
			message(STATUS "liblas configuration:")
			message(STATUS "  LAS_INCLUDE_DIRS: ${LAS_INCLUDE_DIRS}")
			message(STATUS "  LAS_LINK_DIRS: ${LAS_LINK_DIRS}")
			message(STATUS "  LAS_LIBS: ${LAS_LIBS}")
		endif($ENV{VERBOSE})

	else(LAS_CONFIG_FILE)
		set(CMAKE_MRPT_HAS_LIBLAS 0)
	endif(LAS_CONFIG_FILE)
else(UNIX)
	# Windows: (Not supported for now)
	set(CMAKE_MRPT_HAS_LIBLAS 0)
endif(UNIX)

