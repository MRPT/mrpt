if(UNIX)
	execute_process(COMMAND uname -m
		OUTPUT_VARIABLE CMAKE_MRPT_ARCH
                OUTPUT_STRIP_TRAILING_WHITESPACE)
	message(STATUS "Architecture uname -m: ${CMAKE_MRPT_ARCH}")
endif(UNIX)
