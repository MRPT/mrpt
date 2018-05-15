if(UNIX)
	execute_process(COMMAND uname -m
		OUTPUT_VARIABLE CMAKE_MRPT_ARCH
                OUTPUT_STRIP_TRAILING_WHITESPACE)
	execute_process(COMMAND uname -s
		OUTPUT_VARIABLE CMAKE_MRPT_KERNEL
                OUTPUT_STRIP_TRAILING_WHITESPACE)
	message(STATUS "Kernel name (uname -s): ${CMAKE_MRPT_KERNEL}")
endif(UNIX)
