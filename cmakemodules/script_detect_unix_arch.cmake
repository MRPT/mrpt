
# Rely on CMake to detect target system architecture:
SET(CMAKE_MRPT_ARCH ${CMAKE_SYSTEM_PROCESSOR})

if(UNIX)
	execute_process(COMMAND uname -s
		OUTPUT_VARIABLE CMAKE_MRPT_KERNEL
		OUTPUT_STRIP_TRAILING_WHITESPACE)
	if ($ENV{VERBOSE})
		message(STATUS "Kernel name (uname -s): ${CMAKE_MRPT_KERNEL}")
	endif()
endif()

# Detect if we are in i386 / amd64:
# Intel arch names in Linux & Windows.
if ("${CMAKE_MRPT_ARCH}" MATCHES "^(x86_64|i686|AMD64|IA64|x86)$")
	set(MRPT_ARCH_INTEL_COMPATIBLE 1)
else()
	set(MRPT_ARCH_INTEL_COMPATIBLE 0)
endif()
