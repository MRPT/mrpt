# ---------------------------------------------------------------
#   "Clasic" function & headers detection:
# ---------------------------------------------------------------
include (CheckFunctionExists)
include (CheckIncludeFile)
include (CheckTypeSize)

set(CMAKE_REQUIRED_INCLUDES "math.h")
if(CMAKE_COMPILER_IS_GNUCXX)
	set(CMAKE_REQUIRED_LIBRARIES "m")
endif(CMAKE_COMPILER_IS_GNUCXX)

CHECK_FUNCTION_EXISTS(timegm HAVE_TIMEGM)
CHECK_FUNCTION_EXISTS(_mkgmtime HAVE_MKGMTIME)
CHECK_FUNCTION_EXISTS(alloca HAVE_ALLOCA_FUNC)
CHECK_FUNCTION_EXISTS(gettid HAVE_GETTID)
CHECK_FUNCTION_EXISTS(sincos HAVE_SINCOS)
CHECK_FUNCTION_EXISTS(strtok_r HAVE_STRTOK_R)
CHECK_FUNCTION_EXISTS(_aligned_malloc HAVE_ALIGNED_MALLOC)

#  This seems not to work and is more complex than it looks at first sight... :-(
#set(CMAKE_REQUIRED_INCLUDES "windows.h")
#set(CMAKE_REQUIRED_LIBRARIES kernel32)
#CHECK_FUNCTION_EXISTS(OpenThread HAVE_OPENTHREAD)

if(MSVC AND NOT MSVC6 AND NOT MSVC7)
	set(HAVE_OPENTHREAD 1)
else(MSVC AND NOT MSVC6 AND NOT MSVC7)
	set(HAVE_OPENTHREAD 0)
endif(MSVC AND NOT MSVC6 AND NOT MSVC7)


CHECK_INCLUDE_FILE("alloca.h" HAVE_ALLOCA_H)
CHECK_INCLUDE_FILE("linux/serial.h" HAVE_LINUX_SERIAL_H)
CHECK_INCLUDE_FILE("linux/input.h" HAVE_LINUX_INPUT_H)
CHECK_INCLUDE_FILE("malloc.h" HAVE_MALLOC_H)
CHECK_INCLUDE_FILE("malloc/malloc.h" HAVE_MALLOC_MALLOC_H)

if(HAVE_ALLOCA_FUNC OR HAVE_ALLOCA_H)
	set(HAVE_ALLOCA 1)
endif(HAVE_ALLOCA_FUNC OR HAVE_ALLOCA_H)

if(CMAKE_MRPT_HAS_GLUT_SYSTEM)
	set(HAVE_FREEGLUT_EXT_H 0)
	find_file(FREEGLUTEXT_HFILE GL/freeglut_ext.h)
	if(FREEGLUTEXT_HFILE)
		mark_as_advanced(FREEGLUTEXT_HFILE)
		set(HAVE_FREEGLUT_EXT_H 1)
	endif(FREEGLUTEXT_HFILE)
else(CMAKE_MRPT_HAS_GLUT_SYSTEM)
	set(HAVE_FREEGLUT_EXT_H 1)
endif(CMAKE_MRPT_HAS_GLUT_SYSTEM)

CHECK_INCLUDE_FILE("stdint.h" HAVE_STDINT_H)
CHECK_INCLUDE_FILE("inttypes.h" HAVE_INTTYPES_H)
CHECK_INCLUDE_FILE("winsock2.h" HAVE_WINSOCK2_H)

# Yes: This is god damn of a hack, but seems to be the only way to properly detect winusb.h (Windows SDK) from CMake:
CHECK_INCLUDE_FILE("windows.h>\n#include <winusb.h" HAVE_WINUSB_H)

# If we want SSE2, check for the expected headers:
if (CMAKE_MRPT_HAS_SSE2)
	CHECK_INCLUDE_FILE("emmintrin.h" HAVE_EMMINTRIN_H)
	CHECK_INCLUDE_FILE("mmintrin.h"  HAVE_MMINTRIN_H)

	# If the headers are not found, disable optimizations:
	if (NOT HAVE_MMINTRIN_H OR NOT HAVE_EMMINTRIN_H)
		set(CMAKE_MRPT_HAS_SSE2 0)
	endif(NOT HAVE_MMINTRIN_H OR NOT HAVE_EMMINTRIN_H)
endif(CMAKE_MRPT_HAS_SSE2)

# If we want SSE3, check for the expected headers:
if (CMAKE_MRPT_HAS_SSE3)

	# JL: Before the CHECK_INCLUDE_FILE() we need to temporarily enable
	#  the -msse3 flag in GCC or the test program that CMake builds will
	#  always fail even if the header is present:
	if(CMAKE_COMPILER_IS_GNUCXX)
		set(TEMP_BACKUP_CMAKE_C_FLAGS ${CMAKE_C_FLAGS})
		set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -msse3")
	endif(CMAKE_COMPILER_IS_GNUCXX)

	CHECK_INCLUDE_FILE("pmmintrin.h" HAVE_PMMINTRIN_H)

	# Restore from above:
	if(CMAKE_COMPILER_IS_GNUCXX)
		set(CMAKE_C_FLAGS ${TEMP_BACKUP_CMAKE_C_FLAGS})
	endif(CMAKE_COMPILER_IS_GNUCXX)

	# If the headers are not found, disable optimizations:
	if (NOT HAVE_PMMINTRIN_H)
		set(CMAKE_MRPT_HAS_SSE3 0)
	endif (NOT HAVE_PMMINTRIN_H)
endif(CMAKE_MRPT_HAS_SSE3)


# Compiler type sizes:
check_type_size("long double"  HAVE_LONG_DOUBLE)


# ---------------------------------------------------------------
#   detect endian-ness
# ---------------------------------------------------------------
include(TestBigEndian)
TEST_BIG_ENDIAN(CMAKE_MRPT_IS_BIG_ENDIAN)
