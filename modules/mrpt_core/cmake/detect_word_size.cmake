# Detect word size:
if(CMAKE_SIZEOF_VOID_P EQUAL 8)  # Size in bytes!
	set(CMAKE_MRPT_WORD_SIZE 64)
else()
	set(CMAKE_MRPT_WORD_SIZE 32)
endif()
