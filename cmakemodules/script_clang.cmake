# Check for the clang library (NOT the LLVM CLANG compiler!!!). 
# (Linux only, in win32 we use built-in version of ConvertUTF.c):
# ===================================================

# In any case we'll have this lib: embedded or system.
SET(CMAKE_MRPT_HAS_CLANG  1)
SET(CMAKE_MRPT_HAS_CLANG_SYSTEM 0)

IF(UNIX)
	find_library(LIBCLANG_LIBRARY NAMES clang)

	IF(LIBCLANG_LIBRARY)
		MARK_AS_ADVANCED(LIBCLANG_LIBRARY)
		SET(CMAKE_MRPT_HAS_CLANG_SYSTEM 1)
	ENDIF(LIBCLANG_LIBRARY)
ELSE(UNIX)
	# In windows we always have this embedded lib.
ENDIF(UNIX)

IF(CMAKE_MRPT_HAS_CLANG_SYSTEM)	
	SET(CLANG_LIBS clang)  #APPEND_MRPT_LIBS(clang)
ENDIF(CMAKE_MRPT_HAS_CLANG_SYSTEM)

