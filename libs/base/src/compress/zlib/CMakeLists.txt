INCLUDE(../../cmakemodules/AssureCMakeRootFile.cmake) # Avoid user mistake in CMake source directory

PROJECT(MRPT_ZLIB)

SET(ZLIB_SRCS
		adler32.c
		compress.c
		crc32.c
		deflate.c
		gzio.c
		infback.c
		inffast.c
		inflate.c
		inftrees.c
		minigzip.c
		trees.c
		uncompr.c
		zutil.c
		)

INCLUDE_DIRECTORIES("${MRPT_SOURCE_DIR}/otherlibs/zlib")

# Always build as a static lib:
IF(BUILD_SHARED_LIBS AND NOT WIN32)
	# DYNAMIC LIBS
	SET(LIBTYPE SHARED)
ELSE()
	# STATIC LIBS
	SET(LIBTYPE STATIC)
ENDIF()

ADD_LIBRARY(mrpt-zlib ${LIBTYPE} ${ZLIB_SRCS})

SET_TARGET_PROPERTIES(mrpt-zlib PROPERTIES PROJECT_LABEL lib-mrpt-zlib)

# For dynamic link numbering convenions in Linux:
SET_TARGET_PROPERTIES(mrpt-zlib PROPERTIES
	VERSION "${CMAKE_MRPT_VERSION_NUMBER_MAJOR}.${CMAKE_MRPT_VERSION_NUMBER_MINOR}.${CMAKE_MRPT_VERSION_NUMBER_PATCH}"
	SOVERSION ${CMAKE_MRPT_VERSION_NUMBER_MAJOR}.${CMAKE_MRPT_VERSION_NUMBER_MINOR}
	)

IF(CMAKE_COMPILER_IS_GNUCXX AND UNIX)
 	SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fPIC")
ENDIF()


# The target file will be "-d" for DEBUG, "-" for the rest of configs:
# ----------------------------------------------------------------------------------
SET(the_trg mrpt-zlib)

SET_TARGET_PROPERTIES(${the_trg} PROPERTIES
	OUTPUT_NAME ${MRPT_LIB_PREFIX}${the_trg}${MRPT_DLL_VERSION_POSTFIX}
	ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/lib/"
	RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin/")

# make sure the library gets installed
INSTALL(TARGETS ${the_trg}
	RUNTIME DESTINATION ${libmrpt_INSTALL_PREFIX}bin
	LIBRARY DESTINATION ${libmrpt_INSTALL_PREFIX}lib${LIB_SUFFIX}
	ARCHIVE DESTINATION ${libmrpt_INSTALL_PREFIX}lib${LIB_SUFFIX})
