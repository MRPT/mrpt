include(../../cmakemodules/AssureCMakeRootFile.cmake) # Avoid user mistake in CMake source directory

project(MRPT_ZLIB)

set(ZLIB_SRCS
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

include_directories("${MRPT_SOURCE_DIR}/otherlibs/zlib")

# Always build as a static lib:
if(BUILD_SHARED_LIBS AND NOT WIN32)
	# DYNAMIC LIBS
	set(LIBTYPE SHARED)
else()
	# STATIC LIBS
	set(LIBTYPE STATIC)
endif()

add_library(mrpt-zlib ${LIBTYPE} ${ZLIB_SRCS})

set_target_properties(mrpt-zlib PROPERTIES PROJECT_LABEL lib-mrpt-zlib)

# For dynamic link numbering convenions in Linux:
set_target_properties(mrpt-zlib PROPERTIES
	VERSION "${CMAKE_MRPT_VERSION_NUMBER_MAJOR}.${CMAKE_MRPT_VERSION_NUMBER_MINOR}.${CMAKE_MRPT_VERSION_NUMBER_PATCH}"
	SOVERSION ${CMAKE_MRPT_VERSION_NUMBER_MAJOR}.${CMAKE_MRPT_VERSION_NUMBER_MINOR}
	)

if(CMAKE_COMPILER_IS_GNUCXX AND UNIX)
 	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fPIC")
endif()


# The target file will be "-d" for DEBUG, "-" for the rest of configs:
# ----------------------------------------------------------------------------------
set(the_trg mrpt-zlib)

set_target_properties(${the_trg} PROPERTIES
	OUTPUT_NAME ${MRPT_LIB_PREFIX}${the_trg}${MRPT_DLL_VERSION_POSTFIX}
	ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/lib/"
	RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin/")

# make sure the library gets installed
install(TARGETS ${the_trg}
	RUNTIME DESTINATION ${libmrpt_INSTALL_PREFIX}bin
	LIBRARY DESTINATION ${libmrpt_INSTALL_PREFIX}lib${LIB_SUFFIX}
	ARCHIVE DESTINATION ${libmrpt_INSTALL_PREFIX}lib${LIB_SUFFIX})
