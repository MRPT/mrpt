# ------------------------------------------------------------------------------
#        Mobile Robot Programming Toolkit (MRPT)
#
# Copyright (c) 2005-2025, Jose Luis Blanco-Claraco, contributors (see Git history)
# All rights reserved.
# Released under BSD-3 license. See LICENSE file
# ------------------------------------------------------------------------------

cmake_minimum_required(VERSION 3.16)

# Tell CMake we'll use C++ for use in its tests/flags
project(mrpt_io LANGUAGES C CXX)

# MRPT CMake scripts: "mrpt_xxx()"
find_package(mrpt_common REQUIRED)
find_package(mrpt_system REQUIRED)

include(./cmake/script_zlib.cmake REQUIRED)        # Check for zlib

# Embedded version of zlib, if really needed:
# Only include zlib if not available as system library OR it's already in wxWidgets:
if(CMAKE_MRPT_HAS_ZLIB)
  if(CMAKE_MRPT_HAS_ZLIB_SYSTEM_IS_WX OR NOT CMAKE_MRPT_HAS_ZLIB_SYSTEM)
    include_directories(${CMAKE_CURRENT_SOURCE_DIR}/3rdparty/zlib/)
    if(NOT MSVC OR NOT CMAKE_MRPT_HAS_ZLIB_SYSTEM_IS_WX)
      set(io_EXTRA_SRCS
        3rdparty/zlib/adler32.c
        3rdparty/zlib/compress.c
        3rdparty/zlib/crc32.c
        3rdparty/zlib/deflate.c
        3rdparty/zlib/gzclose.c
        3rdparty/zlib/gzlib.c
        3rdparty/zlib/gzread.c
        3rdparty/zlib/gzwrite.c
        3rdparty/zlib/infback.c
        3rdparty/zlib/inffast.c
        3rdparty/zlib/inflate.c
        3rdparty/zlib/inftrees.c
        3rdparty/zlib/trees.c
        3rdparty/zlib/uncompr.c
        3rdparty/zlib/zutil.c
      )
    endif()
  endif()
endif()


# define lib:
set(LIB_SRCS
  src/CStream.cpp
  src/CMemoryStream.cpp
  src/CMemoryStream_unittest.cpp
  src/CFileOutputStream.cpp
  src/vector_loadsave_unittest.cpp
  src/CPipe.cpp
  src/CTextFileLinesParser.cpp
  src/CTextFileLinesParser_unittest.cpp
  src/CFileStream.cpp
  src/CFileGZOutputStream.cpp
  src/zip.cpp
  src/CFileInputStream.cpp
  src/zip_unittest.cpp
  src/lazy_load_path.cpp
  src/CFileGZInputStream.cpp
  src/vector_loadsave.cpp
)

# These tests are moved to mrpt_viz since that's the lowest-level module that includes all dependencies.
#src/CSerializable_unittest.cpp
#src/bimap_serialize_unittest.cpp
#src/stl_serialize_unittest.cpp
#src/CFileGZStreams_unittest.cpp 


set(LIB_PUBLIC_HDRS
  include/mrpt/io/CTextFileLinesParser.h
  include/mrpt/io/CFileGZOutputStream.h
  include/mrpt/io/zip.h
  include/mrpt/io/CFileInputStream.h
  include/mrpt/io/CPipe.h
  include/mrpt/io/registerAllClasses.h
  include/mrpt/io/CStream.h
  include/mrpt/io/vector_loadsave.h
  include/mrpt/io/lazy_load_path.h
  include/mrpt/io/CFileStream.h
  include/mrpt/io/CFileGZInputStream.h
  include/mrpt/io/csv.h
  include/mrpt/io/CFileOutputStream.h
  include/mrpt/io/open_flags.h
  include/mrpt/io/CMemoryStream.h
)

mrpt_add_library(
  TARGET ${PROJECT_NAME}
  SOURCES ${LIB_SRCS} ${LIB_PUBLIC_HDRS}
    # Add the optional extra sources:
    ${io_EXTRA_SRCS}
  PUBLIC_LINK_LIBRARIES
    mrpt::mrpt_system
#  PRIVATE_LINK_LIBRARIES
  CMAKE_DEPENDENCIES
    mrpt_system
  # Other imported targets:
)

if(CMAKE_MRPT_HAS_ZLIB)
  if(NOT CMAKE_MRPT_HAS_ZLIB_SYSTEM_IS_WX AND CMAKE_MRPT_HAS_ZLIB_SYSTEM)
      target_link_libraries(${PROJECT_NAME} PRIVATE ${ZLIB_LIBRARIES})
    target_include_directories(${PROJECT_NAME} PRIVATE ${ZLIB_INCLUDE_DIR})
  endif()

  if (NOT "${io_EXTRA_SRCS}" STREQUAL "")
    if (HAVE_UNISTD_H)
      set(extra_flags2_ "-include unistd.h")
    endif()

    set_source_files_properties(
      ${io_EXTRA_SRCS}
      PROPERTIES COMPILE_FLAGS "${extra_flags_} ${extra_flags2_}"
      )
    unset(extra_flags_)
    unset(extra_flags2_)
  endif()
endif()

# Use wxWidgets version of libzip (gz* functions)
if(MSVC AND CMAKE_MRPT_HAS_WXWIDGETS)
    target_link_libraries(${PROJECT_NAME} PRIVATE imp_wxwidgets)
endif()
