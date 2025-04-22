# ------------------------------------------------------------------------------
#        A Modular Optimization framework for Localization and mApping
#                               (MRPT)
#
# Copyright (C) 2018-2025, Jose Luis Blanco-Claraco, contributors (AUTHORS.md)
# All rights reserved.
# Released under BSD-3. See LICENSE file
# ------------------------------------------------------------------------------

# This file defines utility CMake functions to ensure uniform settings all
# across MRPT modules, programs, and tests.
#
# Usage:
#
# find_package(mrpt_common REQUIRED) # this includes mrpt_cmake_functions.cmake
#

include(GNUInstallDirs) # for install dirs in multilib
include(CMakePackageConfigHelpers)

find_package(GTest QUIET)

# Build static or dynamic libs?
# ===================================================
# Default: dynamic libraries (except if building to JavaScript code)
if ("${CMAKE_SYSTEM_NAME}" STREQUAL "Emscripten")
	set(_def_value OFF)
else()
	set(_def_value ON)
endif()
set(BUILD_SHARED_LIBS ${_def_value} CACHE BOOL "Build shared libraries (.dll/.so) instead of static ones (.lib/.a)")
unset(_def_value)

if(BUILD_SHARED_LIBS)
	set(CMAKE_MRPT_BUILD_SHARED_LIB "#define MRPT_BUILT_AS_DLL")
	set(CMAKE_MRPT_BUILD_SHARED_LIB_ONOFF 1)
else()
	set(CMAKE_MRPT_BUILD_SHARED_LIB "/* #define MRPT_BUILT_AS_DLL */")
	set(CMAKE_MRPT_BUILD_SHARED_LIB_ONOFF 0)
endif()


# ccache:
if(NOT MSVC AND NOT XCODE_VERSION)
    option(MRPT_BUILD_WITH_CCACHE "Use ccache compiler cache" ON)
    find_program(CCACHE_FOUND ccache)
    mark_as_advanced(CCACHE_FOUND)
    if(CCACHE_FOUND)
        if(MRPT_BUILD_WITH_CCACHE)
            set_property(GLOBAL PROPERTY RULE_LAUNCH_COMPILE ccache)
            set_property(GLOBAL PROPERTY RULE_LAUNCH_LINK ccache)
        else()
            set_property(GLOBAL PROPERTY RULE_LAUNCH_COMPILE "")
            set_property(GLOBAL PROPERTY RULE_LAUNCH_LINK "")
        endif()
    endif()
endif()

# mrpt_foo => foo
function(strip_mrpt_name TARGETNAME OUTPUT_VAR)
    set(name "${TARGETNAME}")
    if(name MATCHES "^mrpt_(.+)")
        string(REGEX REPLACE "^mrpt_" "" name "${name}")
    endif()
    set(${OUTPUT_VAR} "${name}" PARENT_SCOPE)
endfunction()

# Project version:
if (mrpt_common_VERSION)  # If installed via colcon+ament
  set(MRPT_VERSION_NUMBER_MAJOR ${mrpt_common_VERSION_MAJOR})
  set(MRPT_VERSION_NUMBER_MINOR ${mrpt_common_VERSION_MINOR})
  set(MRPT_VERSION_NUMBER_PATCH ${mrpt_common_VERSION_PATCH})
else() # Installed without ament:
  include(${CMAKE_CURRENT_LIST_DIR}/mrpt-version.cmake)
endif()

set(_MRPTCOMMON_MODULE_BASE_DIR "${CMAKE_CURRENT_LIST_DIR}")

if (NOT DEFINED MRPT_VERSION_NUMBER_MAJOR)
  message(ERROR "MRPT_VERSION_NUMBER_MAJOR not defined: use `find_package(mrpt_common)`")
endif()

# Avoid the need for DLL export/import macros in Windows:
if (WIN32)
  set(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS  ON)
endif()

# Detect wordsize:
if(CMAKE_SIZEOF_VOID_P EQUAL 8)  # Size in bytes!
  set(MRPT_WORD_SIZE 64)
else()
  set(MRPT_WORD_SIZE 32)
endif()

# Default output dirs for libs:
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/lib/")
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY  "${CMAKE_BINARY_DIR}/lib/")
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin/")

# Compiler ID:
if (MSVC)
  # 1700 = VC 11.0 (2012)
  # 1800 = VC 12.0 (2013)
  #           ... (13 was skipped!)
  # 1900 = VC 14.0 (2015)
  # 1910 = VC 14.1 (2017)
  math(EXPR MSVC_VERSION_3D "(${MSVC_VERSION}/10)-60")
  if (MSVC_VERSION_3D GREATER 120)
    math(EXPR MSVC_VERSION_3D "${MSVC_VERSION_3D}+10")
  endif()
  set(MRPT_COMPILER_NAME "msvc${MSVC_VERSION_3D}")
else()
  set(MRPT_COMPILER_NAME "${CMAKE_CXX_COMPILER_ID}")
endif()

# Build DLL full name:
if (WIN32)
  set(MRPT_DLL_VERSION_POSTFIX
    "${MRPT_VERSION_NUMBER_MAJOR}${MRPT_VERSION_NUMBER_MINOR}${MRPT_VERSION_NUMBER_PATCH}_${MRPT_COMPILER_NAME}_x${MRPT_WORD_SIZE}")
  if ($ENV{VERBOSE})
    message(STATUS "Using DLL version postfix: ${MRPT_DLL_VERSION_POSTFIX}")
  endif()
else()
  set(MRPT_DLL_VERSION_POSTFIX "")
endif()

# Group projects in "folders"
set_property(GLOBAL PROPERTY USE_FOLDERS ON)
set_property(GLOBAL PROPERTY PREDEFINED_TARGETS_FOLDER "CMakeTargets")

# We want libraries to be named "libXXX" in all compilers, "libXXX-dbg" in MSVC
set(CMAKE_SHARED_LIBRARY_PREFIX "lib")
set(CMAKE_IMPORT_LIBRARY_PREFIX "lib")
set(CMAKE_STATIC_LIBRARY_PREFIX "lib")
set(CMAKE_DEBUG_POSTFIX "-dbg")


# GNU GCC options ================================
if(CMAKE_COMPILER_IS_GNUCXX)
  # BUILD_TYPE: SanitizeAddress
  set(CMAKE_CXX_FLAGS_SANITIZEADDRESS "-fsanitize=address  -fsanitize=leak -g")
  set(CMAKE_EXE_LINKER_FLAGS_SANITIZEADDRESS "-fsanitize=address  -fsanitize=leak")
  set(CMAKE_SHARED_LINKER_FLAGS_SANITIZEADDRESS "-fsanitize=address  -fsanitize=leak")

  # BUILD_TYPE: SanitizeThread
  set(CMAKE_CXX_FLAGS_SANITIZETHREAD "-fsanitize=thread -g")
  set(CMAKE_EXE_LINKER_FLAGS_SANITIZETHREAD "-fsanitize=thread")
  set(CMAKE_SHARED_LINKER_FLAGS_SANITIZETHREAD "-fsanitize=thread")
endif()

# -----------------------------------------------------------------------------
# find_mrpt_package(package_name)
#
# Does nothing if the target is known at build time, or issues the corresponding
# standard CMake find_package().
# -----------------------------------------------------------------------------
function(find_mrpt_package PACKAGE_NAME)
    if (NOT TARGET ${PACKAGE_NAME})
        mrpt_message_verbose("find_mrpt_package: ${PACKAGE_NAME} not a known target, using find_package().")
        find_package(${PACKAGE_NAME} REQUIRED)
      else()
        mrpt_message_verbose("find_mrpt_package: ${PACKAGE_NAME} is a known target, doing nothing.")
    endif()
    #TODO: use mrpt namespace (mrpt::xxx)?
endfunction()

# -----------------------------------------------------------------------------
# mrpt_set_target_cxx17(target)
#
# Enabled C++17 for the given target
# -----------------------------------------------------------------------------
function(mrpt_set_target_cxx17 TARGETNAME)
  target_compile_features(${TARGETNAME} PUBLIC cxx_std_17)
  if (MSVC)
    # this seems to be required in addition to the cxx_std_17 above (?)
    target_compile_options(${TARGETNAME} PUBLIC /std:c++latest)
  endif()
endfunction()

# -----------------------------------------------------------------------------
# mrpt_set_target_build_options(target)
#
# Set defaults for each MRPT cmake target
# -----------------------------------------------------------------------------
function(mrpt_set_target_build_options TARGETNAME HEADERS_ONLY_LIBRARY)
  # Build for C++17
  mrpt_set_target_cxx17(${TARGETNAME})

  # Warning level:
  if (MSVC)
    # msvc:
    target_compile_options(${TARGETNAME} PRIVATE /W3)
    target_compile_definitions(${TARGETNAME} PRIVATE
      _CRT_SECURE_NO_DEPRECATE
      _CRT_NONSTDC_NO_DEPRECATE
      _SILENCE_ALL_CXX17_DEPRECATION_WARNINGS
    )
  else()
    # gcc & clang: C and C++:
    target_compile_options(${TARGETNAME} PRIVATE
      -Wall
      -Wextra
      -Wshadow
      -Werror=return-type # error on missing return();
      -Wtype-limits
      -Wcast-align
      -Wparentheses
      -Wunused
      -Wpedantic
      -Wconversion
      -Wsign-conversion
      -Wdouble-promotion
      -fPIC
      )
    # C++:
    target_compile_options(${TARGETNAME}
      PRIVATE $<$<COMPILE_LANGUAGE:CXX>: -Woverloaded-virtual -Wold-style-cast -Wnon-virtual-dtor>
    )
  endif()

  # Optimization:
  # -------------------------
  if((NOT MSVC) AND (NOT CMAKE_CROSSCOMPILING))
    option(MRPT_BUILD_MARCH_NATIVE "Build with `-march=\"native\"`" OFF)

    if (MRPT_BUILD_MARCH_NATIVE)
      # Note 1: GTSAM must be built with identical flags to avoid crashes.
      #  We will use this cmake variable too to populate GTSAM_BUILD_WITH_MARCH_NATIVE
      # Note 2: We must set "march=native" PUBLIC to avoid crashes with Eigen in derived projects
      target_compile_options(${TARGETNAME} PUBLIC -march=native)
    endif()

    if (NOT CMAKE_BUILD_TYPE STREQUAL "Debug")
      target_compile_options(${TARGETNAME} PRIVATE -O3)
    endif()
  endif()
endfunction()

# -----------------------------------------------------------------------------
# mrpt_configure_library(target HEADERS_ONLY ADDITIONAL_EXPORTS [CMAKE_DEPS...])
#
# Define a consistent install behavior for cmake-based library project:
# -----------------------------------------------------------------------------
function(mrpt_configure_library TARGETNAME HEADERS_ONLY_LIBRARY ADDITIONAL_EXPORT_TARGETS CMAKE_DEPS)

  message(STATUS "HEADERS_ONLY_LIBRARY: ${HEADERS_ONLY_LIBRARY}")

  # Public hdrs interface:
  if (HEADERS_ONLY_LIBRARY)
    target_include_directories(${TARGETNAME} INTERFACE
      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
      $<INSTALL_INTERFACE:include>
    )
  else()
    target_include_directories(${TARGETNAME} PUBLIC
      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
      $<INSTALL_INTERFACE:include>
      PRIVATE src
    )
  endif()


  if (NOT HEADERS_ONLY_LIBRARY)
    # Dynamic libraries output options:
    # -----------------------------------
    set_target_properties(${TARGETNAME} PROPERTIES
      OUTPUT_NAME "${TARGETNAME}${MRPT_DLL_VERSION_POSTFIX}"
      COMPILE_PDB_NAME "${TARGETNAME}${MRPT_DLL_VERSION_POSTFIX}"
      COMPILE_PDB_NAME_DEBUG "${TARGETNAME}${MRPT_DLL_VERSION_POSTFIX}${CMAKE_DEBUG_POSTFIX}"
      VERSION "${MRPT_VERSION_NUMBER_MAJOR}.${MRPT_VERSION_NUMBER_MINOR}.${MRPT_VERSION_NUMBER_PATCH}"
      SOVERSION ${MRPT_VERSION_NUMBER_MAJOR}.${MRPT_VERSION_NUMBER_MINOR}
      )
  endif()

  # Project "folder":
  # -------------------
  set_target_properties(${TARGETNAME} PROPERTIES FOLDER "MRPT-modules")

  # Install lib:
  if (NOT HEADERS_ONLY_LIBRARY)
    install(TARGETS ${TARGETNAME} ${ADDITIONAL_EXPORT_TARGETS}
        EXPORT ${TARGETNAME}-targets
        ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
        LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
        RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
      )
  else()
    install(TARGETS ${TARGETNAME} EXPORT ${TARGETNAME}-targets)
  endif()

  # Install hdrs:
  if (EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/include/)
    install(
      DIRECTORY include/
      DESTINATION ${CMAKE_INSTALL_PREFIX}/include
    )
  endif()

  strip_mrpt_name(${TARGETNAME} MRPT_LIB_NAME)

  # Create module/config.h file
  if (EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/config.h.in)
    set(MODULES_BASE_CONFIG_INCLUDES_DIR ${CMAKE_BINARY_DIR}/include/mrpt-configuration/)
    set(MODULE_CONFIG_FILE_INCLUDE_DIR ${MODULES_BASE_CONFIG_INCLUDES_DIR}/mrpt/${MRPT_LIB_NAME})
    file(MAKE_DIRECTORY ${MODULE_CONFIG_FILE_INCLUDE_DIR})

    configure_file(${CMAKE_CURRENT_SOURCE_DIR}/config.h.in ${MODULE_CONFIG_FILE_INCLUDE_DIR}/config.h)

    target_include_directories(${TARGETNAME} PUBLIC
      $<BUILD_INTERFACE:${MODULES_BASE_CONFIG_INCLUDES_DIR}>
    )
    # And install it:
    install(
      FILES ${MODULE_CONFIG_FILE_INCLUDE_DIR}/config.h
      DESTINATION ${CMAKE_INSTALL_PREFIX}/include/mrpt/${MRPT_LIB_NAME}/
    )
    unset(MODULES_BASE_CONFIG_INCLUDES_DIR)
  endif()

  # make project importable from build_dir:
  export(
    TARGETS ${TARGETNAME}
    # export to ROOT cmake directory (when building MRPT as a superproject)
    FILE ${CMAKE_BINARY_DIR}/${TARGETNAME}-targets.cmake
    NAMESPACE mrpt::
  )

  # Add alias to use the namespaced name within local builds from source:
  #add_library(mrpt::${TARGETNAME} ALIAS ${TARGETNAME})

  # And generate the -config.cmake file:
  set(ALL_DEPS_LIST "${CMAKE_DEPS}") # used in xxx-config.cmake.in
  set(MRPT_MODULE_NAME ${TARGETNAME})
  configure_file(
    "${_MRPTCOMMON_MODULE_BASE_DIR}/mrpt-xxx-config.cmake.in"
    "${CMAKE_BINARY_DIR}/${TARGETNAME}-config.cmake" IMMEDIATE @ONLY
  )
  # Version file:
  write_basic_package_version_file(
    "${CMAKE_BINARY_DIR}/${TARGETNAME}-config-version.cmake"
    VERSION ${MRPT_VERSION_NUMBER_MAJOR}.${MRPT_VERSION_NUMBER_MINOR}.${MRPT_VERSION_NUMBER_PATCH}
    COMPATIBILITY AnyNewerVersion
  )

  # Regular arch-dep libraries get to LIBDIR (/usr/lib), while
  # arch-indep (headers-only) go to DATADIR (/usr/share):
  #if (${headers_only})
  #  set(LIB_TARGET_INSTALL_DEST ${CMAKE_INSTALL_DATADIR})
  #else()
  #  set(LIB_TARGET_INSTALL_DEST ${CMAKE_INSTALL_LIBDIR})
  #endif()


  # Install cmake config module
  install(
    EXPORT
      ${TARGETNAME}-targets
    DESTINATION
      ${CMAKE_INSTALL_LIBDIR}/${TARGETNAME}/cmake
    NAMESPACE mrpt::
  )
  install(
    FILES
      ${CMAKE_BINARY_DIR}/${TARGETNAME}-config.cmake
      ${CMAKE_BINARY_DIR}/${TARGETNAME}-config-version.cmake
    DESTINATION
      ${CMAKE_INSTALL_LIBDIR}/${TARGETNAME}/cmake
  )
endfunction()

# -----------------------------------------------------------------------------
# mrpt_configure_app(target)
#
# Define common properties of cmake-based executable projects:
# -----------------------------------------------------------------------------
function(mrpt_configure_app TARGETNAME)
  # Project "folder":
  set_target_properties(${TARGETNAME} PROPERTIES FOLDER "MRPT-apps")

  #TODO: install

endfunction()



# -----------------------------------------------------------------------------
# mrpt_message_verbose(...)
# Maps to `message(STATUS ...)` if the environment variable VERBOSE is !=0.
# Otherwise, does nothing.
# -----------------------------------------------------------------------------
function(mrpt_message_verbose)
  if ($ENV{VERBOSE})
    message(STATUS ${ARGN})
  endif()
endfunction()

# Example of usage: 
#  remove_matching_files_from_list(".*_LIN.cpp" my_srcs)
#
macro(remove_matching_files_from_list match_expr lst_files)
	set(lst_files_aux "")
	foreach(FIL ${${lst_files}})
		if(NOT ${FIL} MATCHES "${match_expr}")
			set(lst_files_aux "${lst_files_aux}" "${FIL}")
		endif()
	endforeach()
	set(${lst_files} ${lst_files_aux})
endmacro()

macro(keep_matching_files_from_list match_expr lst_files)
	set(lst_files_aux "")
	foreach(FIL ${${lst_files}})
		if(${FIL} MATCHES "${match_expr}")
			set(lst_files_aux "${lst_files_aux}" "${FIL}")
		endif()
	endforeach()
	set(${lst_files} ${lst_files_aux})
endmacro()

# handle_special_simd_flags(): Add custom flags to a set of source files
# Only for Intel-compatible archs
#-----------------------------------------------------------------------
function(handle_special_simd_flags lst_files FILE_PATTERN FLAGS_TO_ADD)
	if (MRPT_COMPILER_IS_GCC_OR_CLANG AND MRPT_ARCH_INTEL_COMPATIBLE)
		set(_lst ${lst_files})
		KEEP_MATCHING_FILES_FROM_LIST(${FILE_PATTERN} _lst)
		if(NOT "${_lst}" STREQUAL "")
			set_source_files_properties(${_lst} PROPERTIES COMPILE_FLAGS "${FLAGS_TO_ADD}")
		endif()
	endif()
endfunction()


# -----------------------------------------------------------------------------
# mrpt_add_library(
#	TARGET name
#	SOURCES ${SRC_FILES}
# [HEADERS_ONLY_LIBRARY]
#	[PUBLIC_LINK_LIBRARIES lib1 lib2]
#	[PRIVATE_LINK_LIBRARIES lib3 lib4]
# [CMAKE_DEPENDENCIES pkg1 pkg2]
# [ADDITIONAL_EXPORT_TARGETS target1 target2]
#	)
#
# Defines a MRPT library. `CMAKE_DEPENDENCIES` enumerates those packages
# that needs to be find_package'd in this library's xxx-config.cmake file.
# -----------------------------------------------------------------------------
function(mrpt_add_library)
    set(options HEADERS_ONLY_LIBRARY)
    set(oneValueArgs TARGET)
    set(multiValueArgs SOURCES PUBLIC_LINK_LIBRARIES PRIVATE_LINK_LIBRARIES CMAKE_DEPENDENCIES ADDITIONAL_EXPORT_TARGETS)
    cmake_parse_arguments(MRPT_ADD_LIBRARY "${options}" "${oneValueArgs}" "${multiValueArgs}" ${ARGN})

    # Remove _LIN files when compiling under Windows, and _WIN files when compiling under Linux.
    if(WIN32)
      remove_matching_files_from_list(".*_LIN.cpp" MRPT_ADD_LIBRARY_SOURCES)		# Win32
    else()
      remove_matching_files_from_list(".*_WIN.cpp" MRPT_ADD_LIBRARY_SOURCES)		# Apple & Unix
    endif()

    # Keep a list of unit testing files, for declaring them in /test:
    set(lst_unittests ${MRPT_ADD_LIBRARY_SOURCES})
    keep_matching_files_from_list(".*_unittest.cpp" lst_unittests)
    if(NOT "${lst_unittests}" STREQUAL "")
      # We have unit tests:
      mrpt_add_test(
        TARGET test_${MRPT_ADD_LIBRARY_TARGET}
        SOURCES ${lst_unittests}
        LINK_LIBRARIES ${MRPT_ADD_LIBRARY_TARGET}
      )
    endif()

    # Enable SIMD especial instructions in especialized source files, even if
    # those instructions are NOT enabled globally for the entire build:
    handle_special_simd_flags("${MRPT_ADD_LIBRARY_SOURCES}" ".*\.SSE2.cpp"  "-msse2")
    handle_special_simd_flags("${MRPT_ADD_LIBRARY_SOURCES}" ".*\.SSSE3.cpp"  "-msse3 -mssse3")
    handle_special_simd_flags("${MRPT_ADD_LIBRARY_SOURCES}" ".*\.AVX.cpp"  "-mavx")
    handle_special_simd_flags("${MRPT_ADD_LIBRARY_SOURCES}" ".*\.AVX2.cpp"  "-mavx2")

    # Don't include here the unit testing code:
    remove_matching_files_from_list(".*_unittest.cpp" MRPT_ADD_LIBRARY_SOURCES)

    # Library type: 
    # - HEADERS_ONLY_LIBRARY: `INTERFACE`
    # - Regular lib: (none: default to SHARED / STATIC)

    # Create library target:
    if (NOT MRPT_ADD_LIBRARY_HEADERS_ONLY_LIBRARY)
      # Regular library:
      add_library(${MRPT_ADD_LIBRARY_TARGET}
        ${MRPT_ADD_LIBRARY_SOURCES}
      )
      #add_coverage(${name})

      mrpt_set_target_build_options(${MRPT_ADD_LIBRARY_TARGET} ${MRPT_ADD_LIBRARY_HEADERS_ONLY_LIBRARY})

    else()
      # A hdr-only library: needs no real compiling
      add_library(${MRPT_ADD_LIBRARY_TARGET} INTERFACE)

      # List of hdr files (for editing in IDEs,etc.):
      #target_sources(${MRPT_ADD_LIBRARY_TARGET} INTERFACE ${MRPT_ADD_LIBRARY_SOURCES})
    endif()

    # Define common flags:
    #mrpt_configure_library(target HEADERS_ONLY ADDITIONAL_EXPORTS [CMAKE_DEPS...])
    mrpt_configure_library(${MRPT_ADD_LIBRARY_TARGET} "${MRPT_ADD_LIBRARY_HEADERS_ONLY_LIBRARY}" "${MRPT_ADD_LIBRARY_ADDITIONAL_EXPORT_TARGETS}" "${MRPT_ADD_LIBRARY_CMAKE_DEPENDENCIES}")

    # lib Dependencies:
    target_link_libraries(${MRPT_ADD_LIBRARY_TARGET}
      PUBLIC
      ${MRPT_ADD_LIBRARY_PUBLIC_LINK_LIBRARIES}
    )
    target_link_libraries(${MRPT_ADD_LIBRARY_TARGET}
      PRIVATE
      ${MRPT_ADD_LIBRARY_PRIVATE_LINK_LIBRARIES}
    )

   #TODO: install

endfunction()

# -----------------------------------------------------------------------------
# mrpt_add_executable(
#	TARGET name
#	SOURCES ${SRC_FILES}
#	[LINK_LIBRARIES lib1 lib2]
#	)
#
# Defines a MRPT executable
# -----------------------------------------------------------------------------
function(mrpt_add_executable)
    set(options DONT_INSTALL)
    set(oneValueArgs TARGET)
    set(multiValueArgs SOURCES LINK_LIBRARIES)
    cmake_parse_arguments(MRPT_ADD_EXECUTABLE "${options}" "${oneValueArgs}" "${multiValueArgs}" ${ARGN})

    add_executable(${MRPT_ADD_EXECUTABLE_TARGET}
      ${MRPT_ADD_EXECUTABLE_SOURCES}
    )

    # Define common flags:
    mrpt_set_target_build_options(${MRPT_ADD_EXECUTABLE_TARGET} FALSE) # FALSE: no headers-only target
    mrpt_configure_app(${MRPT_ADD_EXECUTABLE_TARGET})

    # lib Dependencies:
    if (MRPT_ADD_EXECUTABLE_LINK_LIBRARIES)
      target_link_libraries(
      ${MRPT_ADD_EXECUTABLE_TARGET}
      ${MRPT_ADD_EXECUTABLE_LINK_LIBRARIES}
      )
    endif()

    # install:
    if (NOT MRPT_ADD_EXECUTABLE_DONT_INSTALL)
      install(TARGETS ${MRPT_ADD_EXECUTABLE_TARGET}
        ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
        LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
        RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
    )
    endif()
endfunction()


# -----------------------------------------------------------------------------
# mrpt_add_test(
#	TARGET name
#	SOURCES ${SRC_FILES}
#	[LINK_LIBRARIES lib1 lib2]
#	)
#
# Defines a MRPT unit test
# -----------------------------------------------------------------------------
function(mrpt_add_test)
    set(oneValueArgs TARGET)
    set(multiValueArgs SOURCES LINK_LIBRARIES)
    cmake_parse_arguments(MRPT_ADD_TEST "" "${oneValueArgs}" "${multiValueArgs}" ${ARGN})

    enable_testing()
    add_executable(${MRPT_ADD_TEST_TARGET}
      ${MRPT_ADD_TEST_SOURCES}
      ${mrpt_common_DIR}/../common_sources/mrpt_test_main.cpp
    )

    # Define common flags:
    mrpt_set_target_build_options(${MRPT_ADD_TEST_TARGET} FALSE) # FALSE: no headers-only target
    mrpt_configure_app(${MRPT_ADD_TEST_TARGET})

    # lib Dependencies:
    target_link_libraries(
      ${MRPT_ADD_TEST_TARGET}
      ${MRPT_ADD_TEST_LINK_LIBRARIES}
      Threads::Threads
      GTest::GTest
    )

    # Make common_headers visible:
    target_include_directories(${MRPT_ADD_TEST_TARGET} PRIVATE ${mrpt_common_DIR}/../common_headers/)

    # Macro for source dir path:
    if (NOT "${CMAKE_SYSTEM_NAME}" STREQUAL "Emscripten")
      target_compile_definitions(${MRPT_ADD_TEST_TARGET} PRIVATE CMAKE_UNITTEST_BASEDIR="${CMAKE_SOURCE_DIR}")
    else()
      # Use relative paths for embedded files:
      target_compile_definitions(${MRPT_ADD_TEST_TARGET} PRIVATE CMAKE_UNITTEST_BASEDIR=".")
    endif()
  
    # Run it:
    #add_custom_target(run_${MRPT_ADD_TEST_TARGET} COMMAND $<TARGET_FILE:${MRPT_ADD_TEST_TARGET}>)
    add_test(${MRPT_ADD_TEST_TARGET}_build "${CMAKE_COMMAND}" --build ${CMAKE_CURRENT_BINARY_DIR} --target ${MRPT_ADD_TEST_TARGET})
    add_test(run_${MRPT_ADD_TEST_TARGET} ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/${MRPT_ADD_TEST_TARGET})
    set_tests_properties(run_${MRPT_ADD_TEST_TARGET} PROPERTIES DEPENDS ${MRPT_ADD_TEST_TARGET}_build)

    add_custom_target(run_${MRPT_ADD_TEST_TARGET} COMMAND ${MRPT_ADD_TEST_TARGET})
    add_dependencies(run_${MRPT_ADD_TEST_TARGET} ${MRPT_ADD_TEST_TARGET})

endfunction()

# -----------------------------------------------------------------------------
# list_subdirectories(retval curdir)
#
# Lists all subdirectories. Code from MRPT.
# -----------------------------------------------------------------------------
function(list_subdirectories retval curdir)
  file(GLOB sub_dir RELATIVE ${curdir} *)
  set(list_of_dirs "")
  foreach(dir ${sub_dir})
    string(SUBSTRING ${dir} 0 1 dir1st)
    if(IS_DIRECTORY ${curdir}/${dir} AND NOT ${dir1st} STREQUAL "." AND NOT ${dir} STREQUAL "CMakeFiles")
        set(list_of_dirs ${list_of_dirs} ${dir})
    endif()
  endforeach()
  set(${retval} ${list_of_dirs} PARENT_SCOPE)
endfunction()

# -----------------------------------------------------------------------------
# mrpt_find_package_or_return(package_name)
#
# Calls find_package(package_name QUIET), and if it is not found, prints a
# descriptive message and call "return()" to exit the current cmake script.
# -----------------------------------------------------------------------------
macro(mrpt_find_package_or_return PACKAGE_NAME)
  find_package(${PACKAGE_NAME} QUIET)
  if (NOT ${PACKAGE_NAME}_FOUND)
    message(WARNING "${PROJECT_NAME}: Skipping due to missing dependency `${PACKAGE_NAME}`")
    return()
  endif()
endmacro()



# Converts a version like "1.2.3" into a string "0x10203",
# or "3.4.19" into "0x30413".
# Usage: mrpt_version_to_hexadecimal(TARGET_VAR "1.2.3")
macro(mrpt_version_to_hexadecimal OUT_VAR IN_VERSION)
  string(REGEX MATCHALL "[0-9]+" VERSION_PARTS "${IN_VERSION}")
  list(GET VERSION_PARTS 0 VERSION_NUMBER_MAJOR)
  list(GET VERSION_PARTS 1 VERSION_NUMBER_MINOR)
  list(GET VERSION_PARTS 2 VERSION_NUMBER_PATCH)

  # Convert each part to hex:
  math(EXPR ${OUT_VAR}
  "(${VERSION_NUMBER_MAJOR} << 16) + \
    (${VERSION_NUMBER_MINOR} << 8) + \
    (${VERSION_NUMBER_PATCH})" OUTPUT_FORMAT HEXADECIMAL )
endmacro()

# Minimize the time and memory required to build and load debug info:
#-----------------------------------------------------------------------
function(mrpt_reduced_debug_symbols TARGET_)
	get_property(CUR_FLAGS_ TARGET ${TARGET_} PROPERTY COMPILE_OPTIONS)
	set(cxxflags_ "$ENV{CXXFLAGS}")
	separate_arguments(cxxflags_)
	if (("-g" IN_LIST CUR_FLAGS_) OR ("-g" IN_LIST cxxflags_))
		if (MRPT_COMPILER_IS_GCC)
			target_compile_options(${TARGET_} PRIVATE -g1)
		elseif(MRPT_COMPILER_IS_CLANG)
			target_compile_options(${TARGET_} PRIVATE -gline-tables-only)
		endif()
	endif()
endfunction()
