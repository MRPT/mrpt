# ------------------------------------------------------------------------------
#        A Modular Optimization framework for Localization and mApping
#                               (MOLA)
#
# Copyright (C) 2018-2025, Jose Luis Blanco-Claraco, contributors (AUTHORS.md)
# All rights reserved.
# Released under BSD-3. See LICENSE file
# ------------------------------------------------------------------------------

include(GNUInstallDirs) # for install dirs in multilib
include(CMakePackageConfigHelpers)

# This file defines utility CMake functions to ensure uniform settings all
# accross MOLA modules, programs, and tests.
# Usage:
#   include(mola_cmake_functions)
#

# Project version:
if (mola_common_VERSION)  # If installed via colcon+ament
  set(MOLA_VERSION_NUMBER_MAJOR ${mola_common_VERSION_MAJOR})
  set(MOLA_VERSION_NUMBER_MINOR ${mola_common_VERSION_MINOR})
  set(MOLA_VERSION_NUMBER_PATCH ${mola_common_VERSION_PATCH})
else() # Installed without ament:
  include(${CMAKE_CURRENT_LIST_DIR}/mola-version.cmake)
endif()

set(_MOLACOMMON_MODULE_BASE_DIR "${CMAKE_CURRENT_LIST_DIR}")

if (NOT DEFINED MOLA_VERSION_NUMBER_MAJOR)
	message(ERROR "MOLA_VERSION_NUMBER_MAJOR not defined: use `find_package(mola_common)`")
endif()

# Avoid the need for DLL export/import macros in Windows:
if (WIN32)
  set(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS  ON)
endif()

# Detect wordsize:
if(CMAKE_SIZEOF_VOID_P EQUAL 8)  # Size in bytes!
  set(MOLA_WORD_SIZE 64)
else()
  set(MOLA_WORD_SIZE 32)
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
  set(MOLA_COMPILER_NAME "msvc${MSVC_VERSION_3D}")
else()
  set(MOLA_COMPILER_NAME "${CMAKE_CXX_COMPILER_ID}")
endif()

# Build DLL full name:
if (WIN32)
  set(MOLA_DLL_VERSION_POSTFIX
    "${MOLA_VERSION_NUMBER_MAJOR}${MOLA_VERSION_NUMBER_MINOR}${MOLA_VERSION_NUMBER_PATCH}_${MOLA_COMPILER_NAME}_x${MOLA_WORD_SIZE}")
  if ($ENV{VERBOSE})
  	message(STATUS "Using DLL version postfix: ${MOLA_DLL_VERSION_POSTFIX}")
  endif()
else()
  set(MOLA_DLL_VERSION_POSTFIX "")
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
# find_mola_package(package_name)
#
# Does nothing if the target is known at build time, or issues the corresponding
# standard CMake find_package().
# -----------------------------------------------------------------------------
function(find_mola_package PACKAGE_NAME)
    if (NOT TARGET ${PACKAGE_NAME})
        mola_message_verbose("find_mola_package: ${PACKAGE_NAME} not a known target, using find_package().")
        find_package(${PACKAGE_NAME} REQUIRED)
      else()
        mola_message_verbose("find_mola_package: ${PACKAGE_NAME} is a known target, doing nothing.")
    endif()
    #TODO: use mola namespace (mola::xxx)?
endfunction()

# -----------------------------------------------------------------------------
# mola_set_target_cxx17(target)
#
# Enabled C++17 for the given target
# -----------------------------------------------------------------------------
function(mola_set_target_cxx17 TARGETNAME)
  target_compile_features(${TARGETNAME} PUBLIC cxx_std_17)
  if (MSVC)
    # this seems to be required in addition to the cxx_std_17 above (?)
    target_compile_options(${TARGETNAME} PUBLIC /std:c++latest)
  endif()
endfunction()

# -----------------------------------------------------------------------------
# mola_set_target_build_options(target)
#
# Set defaults for each MOLA cmake target
# -----------------------------------------------------------------------------
function(mola_set_target_build_options TARGETNAME)
  # Build for C++17
  mola_set_target_cxx17(${TARGETNAME})

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
    # gcc & clang:
    target_compile_options(${TARGETNAME} PRIVATE
      -Wall -Wextra -Wshadow
      -Werror=return-type # error on missing return();
      -Wtype-limits -Wcast-align -Wparentheses
      -fPIC
    )
  endif()

  # Optimization:
  # -------------------------
  if((NOT MSVC) AND (NOT CMAKE_CROSSCOMPILING))
    option(MOLA_BUILD_MARCH_NATIVE "Build with `-march=\"native\"`" OFF)

    if (MOLA_BUILD_MARCH_NATIVE)
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
# mola_configure_library(target [dep1 dep2...])
#
# Define a consistent install behavior for cmake-based library project:
# -----------------------------------------------------------------------------
function(mola_configure_library TARGETNAME)
  # Public hdrs interface:
  target_include_directories(${TARGETNAME} PUBLIC
      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
      $<INSTALL_INTERFACE:include>
      PRIVATE src
    )

  # Dynamic libraries output options:
  # -----------------------------------
  set_target_properties(${TARGETNAME} PROPERTIES
    OUTPUT_NAME "${TARGETNAME}${MOLA_DLL_VERSION_POSTFIX}"
    COMPILE_PDB_NAME "${TARGETNAME}${MOLA_DLL_VERSION_POSTFIX}"
    COMPILE_PDB_NAME_DEBUG "${TARGETNAME}${MOLA_DLL_VERSION_POSTFIX}${CMAKE_DEBUG_POSTFIX}"
    VERSION "${MOLA_VERSION_NUMBER_MAJOR}.${MOLA_VERSION_NUMBER_MINOR}.${MOLA_VERSION_NUMBER_PATCH}"
    SOVERSION ${MOLA_VERSION_NUMBER_MAJOR}.${MOLA_VERSION_NUMBER_MINOR}
    )

  # Project "folder":
  # -------------------
  set_target_properties(${TARGETNAME} PROPERTIES FOLDER "MOLA-modules")

  # Install lib:
  install(TARGETS ${TARGETNAME} EXPORT ${TARGETNAME}-targets
      ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
      LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
      RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
    )
  # Install hdrs:
  if (EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/include/)
    install(
      DIRECTORY include/
      DESTINATION ${CMAKE_INSTALL_PREFIX}/include
    )
  endif()

  # make project importable from build_dir:
  export(
    TARGETS ${TARGETNAME}
    # export to ROOT cmake directory (when building MOLA as a superproject)
    FILE ${CMAKE_BINARY_DIR}/${TARGETNAME}-targets.cmake
    NAMESPACE mola::
  )

  # Add alias to use the namespaced name within local builds from source:
  add_library(mola::${TARGETNAME} ALIAS ${TARGETNAME})

  # And generate the -config.cmake file:
  set(ALL_DEPS_LIST ${ARGN}) # used in xxx-config.cmake.in
  set(MOLA_MODULE_NAME ${TARGETNAME})
  configure_file(
    "${_MOLACOMMON_MODULE_BASE_DIR}/mola-xxx-config.cmake.in"
    "${CMAKE_BINARY_DIR}/${TARGETNAME}-config.cmake" IMMEDIATE @ONLY
  )
  # Version file:
  write_basic_package_version_file(
    "${CMAKE_BINARY_DIR}/${TARGETNAME}-config-version.cmake"
    VERSION ${MOLA_VERSION_NUMBER_MAJOR}.${MOLA_VERSION_NUMBER_MINOR}.${MOLA_VERSION_NUMBER_PATCH}
    COMPATIBILITY AnyNewerVersion
  )

	# Install cmake config module
	install(
		EXPORT
			${TARGETNAME}-targets
		DESTINATION
			${CMAKE_INSTALL_LIBDIR}/${TARGETNAME}/cmake
		NAMESPACE mola::
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
# mola_configure_app(target)
#
# Define common properties of cmake-based executable projects:
# -----------------------------------------------------------------------------
function(mola_configure_app TARGETNAME)
  # Project "folder":
  set_target_properties(${TARGETNAME} PROPERTIES FOLDER "MOLA-apps")

  #TODO: install

endfunction()



# -----------------------------------------------------------------------------
# mola_message_verbose(...)
# Maps to `message(STATUS ...)` if the environment variable VERBOSE is !=0.
# Otherwise, does nothing.
# -----------------------------------------------------------------------------
function(mola_message_verbose)
	if ($ENV{VERBOSE})
		message(STATUS ${ARGN})
	endif()
endfunction()


# -----------------------------------------------------------------------------
# mola_add_library(
#	TARGET name
#	SOURCES ${SRC_FILES}
#	[PUBLIC_LINK_LIBRARIES lib1 lib2]
#	[PRIVATE_LINK_LIBRARIES lib3 lib4]
# [CMAKE_DEPENDENCIES pkg1 pkg2]
#	)
#
# Defines a MOLA library. `CMAKE_DEPENDENCIES` enumerates those packages
# that needs to be find_package'd in this library's xxx-config.cmake file.
# -----------------------------------------------------------------------------
function(mola_add_library)
    set(options "")
    set(oneValueArgs TARGET)
    set(multiValueArgs SOURCES PUBLIC_LINK_LIBRARIES PRIVATE_LINK_LIBRARIES CMAKE_DEPENDENCIES)
    cmake_parse_arguments(MOLA_ADD_LIBRARY "${options}" "${oneValueArgs}" "${multiValueArgs}" ${ARGN})

    add_library(${MOLA_ADD_LIBRARY_TARGET}
      SHARED
      ${MOLA_ADD_LIBRARY_SOURCES}
    )

    # Define common flags:
    mola_set_target_build_options(${MOLA_ADD_LIBRARY_TARGET})
    mola_configure_library(${MOLA_ADD_LIBRARY_TARGET} ${MOLA_ADD_LIBRARY_CMAKE_DEPENDENCIES})

    # lib Dependencies:
    target_link_libraries(${MOLA_ADD_LIBRARY_TARGET}
      PUBLIC
      ${MOLA_ADD_LIBRARY_PUBLIC_LINK_LIBRARIES}
    )
    target_link_libraries(${MOLA_ADD_LIBRARY_TARGET}
      PRIVATE
      ${MOLA_ADD_LIBRARY_PRIVATE_LINK_LIBRARIES}
    )

   #TODO: install

endfunction()

# -----------------------------------------------------------------------------
# mola_add_executable(
#	TARGET name
#	SOURCES ${SRC_FILES}
#	[LINK_LIBRARIES lib1 lib2]
#	)
#
# Defines a MOLA executable
# -----------------------------------------------------------------------------
function(mola_add_executable)
    set(options DONT_INSTALL)
    set(oneValueArgs TARGET)
    set(multiValueArgs SOURCES LINK_LIBRARIES)
    cmake_parse_arguments(MOLA_ADD_EXECUTABLE "${options}" "${oneValueArgs}" "${multiValueArgs}" ${ARGN})

    add_executable(${MOLA_ADD_EXECUTABLE_TARGET}
      ${MOLA_ADD_EXECUTABLE_SOURCES}
    )

    # Define common flags:
    mola_set_target_build_options(${MOLA_ADD_EXECUTABLE_TARGET})
    mola_configure_app(${MOLA_ADD_EXECUTABLE_TARGET})

    # lib Dependencies:
    if (MOLA_ADD_EXECUTABLE_LINK_LIBRARIES)
      target_link_libraries(
      ${MOLA_ADD_EXECUTABLE_TARGET}
      ${MOLA_ADD_EXECUTABLE_LINK_LIBRARIES}
      )
    endif()

    # install:
    if (NOT MOLA_ADD_EXECUTABLE_DONT_INSTALL)
      install(TARGETS ${MOLA_ADD_EXECUTABLE_TARGET}
        ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
        LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
        RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
    )
    endif()
endfunction()


# -----------------------------------------------------------------------------
# mola_add_test(
#	TARGET name
#	SOURCES ${SRC_FILES}
#	[LINK_LIBRARIES lib1 lib2]
#	)
#
# Defines a MOLA unit test
# -----------------------------------------------------------------------------
function(mola_add_test)
    set(oneValueArgs TARGET)
    set(multiValueArgs SOURCES LINK_LIBRARIES)
    cmake_parse_arguments(MOLA_ADD_TEST "" "${oneValueArgs}" "${multiValueArgs}" ${ARGN})

    add_executable(${MOLA_ADD_TEST_TARGET}
      ${MOLA_ADD_TEST_SOURCES}
    )

    # Define common flags:
    mola_set_target_build_options(${MOLA_ADD_TEST_TARGET})
    mola_configure_app(${MOLA_ADD_TEST_TARGET})

    # lib Dependencies:
    if (MOLA_ADD_TEST_LINK_LIBRARIES)
      target_link_libraries(
      ${MOLA_ADD_TEST_TARGET}
      ${MOLA_ADD_TEST_LINK_LIBRARIES}
      )
    endif()

    # Macro for source dir path:
    target_compile_definitions(${MOLA_ADD_TEST_TARGET} PRIVATE
        MOLA_MODULE_SOURCE_DIR=\"${CMAKE_CURRENT_SOURCE_DIR}\"
        )

    # Run it:
    #add_custom_target(run_${MOLA_ADD_TEST_TARGET} COMMAND $<TARGET_FILE:${MOLA_ADD_TEST_TARGET}>)
    add_test(${MOLA_ADD_TEST_TARGET}_build "${CMAKE_COMMAND}" --build ${CMAKE_CURRENT_BINARY_DIR} --target ${MOLA_ADD_TEST_TARGET})
    add_test(run_${MOLA_ADD_TEST_TARGET} ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/${MOLA_ADD_TEST_TARGET})
    set_tests_properties(run_${MOLA_ADD_TEST_TARGET} PROPERTIES DEPENDS ${MOLA_ADD_TEST_TARGET}_build)

    add_custom_target(run_${MOLA_ADD_TEST_TARGET} COMMAND ${MOLA_ADD_TEST_TARGET})
    add_dependencies(run_${MOLA_ADD_TEST_TARGET} ${MOLA_ADD_TEST_TARGET})

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
# mola_find_package_or_return(package_name)
#
# Calls find_package(package_name QUIET), and if it is not found, prints a
# descriptive message and call "return()" to exit the current cmake script.
# -----------------------------------------------------------------------------
macro(mola_find_package_or_return PACKAGE_NAME)
	find_package(${PACKAGE_NAME} QUIET)
	if (NOT ${PACKAGE_NAME}_FOUND)
		message(WARNING "${PROJECT_NAME}: Skipping due to missing dependency `${PACKAGE_NAME}`")
		return()
	endif()
endmacro()
