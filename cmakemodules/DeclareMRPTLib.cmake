include(GNUInstallDirs)
include(CMakePackageConfigHelpers)

# JLBC Dec 2018: On special install paths for Debian pkg prefixes
# After many hours debugging how to correctly export multiple
# CMake targets in one source package, each to a different debian pkg, I found
# this to be the only working solution:
# * Do NOT add module prefixes to installation paths. For example, this used
#   to be:
#  if(CMAKE_MRPT_USE_DEB_POSTFIXS)
#  	set(this_lib_dev_INSTALL_PREFIX "libmrpt-${name}-dev/usr/")
#   set(this_lib_bin_INSTALL_PREFIX "libmrpt-${name}${CMAKE_MRPT_VERSION_NUMBER_MAJOR}.${CMAKE_MRPT_VERSION_NUMBER_MINOR}/usr/")
#  ...
#  and it was great. But... then CMake xxx-target.cmake files would include
#  "libxxx-dev/" prefix and the user's cmake find_package() would be folish
#  would fail to find the packages.
# * Add, to debian/rules, bash commands to distribute each .so* to its proper
#   place. Even if it seems easier to directly install them to their place
#   from this script, the note above explains why that's incompatible with
#   cmake-exported targets.


# Enforce C++17 in all dependent projects:
function(mrpt_lib_target_requires_cpp17 _TARGET)
	if(${CMAKE_VERSION} VERSION_LESS "3.8.0")
		# Support older cmake versions:
		if (MSVC)
			target_compile_options(${_TARGET} INTERFACE "/std:c++latest")
		else()
			target_compile_options(${_TARGET} INTERFACE "-std=c++17")
		endif()
	else()
		if (NOT MSVC)
			# Modern, clean way to do this:
			target_compile_features(${_TARGET} INTERFACE cxx_std_17)
		else()
			# At present (CMake 3.12 + MSVC 19.15.26732.1) it seems cxx_std_17
			# does not enable C++17 in MSVC (!).
			target_compile_options(${_TARGET} INTERFACE "/std:c++17")
		endif()
	endif()
endfunction()

# define_mrpt_lib(): Declares an MRPT library target:
#-----------------------------------------------------------------------
macro(define_mrpt_lib name)
	internal_define_mrpt_lib(${name} 0 0 ${ARGN}) # headers_only = 0, is_metalib=0
endmacro(define_mrpt_lib)

# define_mrpt_lib_header_only(): Declares an MRPT headers-only library:
#-----------------------------------------------------------------------
macro(define_mrpt_lib_header_only name)
	internal_define_mrpt_lib(${name} 1 0 ${ARGN}) # headers_only = 1, is_metalib=0
endmacro(define_mrpt_lib_header_only)

# define_mrpt_metalib(): Declares an MRPT meta-lib:
#-----------------------------------------------------------------------
macro(define_mrpt_metalib name)
	internal_define_mrpt_lib(${name} 1 1 ${ARGN}) # headers_only = 1, is_metalib=1
endmacro(define_mrpt_metalib)


# Implementation of both define_mrpt_lib() and define_mrpt_lib_headers_only():
#-----------------------------------------------------------------------------
macro(internal_define_mrpt_lib name headers_only is_metalib)
	include(../../cmakemodules/AssureCMakeRootFile.cmake) # Avoid user mistake in CMake source directory

	# Allow programmers of mrpt libs to change the default value of build_mrpt-LIB, which is "ON" by default.
	set(_DEFVAL "${DEFAULT_BUILD_mrpt-${name}}")
	if ("${_DEFVAL}" STREQUAL "")
		set(_DEFVAL "ON")
	endif ()

	set(BUILD_mrpt-${name} ${_DEFVAL} CACHE BOOL "Build the library mrpt-${name}")
	if(BUILD_mrpt-${name})
	# --- Start of conditional build of module ---

	if(NOT ${is_metalib})
		project(mrpt-${name} LANGUAGES C CXX)
	endif()

	# Optional build-time plugin mechanism:
	set(PLUGIN_FILE_mrpt-${name} "" CACHE FILEPATH "Optional CMake file defining additional sources for mrpt-${name}")
	mark_as_advanced(PLUGIN_FILE_mrpt-${name})
	if (EXISTS "${PLUGIN_FILE_mrpt-${name}}")
		include("${PLUGIN_FILE_mrpt-${name}}")
		list(APPEND ${name}_EXTRA_SRCS	      ${${name}_PLUGIN_SRCS})
		list(APPEND ${name}_EXTRA_SRCS_NAME   ${${name}_PLUGIN_SRCS_NAME})
	endif()

	# There is an optional LISTS of extra sources from the caller:
	#  "${name}_EXTRA_SRCS" and
	#  "${name}_EXTRA_SRCS_NAME"   <--- Must NOT contain spaces!!
	#
	#  At return from this macro, there'll be defined a variable:
	#	   "${${name}_EXTRA_SRCS_NAME}_FILES"
	#   with the list of all files under that group.
	#
	#  For code simplicity, let's use the same list, just adding the default sources there:
	list(APPEND ${name}_EXTRA_SRCS
		"${CMAKE_SOURCE_DIR}/libs/${name}/src/*.cpp"
		"${CMAKE_SOURCE_DIR}/libs/${name}/src/*.c"
		"${CMAKE_SOURCE_DIR}/libs/${name}/src/*.cxx"
#		"${CMAKE_SOURCE_DIR}/libs/${name}/src/*.h"
#		"${CMAKE_SOURCE_DIR}/libs/${name}/include/mrpt/${name}/*.h"
#		"${CMAKE_SOURCE_DIR}/libs/${name}/include/mrpt/${name}/*.hpp"
#		"${CMAKE_SOURCE_DIR}/doc/doxygen-pages/lib_mrpt_${name}.h"
		)
	list(APPEND ${name}_EXTRA_SRCS_NAME
		"${name}"
		"${name}"
		"${name}"
#		"${name} Internal Headers"
#		"${name} Public Headers"
#		"${name} Public Headers"
#		"Documentation"
		)
	# Only add these ones for "normal" libraries:
	if (NOT ${headers_only})
		list(APPEND ${name}_EXTRA_SRCS
			"${CMAKE_SOURCE_DIR}/libs/${name}/src/registerAllClasses.cpp"
			)
		list(APPEND ${name}_EXTRA_SRCS_NAME
			"Class register"
			)
	endif ()

	# Collect files
	# ---------------------------------------------------------
	list(LENGTH ${name}_EXTRA_SRCS N_SRCS)
	list(LENGTH ${name}_EXTRA_SRCS_NAME N_SRCS_NAMES)

	if (NOT N_SRCS EQUAL N_SRCS_NAMES)
		message(FATAL_ERROR "Mismatch length in ${name}_EXTRA_SRCS and ${name}_EXTRA_SRCS_NAME!")
	endif ()

	set(${name}_srcs "")  # ALL the files

	math(EXPR N_SRCS "${N_SRCS}-1")  # Indices are 0-based

	foreach(i RANGE 0 ${N_SRCS})
		# Get i'th expression & its name:
		list(GET ${name}_EXTRA_SRCS      ${i} FILS_EXPR)
		list(GET ${name}_EXTRA_SRCS_NAME ${i} FILS_GROUP_NAME)

		file(GLOB aux_list ${FILS_EXPR})

		source_group("${FILS_GROUP_NAME} files" FILES ${aux_list})

		# Add to main list:
		list(APPEND ${name}_srcs ${aux_list})
		# All to group lists, may be used by the user upon return from this macro:
		list(APPEND ${FILS_GROUP_NAME}_FILES ${aux_list})
	endforeach()

	# Remove _LIN files when compiling under Windows, and _WIN files when compiling under Linux.
	if(WIN32)
		REMOVE_MATCHING_FILES_FROM_LIST(".*_LIN.cpp" ${name}_srcs)		# Win32
	else()
		REMOVE_MATCHING_FILES_FROM_LIST(".*_WIN.cpp" ${name}_srcs)		# Apple & Unix
	endif()

	# Keep a list of unit testing files, for declaring them in /test:
	set(lst_unittests ${${name}_srcs})
	KEEP_MATCHING_FILES_FROM_LIST(".*_unittest.cpp" lst_unittests)
	if(NOT "${lst_unittests}" STREQUAL "")
		# We have unit tests:
		get_property(_lst_lib_test GLOBAL PROPERTY "MRPT_TEST_LIBS")
		set_property(GLOBAL PROPERTY "MRPT_TEST_LIBS" ${_lst_lib_test} mrpt_${name})
		set_property(GLOBAL PROPERTY "mrpt_${name}_UNIT_TEST_FILES" ${lst_unittests})
	endif(NOT "${lst_unittests}" STREQUAL "")


	# Don't include here the unit testing code:
	REMOVE_MATCHING_FILES_FROM_LIST(".*_unittest.cpp" ${name}_srcs)

	#  Define the target:
	set(all_${name}_srcs  ${${name}_srcs})

	# Add main lib header (may not exist in meta-libs only):
	if (EXISTS "${CMAKE_SOURCE_DIR}/libs/${name}/include/mrpt/${name}.h")
		set(all_${name}_srcs ${all_${name}_srcs} "${CMAKE_SOURCE_DIR}/libs/${name}/include/mrpt/${name}.h")
	endif ()

	if (NOT ${headers_only})
		# A libray target:
		add_library(mrpt-${name}
			${all_${name}_srcs}      # sources
			${MRPT_VERSION_RC_FILE}  # Only !="" in Win32: the .rc file with version info
			)

		# private include dirs for this lib:
		target_include_directories(mrpt-${name} PRIVATE
				$<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/libs/${name}/src/> # To include ${name}-precomp.h
			)
		if(MSVC)  # Define math constants if built with MSVC
			target_compile_definitions(mrpt-${name} PUBLIC _USE_MATH_DEFINES)
		endif()

		set_target_properties(mrpt-${name} PROPERTIES FOLDER "modules")
		set(iftype PUBLIC)
		add_coverage(mrpt-${name})
		if(CLANG_TIDY_EXE)
			set_target_properties(
				mrpt-${name} PROPERTIES
				CXX_CLANG_TIDY "${DO_CLANG_TIDY}"
			)
		endif()
	else()
		# A hdr-only library: needs no real compiling
		add_library(mrpt-${name} INTERFACE)
		# List of hdr files (for editing in IDEs,etc.):
		target_sources(mrpt-${name} INTERFACE ${all_${name}_srcs})
		set(iftype INTERFACE)
	endif ()

	# Include directories for target:
	if (CMAKE_MRPT_USE_DEB_POSTFIXS)
		set(LIB_INCL_DIR_EXPRESSION "/usr/include/mrpt/${name}/include")
	else()
		set(LIB_INCL_DIR_EXPRESSION "$<INSTALL_PREFIX>/include/mrpt/${name}/include")
	endif()

	target_include_directories(mrpt-${name} ${iftype}
		$<BUILD_INTERFACE:${MRPT_SOURCE_DIR}/libs/${name}/include>
		$<INSTALL_INTERFACE:${LIB_INCL_DIR_EXPRESSION}>
	)

	add_dependencies(all_mrpt_libs mrpt-${name}) # for target: all_mrpt_libs

	# Append to list of all mrpt-* libraries:
	if("${ALL_MRPT_LIBS}" STREQUAL "")  # first one is different to avoid an empty first list element ";mrpt-xxx"
		set(ALL_MRPT_LIBS "mrpt-${name}" CACHE INTERNAL "")  # This emulates global vars
	else()
		set(ALL_MRPT_LIBS "${ALL_MRPT_LIBS};mrpt-${name}" CACHE INTERNAL "")  # This emulates global vars
	endif()

	# Dependencies:
	set(MRPT_ONLY_DEPS_LIST "")
	set(ALL_DEPS_LIST "")
	set(AUX_ALL_DEPS_BUILD 1)  # Will be set to "0" if any dependency if not built
	foreach(DEP ${ARGN})
		list(APPEND ALL_DEPS_LIST ${DEP}) # used in mrpt-*-config.cmake.in

		# Only for "mrpt-XXX" libs:
		if (${DEP} MATCHES "mrpt-")
			# Add it as a dependency
			target_link_libraries(mrpt-${name} PUBLIC ${DEP})
			#add_dependencies() implicit with above link dep

			# Append to list of mrpt-* lib dependences:
			list(APPEND MRPT_ONLY_DEPS_LIST ${DEP})
			# Now, check mrpt-* deps only:
			string(REGEX REPLACE "mrpt-(.*)" "\\1" DEP_MRPT_NAME ${DEP})
			if(NOT "${DEP_MRPT_NAME}" STREQUAL "")
				# Check if all dependencies are to be build:
				if ("${BUILD_mrpt-${DEP_MRPT_NAME}}" STREQUAL "OFF")
					set(AUX_ALL_DEPS_BUILD 0)
					message(STATUS "*Warning*: Lib mrpt-${name} cannot be built because dependency mrpt-${DEP_MRPT_NAME} has been disabled!")
				endif ()
			endif()
		endif ()
	endforeach()

	# Impossible to build?
	if (NOT AUX_ALL_DEPS_BUILD)
		message(STATUS "*Warning* ==> Disabling compilation of lib mrpt-${name} for missing dependencies listed above.")
		set(BUILD_mrpt-${name} OFF CACHE BOOL "Build the library mrpt-${name}" FORCE)
	endif ()

	# Emulates a global variable:
	set_property(GLOBAL PROPERTY "mrpt-${name}_LIB_DEPS" "${MRPT_ONLY_DEPS_LIST}")
	set_property(GLOBAL PROPERTY "mrpt-${name}_LIB_IS_HEADERS_ONLY" "${headers_only}")
	set_property(GLOBAL PROPERTY "mrpt-${name}_LIB_IS_METALIB" "${is_metalib}")

	add_dependencies(mrpt-${name} "DocumentationFiles")  # docs files target (useful for IDE editing)

	if (NOT ${headers_only})
		target_link_libraries(mrpt-${name}
			PRIVATE
			${MRPTLIB_LINKER_LIBS}
			)
	endif ()

	# Set custom name of lib + dynamic link numbering convenions in Linux:
	if (NOT ${headers_only})
		set_target_properties(mrpt-${name} PROPERTIES
			OUTPUT_NAME ${MRPT_LIB_PREFIX}mrpt-${name}${MRPT_DLL_VERSION_POSTFIX}
			COMPILE_PDB_NAME "${MRPT_LIB_PREFIX}mrpt-${name}${MRPT_DLL_VERSION_POSTFIX}"
			COMPILE_PDB_NAME_DEBUG "${MRPT_LIB_PREFIX}mrpt-${name}${MRPT_DLL_VERSION_POSTFIX}${CMAKE_DEBUG_POSTFIX}"
			ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/lib/"
			RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin/"
			VERSION "${CMAKE_MRPT_VERSION_NUMBER_MAJOR}.${CMAKE_MRPT_VERSION_NUMBER_MINOR}.${CMAKE_MRPT_VERSION_NUMBER_PATCH}"
			SOVERSION ${CMAKE_MRPT_VERSION_NUMBER_MAJOR}.${CMAKE_MRPT_VERSION_NUMBER_MINOR}
			)

		# Set all header files as "ignored" (don't build!):
		# -----------------------------------------------------
		set(AUX_LIST_TO_IGNORE ${all_${name}_srcs})
		KEEP_MATCHING_FILES_FROM_LIST("^.*h$" AUX_LIST_TO_IGNORE)
		set_source_files_properties(${AUX_LIST_TO_IGNORE} PROPERTIES HEADER_FILE_ONLY true)

		if(MRPT_ENABLE_PRECOMPILED_HDRS)
			if (MSVC)
				# Precompiled hdrs for MSVC:
				# --------------------------------------
				string(TOUPPER ${name} NAMEUP)

				# The "use precomp.headr" for all the files...
				set_target_properties(mrpt-${name}
					PROPERTIES
					COMPILE_FLAGS "/Yu${name}-precomp.h")

				# But for the file used to build the precomp. header:
				set_source_files_properties("${CMAKE_SOURCE_DIR}/libs/${name}/src/${name}-precomp.cpp"
					PROPERTIES
					COMPILE_FLAGS "/Yc${name}-precomp.h")
			else()
				# Use cotire module for GCC/CLANG:
				list(APPEND COTIRE_PREFIX_HEADER_IGNORE_PATH
					"${OpenCV_INCLUDE_DIR}"
					"${MRPT_LIBS_ROOT}/${name}/src"
					"/usr/"  # avoid problems with Cotire trying to include internal GCC headers, not suitable for direct use.
				)
				set_target_properties(mrpt-${name} PROPERTIES
					COTIRE_PREFIX_HEADER_IGNORE_PATH "${COTIRE_PREFIX_HEADER_IGNORE_PATH}"
				)
				set_target_properties(mrpt-${name} PROPERTIES	COTIRE_CXX_PREFIX_HEADER_INIT "${CMAKE_SOURCE_DIR}/libs/${name}/src/${name}-precomp.h")
				cotire(mrpt-${name})

				if($ENV{VERBOSE})
					#get_target_property(_unitySource example COTIRE_CXX_UNITY_SOURCE)
					#get_target_property(_unityTargetName mrpt-${name} COTIRE_UNITY_TARGET_NAME)
					get_target_property(_prefixHeader mrpt-${name} COTIRE_CXX_PREFIX_HEADER)
					get_target_property(_precompiledHeader mrpt-${name} COTIRE_CXX_PRECOMPILED_HEADER)
					message(STATUS "  mrpt-${name}: Prefix header=${_prefixHeader}")
					message(STATUS "  mrpt-${name}: PCH header=${_precompiledHeader}")
				endif()
			endif()

			source_group("Precompiled headers" FILES
				"${CMAKE_SOURCE_DIR}/libs/${name}/src/${name}-precomp.cpp"
				"${CMAKE_SOURCE_DIR}/libs/${name}/src/${name}-precomp.h"
				)
		endif()

		# (See comments in script_matlab.cmake)
		# Add /DELAYLOAD:... to avoid dependency of these DLLs for standalone (non-mex) projects
		if (CMAKE_MRPT_HAS_MATLAB AND BUILD_SHARED_LIBS AND MSVC)
			set_property(
				TARGET mrpt-${name}
				APPEND_STRING PROPERTY
				LINK_FLAGS " /DELAYLOAD:\"libmx.dll\" /DELAYLOAD:\"libmex.dll\" /ignore:4199")
			# The /ignore:4199 is to disable warnings like these:
			#  warning LNK4199: /DELAYLOAD:libmx.dll ignored; no imports found from libmx.dll
			# in libs which do not (yet) support mex stuff
		endif ()

		if (USE_IWYU)
			set_property(
				TARGET mrpt-${name}
			PROPERTY CXX_INCLUDE_WHAT_YOU_USE ${IWYU_PATH_AND_OPTIONS})
		endif()

		# make sure the library gets installed
		if (NOT is_metalib)
			install(TARGETS mrpt-${name} EXPORT mrpt-${name}-targets
				RUNTIME DESTINATION bin  COMPONENT Libraries
				LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR} COMPONENT Libraries
				ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR} COMPONENT Libraries  # WAS: lib${LIB_SUFFIX}
				)

			# Collect .pdb debug files for optional installation:
			if (MSVC)
				set(PDB_FILE
					"${CMAKE_BINARY_DIR}/bin/Debug/${MRPT_LIB_PREFIX}mrpt-${name}${MRPT_DLL_VERSION_POSTFIX}${CMAKE_DEBUG_POSTFIX}.pdb")
				if (EXISTS "${PDB_FILE}")
					install(FILES ${PDB_FILE} DESTINATION bin COMPONENT LibrariesDebugInfoPDB)
				endif ()
			endif(MSVC)
		endif (NOT is_metalib)
	else() # it IS headers_only:
		install(TARGETS mrpt-${name} EXPORT mrpt-${name}-targets)
	endif (NOT ${headers_only})

	# Create module CMake config file:
	# For local usage from the BUILD directory (without "install"):
	# 1/3: autogenerated target file:
	export(
		TARGETS mrpt-${name}
		FILE "${CMAKE_BINARY_DIR}/mrpt-${name}-targets.cmake"
	)
	# 2/3: config file with manual list of dependencies:
	set(PKG_NAME mrpt-${name})
	configure_file(
		"${MRPT_SOURCE_DIR}/parse-files/mrpt-xxx-config.cmake.in"
		"${CMAKE_BINARY_DIR}/mrpt-${name}-config.cmake" IMMEDIATE @ONLY)
	# 3/3: version file:
	write_basic_package_version_file(
		"${CMAKE_BINARY_DIR}/mrpt-${name}-config-version.cmake"
		VERSION ${CMAKE_MRPT_FULL_VERSION}
		COMPATIBILITY SameMajorVersion
	)

	if(CMAKE_MRPT_USE_DEB_POSTFIXS)
		set(this_lib_dev_INSTALL_PREFIX "libmrpt-${name}-dev/usr/")
	endif()

	# mrpt-xxx-config.cmake file:
	# Makes the project importable from installed dir:
	# 1/3: autogenerated target file:
	install(
		EXPORT mrpt-${name}-targets
		DESTINATION share/mrpt-${name}
	)
	# 2/3: config file with manual list of dependencies:
	# 3/3: version file:
	install(
		FILES
			"${CMAKE_BINARY_DIR}/mrpt-${name}-config.cmake"
			"${CMAKE_BINARY_DIR}/mrpt-${name}-config-version.cmake"
		DESTINATION share/mrpt-${name}
	)

	# Install public headers:
	set(HEADERS_DIR "${MRPT_SOURCE_DIR}/libs/${name}/include/")
	if (EXISTS "${HEADERS_DIR}")  # This is mainly to avoid problems with "virtual module" names
		install(
		DIRECTORY
			"${HEADERS_DIR}"
		DESTINATION
			${this_lib_dev_INSTALL_PREFIX}include/mrpt/${name}/include/
		)
	endif()

	if(MRPT_ENABLE_PRECOMPILED_HDRS AND MSVC)
		foreach(_N ${${name}_PLUGIN_SRCS_NAME})
			#message(STATUS "Disabling precomp hdrs for N=${_N}: ${${_N}_FILES}")
			set_source_files_properties(${${_N}_FILES} PROPERTIES COMPILE_FLAGS "/Y-")
		endforeach()
	endif()

	# --- End of conditional build of module ---
	endif(BUILD_mrpt-${name})

endmacro(internal_define_mrpt_lib)
