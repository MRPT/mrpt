# define_mrpt_lib(): Declares an MRPT library target:
#-----------------------------------------------------------------------
macro(define_mrpt_lib name)
	internal_define_mrpt_lib(${name} 0 ${ARGN}) # headers_only = 0
endmacro(define_mrpt_lib)

# define_mrpt_lib_header_only(): Declares an MRPT headers-only library:
#-----------------------------------------------------------------------
macro(define_mrpt_lib_header_only name)
	internal_define_mrpt_lib(${name} 1 ${ARGN}) # headers_only = 1
endmacro(define_mrpt_lib_header_only)


# Implementation of both define_mrpt_lib() and define_mrpt_lib_headers_only():
#-----------------------------------------------------------------------------
macro(internal_define_mrpt_lib name headers_only)
	INCLUDE(../../cmakemodules/AssureCMakeRootFile.cmake) # Avoid user mistake in CMake source directory

	SET(BUILD_mrpt-${name} ON CACHE BOOL "Include module mrpt-${name} in this MRPT build.")
	IF(BUILD_mrpt-${name}) 
	# --- Start of conditional build of module ---

	PROJECT(mrpt-${name})
	
	# There is an optional LISTS of extra sources from the caller: 
	#  "${name}_EXTRA_SRCS" and 
	#  "${name}_EXTRA_SRCS_NAME"   <--- Must NOT contain spaces!!
	#
	#  At return from this macro, there'll be defined a variable:
	#	   "${${name}_EXTRA_SRCS_NAME}_FILES"
	#   with the list of all files under that group.
	#
	#  For code simplicity, let's use the same list, just adding the default sources there:
	LIST(APPEND ${name}_EXTRA_SRCS 
		"${CMAKE_SOURCE_DIR}/libs/${name}/src/*.cpp"
		"${CMAKE_SOURCE_DIR}/libs/${name}/src/*.c"
		"${CMAKE_SOURCE_DIR}/libs/${name}/src/*.cxx"
		"${CMAKE_SOURCE_DIR}/libs/${name}/src/*.h"
		"${CMAKE_SOURCE_DIR}/libs/${name}/include/mrpt/${name}/*.h"
		"${CMAKE_SOURCE_DIR}/libs/${name}/include/mrpt/${name}/*.hpp"
		)
	LIST(APPEND ${name}_EXTRA_SRCS_NAME
		"${name}"
		"${name}"
		"${name}"
		"${name}"
		"${name}"
		"${name}"
		)
	# Only add these ones for "normal" libraries:
	IF (NOT ${headers_only})
		LIST(APPEND ${name}_EXTRA_SRCS 
			"${CMAKE_SOURCE_DIR}/libs/${name}/include/mrpt/${name}/link_pragmas.h"
			"${CMAKE_SOURCE_DIR}/libs/${name}/src/registerAllClasses.cpp"
			)
		LIST(APPEND ${name}_EXTRA_SRCS_NAME
			"DLL link macros"
			"Class register"
			)
	ENDIF (NOT ${headers_only})

	# Collect files
	# ---------------------------------------------------------
	LIST(LENGTH ${name}_EXTRA_SRCS N_SRCS)
	LIST(LENGTH ${name}_EXTRA_SRCS_NAME N_SRCS_NAMES)
	
	IF (NOT N_SRCS EQUAL N_SRCS_NAMES)
		MESSAGE(FATAL_ERROR "Mismatch length in ${name}_EXTRA_SRCS and ${name}_EXTRA_SRCS_NAME!")
	ENDIF (NOT N_SRCS EQUAL N_SRCS_NAMES)
	
	SET(${name}_srcs "")  # ALL the files
	
	MATH(EXPR N_SRCS "${N_SRCS}-1")  # Indices are 0-based
	
	foreach(i RANGE 0 ${N_SRCS})
		# Get i'th expression & its name:
		LIST(GET ${name}_EXTRA_SRCS      ${i} FILS_EXPR)
		LIST(GET ${name}_EXTRA_SRCS_NAME ${i} FILS_GROUP_NAME)
		
		FILE(GLOB aux_list ${FILS_EXPR})
		
		SOURCE_GROUP("${FILS_GROUP_NAME} files" FILES ${aux_list})
		
		# Add to main list:
		LIST(APPEND ${name}_srcs ${aux_list})
		# All to group lists, may be used by the user upon return from this macro:
		LIST(APPEND ${FILS_GROUP_NAME}_FILES ${aux_list})
	endforeach(i)

	# Remove _LIN files when compiling under Windows, and _WIN files when compiling under Linux.
	IF(WIN32)		
		REMOVE_MATCHING_FILES_FROM_LIST(".*_LIN.cpp" ${name}_srcs)		# Win32
	ELSE(WIN32)
		REMOVE_MATCHING_FILES_FROM_LIST(".*_WIN.cpp" ${name}_srcs)		# Apple & Unix
	ENDIF(WIN32)

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
	set(all_${name}_srcs 
		${${name}_srcs}
		"${CMAKE_SOURCE_DIR}/libs/${name}/include/mrpt/${name}.h")
	
	IF (NOT ${headers_only})

		# A libray target:
		ADD_LIBRARY(mrpt-${name}   ${all_${name}_srcs})

	ELSE(NOT ${headers_only})

		# A custom target (needs no real compiling)
		add_custom_target(mrpt-${name} DEPENDS ${all_${name}_srcs})

	ENDIF (NOT ${headers_only})

	# Append to list of all mrpt-* libraries:
	if("${ALL_MRPT_LIBS}" STREQUAL "")  # first one is different to avoid an empty first list element ";mrpt-xxx"
		SET(ALL_MRPT_LIBS "mrpt-${name}" CACHE INTERNAL "")  # This emulates global vars
	else("${ALL_MRPT_LIBS}" STREQUAL "")
		SET(ALL_MRPT_LIBS "${ALL_MRPT_LIBS};mrpt-${name}" CACHE INTERNAL "")  # This emulates global vars
	endif("${ALL_MRPT_LIBS}" STREQUAL "")
	
	# Include dir for this lib:
	INCLUDE_DIRECTORIES("${MRPT_SOURCE_DIR}/libs/${name}/include")
	
	# Include dirs for mrpt-XXX libs:
	set(AUX_DEPS_LIST "")
	set(AUX_EXTRA_LINK_LIBS "")
	FOREACH(DEP ${ARGN})
		# Only for "mrpt-XXX" libs:
		IF (${DEP} MATCHES "mrpt-")
			STRING(REGEX REPLACE "mrpt-(.*)" "\\1" DEP_MRPT_NAME ${DEP})
			IF(NOT "${DEP_MRPT_NAME}" STREQUAL "")
				# Include dir:
				INCLUDE_DIRECTORIES("${MRPT_SOURCE_DIR}/libs/${DEP_MRPT_NAME}/include")
				
				# Link "-lmrpt-name", only for GCC and if both THIS and the dependence are non-header-only:
				IF(CMAKE_COMPILER_IS_GNUCXX AND NOT ${headers_only})
					get_property(_LIB_HDRONLY GLOBAL PROPERTY "${DEP}_LIB_IS_HEADERS_ONLY")
					IF(NOT _LIB_HDRONLY)
						#MESSAGE(STATUS "adding link dep: mrpt-${name} -> ${DEP}")
						LIST(APPEND AUX_EXTRA_LINK_LIBS ${DEP}${MRPT_LINKER_LIBS_POSTFIX})
					ENDIF(NOT _LIB_HDRONLY)
				ENDIF(CMAKE_COMPILER_IS_GNUCXX AND NOT ${headers_only})
				
				# Append to list of mrpt-* lib dependences:
				LIST(APPEND AUX_DEPS_LIST ${DEP})
			ENDIF(NOT "${DEP_MRPT_NAME}" STREQUAL "")
		ENDIF (${DEP} MATCHES "mrpt-")		
	ENDFOREACH(DEP)
	
	# Emulates a global variable:
	set_property(GLOBAL PROPERTY "mrpt-${name}_LIB_DEPS" "${AUX_DEPS_LIST}")
	set_property(GLOBAL PROPERTY "mrpt-${name}_LIB_IS_HEADERS_ONLY" "${headers_only}")

	# Dependencies between projects:
	IF(NOT "${ARGN}" STREQUAL "")
		ADD_DEPENDENCIES(mrpt-${name} ${ARGN})
	ENDIF(NOT "${ARGN}" STREQUAL "")

	IF (NOT ${headers_only})
		TARGET_LINK_LIBRARIES(mrpt-${name} 
			${MRPTLIB_LINKER_LIBS}
			${AUX_EXTRA_LINK_LIBS}
			)
	ENDIF (NOT ${headers_only})

	SET_TARGET_PROPERTIES(mrpt-${name} PROPERTIES PROJECT_LABEL "(LIB) mrpt-${name}")

	# Set custom name of lib + dynamic link numbering convenions in Linux:
	IF (NOT ${headers_only})
		SET_TARGET_PROPERTIES(mrpt-${name} PROPERTIES 
			OUTPUT_NAME ${MRPT_LIB_PREFIX}mrpt-${name}${MRPT_DLL_VERSION_POSTFIX}
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
	
		IF(MRPT_ENABLE_PRECOMPILED_HDRS)
			IF (MSVC)
				# Precompiled hdrs for MSVC:
				# --------------------------------------
				STRING(TOUPPER ${name} NAMEUP)

				# The "use precomp.headr" for all the files...
				set_target_properties(mrpt-${name}
					PROPERTIES
					COMPILE_FLAGS "/Yumrpt/${name}.h")

				# But for the file used to build the precomp. header:
				set_source_files_properties("${CMAKE_SOURCE_DIR}/libs/${name}/src/precomp_hdr.cpp"
					PROPERTIES
					COMPILE_FLAGS "/Ycmrpt/${name}.h")
			ENDIF (MSVC)
		
			SOURCE_GROUP("Precomp. header files" FILES 
				"${CMAKE_SOURCE_DIR}/libs/${name}/src/precomp_hdr.cpp"
				"${CMAKE_SOURCE_DIR}/libs/${name}/include/mrpt/${name}.h"
				)	
		ENDIF(MRPT_ENABLE_PRECOMPILED_HDRS)

		# Special directories when building a .deb package:
		IF(CMAKE_MRPT_USE_DEB_POSTFIXS)
			SET(MRPT_PREFIX_INSTALL "${CMAKE_INSTALL_PREFIX}/libmrpt-${name}${CMAKE_MRPT_VERSION_NUMBER_MAJOR}.${CMAKE_MRPT_VERSION_NUMBER_MINOR}/usr/")
		ELSE(CMAKE_MRPT_USE_DEB_POSTFIXS)
			SET(MRPT_PREFIX_INSTALL "")
		ENDIF(CMAKE_MRPT_USE_DEB_POSTFIXS)

		# make sure the library gets installed
		INSTALL(TARGETS mrpt-${name}
			RUNTIME DESTINATION ${MRPT_PREFIX_INSTALL}bin  COMPONENT main
			LIBRARY DESTINATION ${MRPT_PREFIX_INSTALL}lib${LIB_SUFFIX} COMPONENT main
			ARCHIVE DESTINATION ${MRPT_PREFIX_INSTALL}lib${LIB_SUFFIX} COMPONENT main
			)
	ENDIF (NOT ${headers_only})

	# Generate the libmrpt-$NAME.pc file for pkg-config:
	IF(UNIX)
		SET(mrpt_pkgconfig_LIBNAME ${name})
		get_property(_lst_deps GLOBAL PROPERTY "mrpt-${name}_LIB_DEPS")
		
		# a comma-separated list of other mrpt-* dependencies.
		SET(mrpt_pkgconfig_REQUIRES "")
		FOREACH(DEP ${_lst_deps})
			IF(NOT "${mrpt_pkgconfig_REQUIRES}" STREQUAL "")
				SET(mrpt_pkgconfig_REQUIRES "${mrpt_pkgconfig_REQUIRES},")
			ENDIF(NOT "${mrpt_pkgconfig_REQUIRES}" STREQUAL "")
			SET(mrpt_pkgconfig_REQUIRES "${mrpt_pkgconfig_REQUIRES}${DEP}")
		ENDFOREACH(DEP)

		# Special case: For mrpt-base, mark "eigen3" as a pkg-config dependency only 
		#  if we are instructed to do so: (EIGEN_USE_EMBEDDED_VERSION=OFF)
		IF(NOT EIGEN_USE_EMBEDDED_VERSION)
			SET(mrpt_pkgconfig_REQUIRES "${mrpt_pkgconfig_REQUIRES},eigen3")
		ENDIF(NOT EIGEN_USE_EMBEDDED_VERSION)

		# "Libs" lines in .pc files:
		# -----------------------------------------------------------
		# * for install, normal lib:
		#    Libs: -L${libdir}  -lmrpt-@mrpt_pkgconfig_LIBNAME@ 
		# * for install, headers-only lib:
		#    <none>
		# * for local usage, normal lib:
		#    Libs: -L${libdir} -Wl,-rpath,${libdir} -lmrpt-@mrpt_pkgconfig_LIBNAME@ 
		# * for local usage, headers-only lib:
		#    <none>
		IF (${headers_only})
			SET(mrpt_pkgconfig_lib_line_install "")
			SET(mrpt_pkgconfig_lib_line_noinstall "")
			SET(mrpt_pkgconfig_libs_private_line "")
		ELSE (${headers_only})
			SET(mrpt_pkgconfig_lib_line_install "Libs: -L\${libdir}  -lmrpt-${name}")
			SET(mrpt_pkgconfig_lib_line_noinstall "Libs: -L\${libdir} -Wl,-rpath,\${libdir} -lmrpt-${name}")
			SET(mrpt_pkgconfig_libs_private_line "Libs.private: ${MRPTLIB_LINKER_LIBS}")
		ENDIF (${headers_only})

		# (1/2) Generate the .pc file for "make install"
		CONFIGURE_FILE("${CMAKE_SOURCE_DIR}/parse-files/mrpt_template.pc.in" "${CMAKE_BINARY_DIR}/pkgconfig/mrpt-${name}.pc" @ONLY)

		# (2/2) And another .pc file for local usage:
		SET(mrpt_pkgconfig_NO_INSTALL_SOURCE "${MRPT_SOURCE_DIR}")
		SET(mrpt_pkgconfig_NO_INSTALL_BINARY "${MRPT_BINARY_DIR}")
		CONFIGURE_FILE("${CMAKE_SOURCE_DIR}/parse-files/mrpt_template_no_install.pc.in" "${CMAKE_BINARY_DIR}/pkgconfig-no-install/mrpt-${name}.pc" @ONLY)
		
	ENDIF(UNIX)

	# --- End of conditional build of module ---
	ENDIF(BUILD_mrpt-${name}) 

endmacro(internal_define_mrpt_lib)

