# Declares an MRPT library target:
macro(define_mrpt_lib name)

	INCLUDE(../../cmakemodules/AssureCMakeRootFile.cmake) # Avoid user mistake in CMake source directory
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
		"${CMAKE_SOURCE_DIR}/libs/${name}/include/mrpt/${name}/link_pragmas.h"
		"${CMAKE_SOURCE_DIR}/libs/${name}/src/registerAllClasses.cpp"
		)

	LIST(APPEND ${name}_EXTRA_SRCS_NAME
		"${name}"
		"${name}"
		"${name}"
		"${name}"
		"${name}"
		"${name}"
		"DLL link macros"
		"Class register"
		)

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
	
	ADD_LIBRARY(mrpt-${name}   ${all_${name}_srcs})

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
				
				# Link "-lmrpt-name", only for GCC:
				IF(CMAKE_COMPILER_IS_GNUCXX)
					LIST(APPEND AUX_EXTRA_LINK_LIBS ${DEP}${MRPT_LINKER_LIBS_POSTFIX})
				ENDIF(CMAKE_COMPILER_IS_GNUCXX)
				
				# Append to list of mrpt-* lib dependences:
				LIST(APPEND AUX_DEPS_LIST ${DEP})
			ENDIF(NOT "${DEP_MRPT_NAME}" STREQUAL "")
		ENDIF (${DEP} MATCHES "mrpt-")		
	ENDFOREACH(DEP)
	
	# Emulates a global variable:
	set_property(GLOBAL PROPERTY "mrpt-${name}_LIB_DEPS" "${AUX_DEPS_LIST}")

	# Dependencies between projects:
	IF(NOT "${ARGN}" STREQUAL "")
		ADD_DEPENDENCIES(mrpt-${name} ${ARGN})
	ENDIF(NOT "${ARGN}" STREQUAL "")

	TARGET_LINK_LIBRARIES(mrpt-${name} 
		${MRPTLIB_LINKER_LIBS}
		${AUX_EXTRA_LINK_LIBS}
		)

	SET_TARGET_PROPERTIES(mrpt-${name} PROPERTIES PROJECT_LABEL "(LIB) mrpt-${name}")

	# Set custom name of lib + dynamic link numbering convenions in Linux:
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
	
	#  Add precompiled headers options (based on OpenCV CMake scripts)
	# --------------------------------------------
if(0)
	IF(MRPT_ENABLE_PRECOMPILED_HDRS)
		if(PCHSupport_FOUND)
		    set(pch_header  ${CMAKE_SOURCE_DIR}/libs/${name}/include/mrpt/${name}.h)
			
		    if(${CMAKE_GENERATOR} MATCHES "Visual*" OR ${CMAKE_GENERATOR} MATCHES "Xcode*")
			if(${CMAKE_GENERATOR} MATCHES "Visual*")
			    set("mrpt-${name}_pch" "${CMAKE_SOURCE_DIR}/libs/${name}/src/precomp_hdr.cpp")
			endif(${CMAKE_GENERATOR} MATCHES "Visual*" OR ${CMAKE_GENERATOR} MATCHES "Xcode*")
			add_native_precompiled_header(mrpt-${name} ${pch_header})
		    elseif(CMAKE_COMPILER_IS_GNUCXX AND ${CMAKE_GENERATOR} MATCHES ".*Makefiles")
			add_precompiled_header(mrpt-${name} ${pch_header})
		    endif(${CMAKE_GENERATOR} MATCHES "Visual*" OR ${CMAKE_GENERATOR} MATCHES "Xcode*")
		endif(PCHSupport_FOUND)
	ENDIF(MRPT_ENABLE_PRECOMPILED_HDRS)
endif(0)

if(1)
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
endif(1)

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

	# Generate the libmrpt-$NAME.pc file for pkg-config:
	IF(UNIX)
		# TODO: Declare some more vars, etc...
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

		# Generate the .pc file for "make install"
		CONFIGURE_FILE("${CMAKE_SOURCE_DIR}/parse-files/mrpt_template.pc.in" "${CMAKE_BINARY_DIR}/pkgconfig/mrpt-${name}.pc" @ONLY)

		# And another .pc file for local usage:
		SET(mrpt_pkgconfig_NO_INSTALL_SOURCE "${MRPT_SOURCE_DIR}")
		SET(mrpt_pkgconfig_NO_INSTALL_BINARY "${MRPT_BINARY_DIR}")
		CONFIGURE_FILE("${CMAKE_SOURCE_DIR}/parse-files/mrpt_template_no_install.pc.in" "${CMAKE_BINARY_DIR}/pkgconfig-no-install/mrpt-${name}.pc" @ONLY)
		
	ENDIF(UNIX)

endmacro(define_mrpt_lib)
