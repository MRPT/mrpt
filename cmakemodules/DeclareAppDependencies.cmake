# Declares the dependencies of an application:
# Usage: DeclareAppDependencies( appTargetName [mrpt-xxx [mrpt-yyy] ...] )
#
add_custom_target(apps_all ALL)

macro(DeclareAppDependencies name)
	add_dependencies(apps_all ${name})

	# Set app names:
	set_target_properties(${name} PROPERTIES FOLDER "applications")

	# set deps:
	set(ALL_DEPS "")

	foreach(DEP ${ARGN})
		# Only for "mrpt-XXX" libs:
		if (DEP MATCHES "^mrpt-?:*(.*)")
			# First, add the lib itself:
			string(REGEX REPLACE "mrpt-?:*(.*)" "\\1" DEP_MRPT_NAME ${DEP})
			list(APPEND ALL_DEPS mrpt::${DEP_MRPT_NAME})
			# now, their dependencies:
			get_property(LIB_DEP GLOBAL PROPERTY "${DEP}_LIB_DEPS")
			list(APPEND ALL_DEPS ${LIB_DEP})
		endif ()
	endforeach()


	# Add the detected dependencies:
	list(REMOVE_DUPLICATES ALL_DEPS)

	set(AUX_ALL_DEPS_BUILD 1)  # Will be set to "0" if any dependency if not built

	if (NOT "${ALL_DEPS}" STREQUAL "")
		#message(STATUS "Adding deps: ${name} --> ${ALL_DEPS}")
		add_dependencies(${name} ${ALL_DEPS})

		foreach (_DEP ${ALL_DEPS})
			# Link:
			get_property(_LIB_HDRONLY GLOBAL PROPERTY "${_DEP}_LIB_IS_HEADERS_ONLY")
			if (NOT _LIB_HDRONLY)
				target_link_libraries(${name} ${_DEP}${MRPT_LINKER_LIBS_POSTFIX})
				list(APPEND AUX_EXTRA_LINK_LIBS
					optimized ${MRPT_LIB_PREFIX}${DEP}${MRPT_DLL_VERSION_POSTFIX}
					debug     ${MRPT_LIB_PREFIX}${DEP}${MRPT_DLL_VERSION_POSTFIX}${CMAKE_DEBUG_POSTFIX})
			endif ()

			# Include:
			string(REGEX REPLACE "mrpt::(.*)" "\\1" DEP_MRPT_NAME ${_DEP})
			if(NOT "${DEP_MRPT_NAME}" STREQUAL "")
				include_directories("${MRPT_LIBS_ROOT}/${DEP_MRPT_NAME}/include/")
			endif()

			# Check if all dependencies are to be build:
			if (${BUILD_mrpt-${DEP_MRPT_NAME}} STREQUAL "OFF")
				set(AUX_ALL_DEPS_BUILD 0)
				message(STATUS "*Warning*: App ${name} cannot be built because dependency mrpt-${DEP_MRPT_NAME} has been disabled!")
			endif ()

		endforeach (_DEP)
	endif ()

	# Impossible to build?
	if (NOT AUX_ALL_DEPS_BUILD)
		message(STATUS "*Warning* ==> Forcing BUILD_APP_${name}=OFF for missing dependencies listed above (re-enable manually if needed).")
		set(BUILD_APP_${name} OFF CACHE BOOL "Build ${name}" FORCE) # this var is checked in [MRPT]/app/CMakeLists.txt
		mark_as_advanced(CLEAR BUILD_APP_${name})
	endif ()

	# We need pthread's on unices
	target_link_libraries(${name} Threads::Threads)

endmacro()

# Macro for adding links to the Start menu folder (for binary packages in Windows)
macro(AppStartMenuLink name title)
	get_property(_str GLOBAL PROPERTY "MRPT_CPACK_PACKAGE_EXECUTABLES")
	set_property(GLOBAL PROPERTY "MRPT_CPACK_PACKAGE_EXECUTABLES" "${_str}${name};${title};")
endmacro()


macro(DeclareAppForInstall name)
install(TARGETS ${name}
	RUNTIME DESTINATION ${mrpt_apps_INSTALL_PREFIX}bin COMPONENT Apps
	LIBRARY DESTINATION ${mrpt_apps_INSTALL_PREFIX}lib${LIB_SUFFIX} COMPONENT Apps
	ARCHIVE DESTINATION ${mrpt_apps_INSTALL_PREFIX}lib${LIB_SUFFIX} COMPONENT Apps)
endmacro()
