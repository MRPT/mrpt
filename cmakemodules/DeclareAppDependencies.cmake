# Declares the dependencies of an application:
# Usage: DeclareAppDependencies( appTargetName [mrpt-xxx [mrpt-yyy] ...] )
#
macro(DeclareAppDependencies name)
	# Set app names:
	if(ENABLE_SOLUTION_FOLDERS)
		set_target_properties(${name} PROPERTIES FOLDER "applications")
	else(ENABLE_SOLUTION_FOLDERS)
		SET_TARGET_PROPERTIES(${name} PROPERTIES PROJECT_LABEL "(APP) ${name}")
	endif(ENABLE_SOLUTION_FOLDERS)

	# set deps:
	set(ALL_DEPS "")

	FOREACH(DEP ${ARGN})
		LIST(APPEND ALL_DEPS ${DEP})
	ENDFOREACH(DEP)

	FOREACH(DEP ${ARGN})
		# Only for "mrpt-XXX" libs:
		IF (${DEP} MATCHES "mrpt-")
			get_property(LIB_DEP GLOBAL PROPERTY "${DEP}_LIB_DEPS")
			LIST(APPEND ALL_DEPS ${LIB_DEP})
		ENDIF (${DEP} MATCHES "mrpt-")
	ENDFOREACH(DEP)


	# Add the detected dependencies:
	list(REMOVE_DUPLICATES ALL_DEPS)

	set(AUX_ALL_DEPS_BUILD 1)  # Will be set to "0" if any dependency if not built

	IF (NOT "${ALL_DEPS}" STREQUAL "")
		#MESSAGE(STATUS "Adding deps: ${name} --> ${ALL_DEPS}")
		ADD_DEPENDENCIES(${name} ${ALL_DEPS})

		FOREACH (_DEP ${ALL_DEPS})
			# Link:
			get_property(_LIB_HDRONLY GLOBAL PROPERTY "${_DEP}_LIB_IS_HEADERS_ONLY")
			if (NOT _LIB_HDRONLY)
				TARGET_LINK_LIBRARIES(${name} ${_DEP}${MRPT_LINKER_LIBS_POSTFIX})
				LIST(APPEND AUX_EXTRA_LINK_LIBS
					optimized ${MRPT_LIB_PREFIX}${DEP}${MRPT_DLL_VERSION_POSTFIX}
					debug     ${MRPT_LIB_PREFIX}${DEP}${MRPT_DLL_VERSION_POSTFIX}${CMAKE_DEBUG_POSTFIX})
			endif (NOT _LIB_HDRONLY)

			# Include:
			STRING(REGEX REPLACE "mrpt-(.*)" "\\1" DEP_MRPT_NAME ${_DEP})
			IF(NOT "${DEP_MRPT_NAME}" STREQUAL "")
				INCLUDE_DIRECTORIES("${MRPT_LIBS_ROOT}/${DEP_MRPT_NAME}/include/")
			ENDIF(NOT "${DEP_MRPT_NAME}" STREQUAL "")

			# Check if all dependencies are to be build:
			if (${BUILD_mrpt-${DEP_MRPT_NAME}} STREQUAL "OFF")
				SET(AUX_ALL_DEPS_BUILD 0)
				MESSAGE(STATUS "*Warning*: App ${name} cannot be built because dependency mrpt-${DEP_MRPT_NAME} has been disabled!")
			endif (${BUILD_mrpt-${DEP_MRPT_NAME}} STREQUAL "OFF")

		ENDFOREACH (_DEP)
	ENDIF ()

	# Impossible to build?
	if (NOT AUX_ALL_DEPS_BUILD)
		MESSAGE(STATUS "*Warning* ==> Forcing BUILD_APP_${name}=OFF for missing dependencies listed above (re-enable manually if needed).")
		SET(BUILD_APP_${name} OFF CACHE BOOL "Build ${name}" FORCE) # this var is checked in [MRPT]/app/CMakeLists.txt
		mark_as_advanced(CLEAR BUILD_APP_${name})
	endif (NOT AUX_ALL_DEPS_BUILD)

endmacro(DeclareAppDependencies)

# Macro for adding links to the Start menu folder (for binary packages in Windows)
macro(AppStartMenuLink name title)
	get_property(_str GLOBAL PROPERTY "MRPT_CPACK_PACKAGE_EXECUTABLES")
	set_property(GLOBAL PROPERTY "MRPT_CPACK_PACKAGE_EXECUTABLES" "${_str}${name};${title};")
endmacro(AppStartMenuLink)


macro(DeclareAppForInstall name)
INSTALL(TARGETS ${name}
	RUNTIME DESTINATION ${mrpt_apps_INSTALL_PREFIX}bin COMPONENT Apps
	LIBRARY DESTINATION ${mrpt_apps_INSTALL_PREFIX}lib${LIB_SUFFIX} COMPONENT Apps
	ARCHIVE DESTINATION ${mrpt_apps_INSTALL_PREFIX}lib${LIB_SUFFIX} COMPONENT Apps)
endmacro(DeclareAppForInstall)
