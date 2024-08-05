# Declares the dependencies of an application:
# Usage: DeclareAppDependencies( appTargetName [mrpt-xxx [mrpt-yyy] ...] )
#
add_custom_target(apps_all ALL)

macro(mrpt_return_if_not_wxwidgets)
	if(NOT CMAKE_MRPT_HAS_WXWIDGETS)
		message(STATUS "No wxWidgets: Skipping target `${PROJECT_NAME}`")
		return()
	endif()
endmacro()

macro(mrpt_return_if_not_nanogui)
	if(NOT CMAKE_MRPT_HAS_NANOGUI)
		message(STATUS "Build without nanogui: Skipping target `${PROJECT_NAME}`")
		return()
	endif()
endmacro()

macro(DeclareAppDependencies name)
	add_dependencies(apps_all ${name})

	# Set app names:
	set_target_properties(${name} PROPERTIES FOLDER "applications")

	set(ALL_DEPS ${ARGN})
	#message(STATUS "Adding deps: ${name} --> ${ALL_DEPS}")

	set(AUX_ALL_DEPS_BUILD 1)  # Will be set to "0" if any dependency if not built

	foreach (_DEP ${ALL_DEPS})
		# Check if all dependencies are to be build:
		string(REGEX REPLACE "mrpt::(.*)" "\\1" DEP_MRPT_NAME ${_DEP})

		if ("${BUILD_mrpt-${DEP_MRPT_NAME}}" STREQUAL "OFF")
			# Attempt at using system version?
			find_package(mrpt-${DEP_MRPT_NAME})
			if (NOT TARGET mrpt::${DEP_MRPT_NAME})
				set(AUX_ALL_DEPS_BUILD 0)
				message(STATUS "*Warning*: App ${name} cannot be built because dependency mrpt-${DEP_MRPT_NAME} has been disabled!")
			else()
				message(STATUS "*Warning*: App ${name} is using system depency: mrpt-${DEP_MRPT_NAME}")
			endif()
		endif()
	endforeach()

	# Impossible to build?
	if (NOT AUX_ALL_DEPS_BUILD)
		message(STATUS "*Warning* ==> Forcing BUILD_APP_${name}=OFF for missing dependencies listed above (re-enable manually if needed).")
		set(BUILD_APP_${name} OFF CACHE BOOL "Build ${name}" FORCE) # this var is checked in [MRPT]/app/CMakeLists.txt
		mark_as_advanced(CLEAR BUILD_APP_${name})
		return()
	endif()

	# We need pthread's on unices
	target_link_libraries(${name} Threads::Threads)

	add_dependencies(${name} ${ALL_DEPS})
	target_link_libraries(${name} ${ALL_DEPS})

endmacro()

# Macro for adding links to the Start menu folder (for binary packages in Windows)
macro(AppStartMenuLink name title)
	get_property(_str GLOBAL PROPERTY "MRPT_CPACK_PACKAGE_EXECUTABLES")
	set_property(GLOBAL PROPERTY "MRPT_CPACK_PACKAGE_EXECUTABLES" "${_str}${name};${title};")
endmacro()


macro(DeclareAppForInstall name)
install(TARGETS ${name}
	RUNTIME DESTINATION bin COMPONENT Apps
	LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR} COMPONENT Apps
	ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR} COMPONENT Apps)
endmacro()
