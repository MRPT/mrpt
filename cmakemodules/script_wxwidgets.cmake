# Check for wxWidgets + GL
#  If it is found, will set CMAKE_MRPT_HAS_WXWIDGETS=1
# ===================================================

# Here you can define what libraries of wxWidgets you need for your application.
#  You can figure out what libraries you need here;  http://www.wxwidgets.org/manuals/2.8/wx_librarieslist.html
set(wxWidgets_MRPT_COMPONENTS_TO_SEARCH "base;core;gl;adv;aui;html" CACHE STRING "Components to search in wxWidgets")

set(CMAKE_MRPT_HAS_WXWIDGETS 0)
set(wxWidgets_LIBRARIES "")

set(DISABLE_WXWIDGETS OFF CACHE BOOL "Forces compilation WITHOUT wxWidgets")
mark_as_advanced(DISABLE_WXWIDGETS)
if(DISABLE_WXWIDGETS)
	return()
endif()

if((CMAKE_BUILD_TYPE MATCHES "Debug") OR (NOT UNIX))
	set(wxWidgets_USE_DEBUG ON CACHE BOOL "Use wxWidgets debug libs" FORCE)
endif()

# Select wx toolkit options, like GTK2 vs GTK3, etc.
if(UNIX)
	# If available, prefer gtk3:
	execute_process(
		COMMAND wx-config --selected-config --toolkit=gtk3
		RESULT_VARIABLE ret
		OUTPUT_QUIET
	)
	if(ret EQUAL "0")
		set(CMAKE_WXWIDGETS_TOOLKIT_NAME "(gtk3)")
		if ($ENV{VERBOSE})
			message(STATUS "wxWidgets: Found gtk3 version, using it.")
		endif()
		set(wxWidgets_CONFIG_OPTIONS_DEFAULT "--toolkit=gtk3")
	else()
		message(STATUS "wxWidgets: No gtk3 version found, falling back to default (likely gtk2)")
	endif()
	unset(ret)

	set(wxWidgets_CONFIG_OPTIONS "${wxWidgets_CONFIG_OPTIONS_DEFAULT}" ON STRING "wxWidgets toolkit options")
endif()

find_package(wxWidgets COMPONENTS ${wxWidgets_MRPT_COMPONENTS_TO_SEARCH})
# Did we find wxWidgets ? This condition will fail for as long as the internal vars do not point to the proper wxWidgets configuration
if(wxWidgets_FOUND)
	# Check for non-found debug libs:
	if(UNIX)
		if(CMAKE_BUILD_TYPE MATCHES "Debug")
			if(NOT wxWidgets_USE_DEBUG)
				message("Warning: The debug libraries for wxWidgets couldn't be found by CMake. Please install them (libwxgtk2.8-dbg) or build in release.")
			endif()
		endif()
	endif()

	# Include wxWidgets macros

	# *** include(${wxWidgets_USE_FILE})

	mrpt_split_lib_list(wxWidgets_LIBRARIES wxWidgets_LIBRARIES_RELEASE optimized debug)
	mrpt_split_lib_list(wxWidgets_LIBRARIES wxWidgets_LIBRARIES_DEBUG debug optimized)

	add_library(imp_wxwidgets INTERFACE IMPORTED)
	set_target_properties(imp_wxwidgets
		PROPERTIES
		INTERFACE_INCLUDE_DIRECTORIES "${wxWidgets_INCLUDE_DIRS}"
		INTERFACE_LINK_LIBRARIES "$<$<CONFIG:Debug>:${wxWidgets_LIBRARIES_DEBUG}>$<$<NOT:$<CONFIG:Debug>>:${wxWidgets_LIBRARIES_RELEASE}>"
		INTERFACE_COMPILE_OPTIONS "${wxWidgets_CXX_FLAGS}"
		INTERFACE_COMPILE_DEFINITIONS "${wxWidgets_DEFINITIONS}"
		)


	if($ENV{VERBOSE})
		message(STATUS "wxWidgets:")
		message(STATUS "   wxWidgets_LIBRARIES     : ${wxWidgets_LIBRARIES}")
		message(STATUS "   wxWidgets_LIBRARIES_RELEASE : ${wxWidgets_LIBRARIES_RELEASE}")
		message(STATUS "   wxWidgets_LIBRARIES_DEBUG   : ${wxWidgets_LIBRARIES_DEBUG}")
		message(STATUS "   wxWidgets_LIBRARY_DIRS  : ${wxWidgets_LIBRARY_DIRS}")
		message(STATUS "   wxWidgets_CXX_FLAGS     : ${wxWidgets_CXX_FLAGS}")
		message(STATUS "   wxWidgets_DEFINITIONS   : ${wxWidgets_DEFINITIONS}")
		message(STATUS "   wxWidgets_INCLUDE_DIRS  : ${wxWidgets_INCLUDE_DIRS}")
	endif()

	if(MSVC)
		set_property(
			TARGET imp_wxwidgets
			APPEND PROPERTY
			INTERFACE_COMPILE_DEFINITIONS "-DwxUSE_NO_MANIFEST=1"
			)
	endif()

	# For use in the MRPTconfig.cmake
	set(CMAKE_MRPT_WX_ROOT_DIR ${wxWidgets_ROOT_DIR})

	set(CMAKE_MRPT_HAS_WXWIDGETS 1)
endif()

# It's always a system lib:
set(CMAKE_MRPT_HAS_WXWIDGETS_SYSTEM 0)
if(CMAKE_MRPT_HAS_WXWIDGETS)
	set(CMAKE_MRPT_HAS_WXWIDGETS_SYSTEM 1)
endif(CMAKE_MRPT_HAS_WXWIDGETS)


# -- install DLLs --
if(WIN32)
	if (EXISTS "${wxWidgets_LIB_DIR}")
		file(GLOB_RECURSE EXTRA_DLLS "${wxWidgets_LIB_DIR}/*.dll")
		foreach(F ${EXTRA_DLLS})
			install(FILES "${F}" DESTINATION bin)
		endforeach()
	endif ()
endif()
