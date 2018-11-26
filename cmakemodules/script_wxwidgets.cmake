# Check for wxWidgets + GL
#  If it is found, will set CMAKE_MRPT_HAS_WXWIDGETS=1
# ===================================================

# Here you can define what libraries of wxWidgets you need for your application.
#  You can figure out what libraries you need here;  http://www.wxwidgets.org/manuals/2.8/wx_librarieslist.html
set(wxWidgets_MRPT_COMPONENTS_TO_SEARCH "base;core;gl;adv;aui;html" CACHE STRING "Components to search in wxWidgets")

if(UNIX)
	# Linux:
	if(CMAKE_BUILD_TYPE MATCHES "Debug")
		set(wxWidgets_USE_DEBUG ON CACHE BOOL "Use wxWidgets debug libs" FORCE)
	endif()
else(UNIX)
	# Windows configuration of wxWidgets:
	set(wxWidgets_USE_REL_AND_DBG ON)
endif(UNIX)

# Select wx toolkit options, like GTK2 vs GTK3, etc.
if(UNIX)
	# If available, prefer gtk3:
	execute_process(
		COMMAND wx-config --selected-config --toolkit=gtk3
		RESULT_VARIABLE ret
		OUTPUT_QUIET
	)
	if(ret EQUAL "0")
		message(STATUS "wxWidgets: Found gtk3 version, using it.")
		set(wxWidgets_CONFIG_OPTIONS_DEFAULT "--toolkit=gtk3")
	else()
		message(STATUS "wxWidgets: No gtk3 version found, falling back to default (likely gtk2)")
	endif()
	unset(ret)

	set(wxWidgets_CONFIG_OPTIONS "${wxWidgets_CONFIG_OPTIONS_DEFAULT}" ON STRING "wxWidgets toolkit options")
endif()

# We need the Find package for wxWidgets to work
find_package(wxWidgets COMPONENTS ${wxWidgets_MRPT_COMPONENTS_TO_SEARCH})
# Did we find wxWidgets ? This condition will fail for as long as the internal vars do not point to the proper wxWidgets configuration
if(wxWidgets_FOUND)
	# Check for non-found debug libs:
	if(UNIX)
		if(CMAKE_BUILD_TYPE MATCHES "Debug")
			if(NOT wxWidgets_USE_DEBUG)
				message("Warning: The debug libraries for wxWidgets couldn't be found by CMake. Please install them (libwxgtk2.8-dbg) or build in release.")
			endif(NOT wxWidgets_USE_DEBUG)
		endif(CMAKE_BUILD_TYPE MATCHES "Debug")
	endif(UNIX)

	# Include wxWidgets macros
	include(${wxWidgets_USE_FILE})
	# ${wxWidgets_LIBRARIES}  will contain the libraries that should be added through target_link_libraries(...)
	if($ENV{VERBOSE})
		message(STATUS "wxWidgets:")
		message(STATUS "WX: ${wxWidgets_LIBRARIES}")
		message(STATUS "   wxWidgets_LIBRARY_DIRS  : ${wxWidgets_LIBRARY_DIRS}")
		message(STATUS "   wxWidgets_CXX_FLAGS     : ${wxWidgets_CXX_FLAGS}")
		message(STATUS "   wxWidgets_INCLUDE_DIRS  : ${wxWidgets_INCLUDE_DIRS}")
	endif($ENV{VERBOSE})

	link_directories(${wxWidgets_LIBRARY_DIRS})

	if(MSVC)
		add_definitions(-DwxUSE_NO_MANIFEST=1)
	endif(MSVC)

	ADD_DIRECTORIES_AS_INCLUDE_AND_ISYSTEM(wxWidgets_INCLUDE_DIRS)

	# For use in the MRPTconfig.cmake
	set(CMAKE_MRPT_WX_ROOT_DIR ${wxWidgets_ROOT_DIR})

	set(CMAKE_MRPT_HAS_WXWIDGETS 1)
else(wxWidgets_FOUND)
	# Monolithic?
	if (WX_mono)
		set(wxWidgets_MRPT_COMPONENTS_TO_SEARCH mono CACHE STRING "Components to search in wxWidgets" FORCE)
		find_package(wxWidgets COMPONENTS ${wxWidgets_MRPT_COMPONENTS_TO_SEARCH})

		include(${wxWidgets_USE_FILE})
		# ${wxWidgets_LIBRARIES}  will contain the libraries that should be added through target_link_libraries(...)
		#message(STATUS "wxWidgets_LIBRARIES: ${wxWidgets_LIBRARIES}")
		#message(STATUS "wxWidgets_CXX_FLAGS: ${wxWidgets_CXX_FLAGS}")

		link_directories(${wxWidgets_LIBRARY_DIRS})

		if(MSVC)
			add_definitions(-DwxUSE_NO_MANIFEST=1)
		endif(MSVC)

		# For use in the MRPTconfig.cmake
		set(CMAKE_MRPT_WX_ROOT_DIR ${wxWidgets_ROOT_DIR})

		set(CMAKE_MRPT_HAS_WXWIDGETS 1)

	else(WX_mono)
		message("wxWidgets was not found automatically. Please, set wxWidgets_ROOT_DIR to the lib directory to enable it in MRPT.")
		set(CMAKE_MRPT_HAS_WXWIDGETS 0)
	endif(WX_mono)
endif(wxWidgets_FOUND)

# DISABLE_WXWIDGETS
# ---------------------
set( DISABLE_WXWIDGETS OFF CACHE BOOL "Forces compilation WITHOUT wxWidgets")
mark_as_advanced(DISABLE_WXWIDGETS)
if(DISABLE_WXWIDGETS)
	set(CMAKE_MRPT_HAS_WXWIDGETS 0)
	set(wxWidgets_LIBRARIES "")
endif(DISABLE_WXWIDGETS)

# WXWIDGETS+GL: Add libraries
# -------------------------------------------
if(CMAKE_MRPT_HAS_WXWIDGETS)
	list(LENGTH wxWidgets_LIBRARIES wxLibsCount)
	if (wxLibsCount GREATER 0)
		#APPEND_MRPT_LIBS(${wxWidgets_LIBRARIES})
	endif(wxLibsCount GREATER 0)
endif(CMAKE_MRPT_HAS_WXWIDGETS)

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
