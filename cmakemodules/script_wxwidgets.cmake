# Check for wxWidgets + GL
#  If it is found, will set CMAKE_MRPT_HAS_WXWIDGETS=1
# ===================================================

# Here you can define what libraries of wxWidgets you need for your application.
#  You can figure out what libraries you need here;  http://www.wxwidgets.org/manuals/2.8/wx_librarieslist.html
SET(wxWidgets_MRPT_COMPONENTS_TO_SEARCH "base;core;gl;adv;aui;html" CACHE STRING "Components to search in wxWidgets")

IF(UNIX)
	# Linux:
	IF(CMAKE_BUILD_TYPE MATCHES "Debug")
		SET(wxWidgets_USE_DEBUG ON CACHE BOOL "Use wxWidgets debug libs" FORCE)
	ENDIF(CMAKE_BUILD_TYPE MATCHES "Debug")
ELSE(UNIX)
	# Windows configuration of wxWidgets:
	SET(wxWidgets_USE_REL_AND_DBG ON)
ENDIF(UNIX)

# Make things easier: If we are building for MSVC, select the vc_lib or vc_dll directory automatically:
if(msvc)
	if(BUILD_SHARED_LIBS)
		set(wxWidgets_LIB_DIR "${wxWidgets_ROOT_DIR}/lib/vc_dll" FORCE)
	else(BUILD_SHARED_LIBS)
		set(wxWidgets_LIB_DIR "${wxWidgets_ROOT_DIR}/lib/vc_lib" FORCE)
	endif(BUILD_SHARED_LIBS)
endif(msvc)


# We need the Find package for wxWidgets to work
FIND_PACKAGE(wxWidgets COMPONENTS ${wxWidgets_MRPT_COMPONENTS_TO_SEARCH})
# Did we find wxWidgets ? This condition will fail for as long as the internal vars do not point to the proper wxWidgets configuration
IF(wxWidgets_FOUND)
	# Check for non-found debug libs:
	IF(UNIX)
		IF(CMAKE_BUILD_TYPE MATCHES "Debug")
			IF(NOT wxWidgets_USE_DEBUG)
				MESSAGE("Warning: The debug libraries for wxWidgets couldn't be found by CMake. Please install them (libwxgtk2.8-dbg) or build in release.")
			ENDIF(NOT wxWidgets_USE_DEBUG)
		ENDIF(CMAKE_BUILD_TYPE MATCHES "Debug")
	ENDIF(UNIX)

	# Include wxWidgets macros
	INCLUDE(${wxWidgets_USE_FILE})
	# ${wxWidgets_LIBRARIES}  will contain the libraries that should be added through TARGET_LINK_LIBRARIES(...)
	IF($ENV{VERBOSE})
		MESSAGE(STATUS "wxWidgets:")
		MESSAGE(STATUS "WX: ${wxWidgets_LIBRARIES}")
		MESSAGE(STATUS "   wxWidgets_LIBRARY_DIRS  : ${wxWidgets_LIBRARY_DIRS}")
		MESSAGE(STATUS "   wxWidgets_CXX_FLAGS     : ${wxWidgets_CXX_FLAGS}")
		MESSAGE(STATUS "   wxWidgets_INCLUDE_DIRS  : ${wxWidgets_INCLUDE_DIRS}")
	ENDIF($ENV{VERBOSE})

	LINK_DIRECTORIES(${wxWidgets_LIBRARY_DIRS})

	IF(MSVC)
		ADD_DEFINITIONS(-DwxUSE_NO_MANIFEST=1)
	ENDIF(MSVC)

	ADD_DIRECTORIES_AS_INCLUDE_AND_ISYSTEM(wxWidgets_INCLUDE_DIRS)

	# For use in the MRPTconfig.cmake
	SET(CMAKE_MRPT_WX_ROOT_DIR ${wxWidgets_ROOT_DIR})

	SET(CMAKE_MRPT_HAS_WXWIDGETS 1)
ELSE(wxWidgets_FOUND)
	# Monolithic?
	IF (WX_mono)
		SET(wxWidgets_MRPT_COMPONENTS_TO_SEARCH mono CACHE STRING "Components to search in wxWidgets" FORCE)
		FIND_PACKAGE(wxWidgets COMPONENTS ${wxWidgets_MRPT_COMPONENTS_TO_SEARCH})

		INCLUDE(${wxWidgets_USE_FILE})
		# ${wxWidgets_LIBRARIES}  will contain the libraries that should be added through TARGET_LINK_LIBRARIES(...)
		#MESSAGE(STATUS "wxWidgets_LIBRARIES: ${wxWidgets_LIBRARIES}")
		#MESSAGE(STATUS "wxWidgets_CXX_FLAGS: ${wxWidgets_CXX_FLAGS}")

		LINK_DIRECTORIES(${wxWidgets_LIBRARY_DIRS})

		IF(MSVC)
			ADD_DEFINITIONS(-DwxUSE_NO_MANIFEST=1)
		ENDIF(MSVC)

		# For use in the MRPTconfig.cmake
		SET(CMAKE_MRPT_WX_ROOT_DIR ${wxWidgets_ROOT_DIR})

		SET(CMAKE_MRPT_HAS_WXWIDGETS 1)

	ELSE(WX_mono)
		MESSAGE("wxWidgets was not found automatically. Please, set wxWidgets_ROOT_DIR to the lib directory to enable it in MRPT.")
		SET(CMAKE_MRPT_HAS_WXWIDGETS 0)
	ENDIF(WX_mono)
ENDIF(wxWidgets_FOUND)

# DISABLE_WXWIDGETS
# ---------------------
SET( DISABLE_WXWIDGETS OFF CACHE BOOL "Forces compilation WITHOUT wxWidgets")
MARK_AS_ADVANCED(DISABLE_WXWIDGETS)
IF(DISABLE_WXWIDGETS)
	SET(CMAKE_MRPT_HAS_WXWIDGETS 0)
	SET(wxWidgets_LIBRARIES "")
ENDIF(DISABLE_WXWIDGETS)

# WXWIDGETS+GL: Add libraries
# -------------------------------------------
IF(CMAKE_MRPT_HAS_WXWIDGETS)
	LIST(LENGTH wxWidgets_LIBRARIES wxLibsCount)
	if (wxLibsCount GREATER 0)
		#APPEND_MRPT_LIBS(${wxWidgets_LIBRARIES})
	endif(wxLibsCount GREATER 0)
ENDIF(CMAKE_MRPT_HAS_WXWIDGETS)

# It's always a system lib:
SET(CMAKE_MRPT_HAS_WXWIDGETS_SYSTEM 0)
IF(CMAKE_MRPT_HAS_WXWIDGETS)
	SET(CMAKE_MRPT_HAS_WXWIDGETS_SYSTEM 1)
ENDIF(CMAKE_MRPT_HAS_WXWIDGETS)

