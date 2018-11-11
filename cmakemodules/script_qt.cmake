# Check for Qt
# If it is found, will set CMAKE_MRPT_HAS_Qt5=1

set(USE_QT ON CACHE BOOL "Build Qt")
set(CMAKE_MRPT_HAS_Qt5 0)

if (USE_QT)
	set(QT_MRPT_COMPONENTS_TO_SEARCH "Gui;Widgets;Core;OpenGL" CACHE STRING "Components to search in Qt")

	find_package(Qt5 COMPONENTS ${QT_MRPT_COMPONENTS_TO_SEARCH})

	if (Qt5Core_FOUND)
		set(CMAKE_MRPT_HAS_Qt5 1)
		list(APPEND Qt5_COMPONENTS_LIBS "")
		list(APPEND Qt5_COMPONENTS_INCLUDE_DIRS "")

		set(location_type LOCATION_RELEASE)
		if ("${CMAKE_BUILD_TYPE}" STREQUAL "Debug")
			set(location_type LOCATION_DEBUG)
		endif()

		foreach(component ${QT_MRPT_COMPONENTS_TO_SEARCH})
			get_target_property(Qt5_component_lib "Qt5::${component}" ${location_type})
			list(APPEND Qt5_COMPONENTS_LIBS "${Qt5_component_lib}")

			get_target_property(Qt5_component_incl "Qt5::${component}" "INTERFACE_INCLUDE_DIRECTORIES")
			list(APPEND Qt5_COMPONENTS_INCLUDE_DIRS "${Qt5_component_incl}")
		endforeach()

		if($ENV{VERBOSE})
			message(STATUS "Qt5 link libs: ${Qt5_COMPONENTS_LIBS}")
			message(STATUS "Qt5 include dirs: ${Qt5_COMPONENTS_INCLUDE_DIRS}")
		endif()

	endif(Qt5Core_FOUND)
endif()


# -- install DLLs --
if(WIN32)
	if(${CMAKE_MRPT_HAS_Qt5})
		foreach(F ${Qt5_COMPONENTS_LIBS})
			install(FILES "${F}" DESTINATION bin)
		endforeach()
	endif()
endif()
