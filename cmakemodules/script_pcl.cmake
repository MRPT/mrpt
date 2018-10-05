
set(CMAKE_MRPT_HAS_PCL 0)
set(CMAKE_MRPT_HAS_PCL_SYSTEM 0)

# Leave at the user's choice to disable the SWR libs:
option(DISABLE_PCL "Forces NOT using PCL, even if it could be found by CMake" "OFF")
mark_as_advanced(DISABLE_PCL)

if(NOT DISABLE_PCL)
	# It seems the PCL find_package changes some wxWidgets variables (wtf!), so make a backup:
	#set(BCK_wxWidgets_LIB_DIR ${wxWidgets_LIB_DIR})
	#set(BCK_wxWidgets_LIBRARIES ${wxWidgets_LIBRARIES})

	# PCL library:
	# --------------------------------------------
	find_package(PCL COMPONENTS io common registration visualization segmentation surface QUIET)
	if(PCL_FOUND AND PCL_IO_FOUND AND PCL_COMMON_FOUND AND PCL_REGISTRATION_FOUND AND PCL_VISUALIZATION_FOUND AND PCL_SURFACE_FOUND)

		# Filter: It seems clang + c++11 fails to build against PCL (for clang <3.5):
		if (${CMAKE_CXX_COMPILER_ID} STREQUAL "Clang" AND NOT ${CMAKE_CXX_COMPILER_VERSION} VERSION_GREATER "3.4")
			message(WARNING "Disabling PCL because of incompatibility between clang ${CMAKE_CXX_COMPILER_VERSION} and PCL with c++11 enabled")
			set(DISABLE_PCL ON)
			return()
		endif ()

		set(CMAKE_MRPT_HAS_PCL 1)
		set(CMAKE_MRPT_HAS_PCL_SYSTEM 1)

		include_directories(${PCL_INCLUDE_DIRS})
		link_directories(${PCL_LIBRARY_DIRS})
		add_definitions(${PCL_DEFINITIONS})

		if (NOT Boost_FOUND)
			message(FATAL_ERROR "PCL requires Boost. Either disable PCL (with DISABLE_PCL=ON) or, to fix the error, create the entries BOOST_ROOT and BOOST_LIBRARYDIR and set them to the correct values")
		endif (NOT Boost_FOUND)

		if($ENV{VERBOSE})
			message(STATUS "PCL:")
			message(STATUS " Include dirs: ${PCL_INCLUDE_DIRS}")
			message(STATUS " Library dirs: ${PCL_LIBRARY_DIRS}")
			message(STATUS " Definitions : ${PCL_DEFINITIONS}")
			message(STATUS " Libraries   : ${PCL_LIBRARIES}")
		endif($ENV{VERBOSE})

		# Add PCL directories as "-isystem" to avoid warnings:
		ADD_DIRECTORIES_AS_ISYSTEM(PCL_INCLUDE_DIRS)

	endif()

	# Undo backups: (read above)
	#set(wxWidgets_LIB_DIR ${BCK_wxWidgets_LIB_DIR})
	#set(wxWidgets_LIBRARIES ${BCK_wxWidgets_LIBRARIES})

endif()
