
SET(CMAKE_MRPT_HAS_PCL 0)
SET(CMAKE_MRPT_HAS_PCL_SYSTEM 0)

# Leave at the user's choice to disable the SWR libs:
OPTION(DISABLE_PCL "Forces NOT using PCL, even if it could be found by CMake" "OFF")
MARK_AS_ADVANCED(DISABLE_PCL)
IF(NOT DISABLE_PCL)
	# It seems the PCL find_package changes some wxWidgets variables (wtf!), so make a backup:
	#set(BCK_wxWidgets_LIB_DIR ${wxWidgets_LIB_DIR})
	#set(BCK_wxWidgets_LIBRARIES ${wxWidgets_LIBRARIES})

	# PCL library:
	# --------------------------------------------
	find_package(PCL COMPONENTS io common registration visualization segmentation surface QUIET)
	if (PCL_FOUND)
		SET(CMAKE_MRPT_HAS_PCL 1)
		SET(CMAKE_MRPT_HAS_PCL_SYSTEM 1)

		INCLUDE_DIRECTORIES(${PCL_INCLUDE_DIRS})
		link_directories(${PCL_LIBRARY_DIRS})
		add_definitions(${PCL_DEFINITIONS})
		
		IF (NOT Boost_FOUND)
			MESSAGE("*MRPT ERROR MESSAGE*: PCL requires Boost. Either disable PCL (with DISABLE_PCL=ON) or, to fix the error, create the entries BOOST_ROOT and BOOST_LIBRARYDIR and set them to the correct values")
		ENDIF (NOT Boost_FOUND)

		MESSAGE(STATUS "PCL:")
		MESSAGE(STATUS " Include dirs: ${PCL_INCLUDE_DIRS}")
		MESSAGE(STATUS " Library dirs: ${PCL_LIBRARY_DIRS}")
		MESSAGE(STATUS " Definitions : ${PCL_DEFINITIONS}")
		MESSAGE(STATUS " Libraries   : ${PCL_LIBRARIES}")
		
		# Add PCL directories as "-isystem" to avoid warnings:
		ADD_DIRECTORIES_AS_ISYSTEM(PCL_INCLUDE_DIRS)

	endif(PCL_FOUND)

	# Undo backups: (read above)
	#set(wxWidgets_LIB_DIR ${BCK_wxWidgets_LIB_DIR})
	#set(wxWidgets_LIBRARIES ${BCK_wxWidgets_LIBRARIES})
	
ENDIF(NOT DISABLE_PCL)
