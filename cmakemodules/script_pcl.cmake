# It seems the PCL find_package changes some wxWidgets variables (wtf!), so make a backup:
#set(BCK_wxWidgets_LIB_DIR ${wxWidgets_LIB_DIR})
#set(BCK_wxWidgets_LIBRARIES ${wxWidgets_LIBRARIES})

# PCL library:
# --------------------------------------------
SET(CMAKE_MRPT_HAS_PCL 0)
find_package(PCL COMPONENTS io common registration QUIET)
if (PCL_FOUND)
	SET(CMAKE_MRPT_HAS_PCL 1)

	INCLUDE_DIRECTORIES(${PCL_INCLUDE_DIRS})
	link_directories(${PCL_LIBRARY_DIRS})
	add_definitions(${PCL_DEFINITIONS})

	# Add PCL directories as "-isystem" to avoid warnings:
	ADD_DIRECTORIES_AS_ISYSTEM(PCL_INCLUDE_DIRS)

endif(PCL_FOUND)

# Undo backups: (read above)
#set(wxWidgets_LIB_DIR ${BCK_wxWidgets_LIB_DIR})
#set(wxWidgets_LIBRARIES ${BCK_wxWidgets_LIBRARIES})
