INCLUDE(../../cmakemodules/AssureCMakeRootFile.cmake) # Avoid user mistake in CMake source directory

#-----------------------------------------------------------------
# CMake file for the MRPT application:  navlog-viewer
#
#  Run with "cmake ." at the root directory
#
#  Feb 2010, Jose Luis Blanco <jlblanco@ctima.uma.es>
#-----------------------------------------------------------------

PROJECT(navlog-viewer)

IF(wxWidgets_FOUND)
	# ---------------------------------------------
	# TARGET:
	# ---------------------------------------------
	SET(SRCS
		navlog_viewer_GUI_designMain.h navlog_viewer_GUI_designMain.cpp
		navlog_viewer_GUI_designApp.h  navlog_viewer_GUI_designApp.cpp
		CAboutBox.h CAboutBox.cpp
	 	)

	IF(WIN32)
		SET(SRCS ${SRCS} resource.rc)
	ENDIF(WIN32)

	INCLUDE_DIRECTORIES("${MRPT_SOURCE_DIR}/otherlibs/wxThings/")


	# Define the executable target:
	ADD_EXECUTABLE(navlog-viewer WIN32 ${SRCS})

	SET(TMP_TARGET_NAME "navlog-viewer")

	SET_TARGET_PROPERTIES(${TMP_TARGET_NAME} PROPERTIES PROJECT_LABEL "(APP) ${TMP_TARGET_NAME}")

	# Add the required libraries for linking:
	TARGET_LINK_LIBRARIES(${TMP_TARGET_NAME} ${wxWidgets_LIBRARIES} )
	TARGET_LINK_LIBRARIES(${TMP_TARGET_NAME} ${MRPT_LINKER_LIBS})

	TARGET_LINK_LIBRARIES(${TMP_TARGET_NAME} mrpt-reactivenav mrptwxthings)

	# Assure the MRPT library is updated:
	ADD_DEPENDENCIES(${TMP_TARGET_NAME} mrptwxthings)
	
	# Dependencies on MRPT libraries:
	#  Just mention the top-level dependency, the rest will be detected automatically, 
	#  and all the needed #include<> dirs added (see the script DeclareAppDependencies.cmake for further details)
	DeclareAppDependencies(${TMP_TARGET_NAME} mrpt-gui mrpt-reactivenav)

	# ----------------------------------------------------------------------------
	#   				INSTALL
	# ----------------------------------------------------------------------------
	INSTALL(TARGETS ${TMP_TARGET_NAME}
	    RUNTIME DESTINATION ${mrpt_apps_INSTALL_PREFIX}bin
	    LIBRARY DESTINATION ${mrpt_apps_INSTALL_PREFIX}lib${LIB_SUFFIX}
	    ARCHIVE DESTINATION ${mrpt_apps_INSTALL_PREFIX}lib${LIB_SUFFIX})


ELSE(wxWidgets_FOUND)
	# ----------------------------------------------------------------------------------
    	# For convenience. When we cannot continue, inform the user
	# ----------------------------------------------------------------------------------
    MESSAGE(STATUS "wxWidgets not found!: **This target will not be included in the Makefiles**")
ENDIF(wxWidgets_FOUND)

