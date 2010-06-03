INCLUDE(../../cmakemodules/AssureCMakeRootFile.cmake) # Avoid user mistake in CMake source directory

#-----------------------------------------------------------------
# CMake file for the MRPT application:  ReactiveNavigationDemo
#
#  Run with "cmake ." at the root directory
#
#  October 2007, Jose Luis Blanco <jlblanco@ctima.uma.es>
#-----------------------------------------------------------------

PROJECT(ReactiveNavigationDemo)

IF(wxWidgets_FOUND)

	# ---------------------------------------------
	# TARGET:
	# ---------------------------------------------
	SET(SRCS
	 ReactiveNavigationDemoApp.cpp  ReactiveNavigationDemoApp.h
	 ReactiveNavigationDemoMain.cpp  ReactiveNavigationDemoMain.h
	 CIniEditor.cpp  CIniEditor.h
	 DEFAULT_GRIDMAP_DATA.h
	 ../wx-common/CMyRedirector.h
	 )

	IF(WIN32)
		SET(SRCS ${SRCS} resource.rc)
	ENDIF(WIN32)

	# Define the executable target:
	ADD_EXECUTABLE(ReactiveNavigationDemo WIN32 ${SRCS})

	SET(TMP_TARGET_NAME "ReactiveNavigationDemo")


	SET_TARGET_PROPERTIES(${TMP_TARGET_NAME} PROPERTIES PROJECT_LABEL "(APP) ${TMP_TARGET_NAME}")

	# Add the required libraries for linking:
	TARGET_LINK_LIBRARIES(${TMP_TARGET_NAME} ${wxWidgets_LIBRARIES} )
	TARGET_LINK_LIBRARIES(${TMP_TARGET_NAME} ${MRPT_LINKER_LIBS})

	TARGET_LINK_LIBRARIES(${TMP_TARGET_NAME} mrpt-reactivenav)

	# Dependencies on MRPT libraries:
	#  Just mention the top-level dependency, the rest will be detected automatically, 
	#  and all the needed #include<> dirs added (see the script DeclareAppDependencies.cmake for further details)
	DeclareAppDependencies(${TMP_TARGET_NAME} mrpt-gui mrpt-reactivenav)

	# ----------------------------------------------------------------------------
	#   				INSTALL
	# ----------------------------------------------------------------------------
	INSTALL(TARGETS ReactiveNavigationDemo
	    RUNTIME DESTINATION ${mrpt_apps_INSTALL_PREFIX}bin
	    LIBRARY DESTINATION ${mrpt_apps_INSTALL_PREFIX}lib${LIB_SUFFIX}
	    ARCHIVE DESTINATION ${mrpt_apps_INSTALL_PREFIX}lib${LIB_SUFFIX})


ELSE(wxWidgets_FOUND)
	# ----------------------------------------------------------------------------------
    # For convenience. When we cannot continue, inform the user
	# ----------------------------------------------------------------------------------
    MESSAGE(STATUS "wxWidgets not found!: **This target will not be included in the Makefiles**")
ENDIF(wxWidgets_FOUND)


