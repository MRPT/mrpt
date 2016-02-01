INCLUDE(../../cmakemodules/AssureCMakeRootFile.cmake) # Avoid user mistake in CMake source directory

#-----------------------------------------------------------------
# CMake file for the MRPT application:  RawLogViewer
#
#  Run with "cmake ." at the root directory
#
#  October 2007, Jose Luis Blanco <jlblanco@ctima.uma.es>
#-----------------------------------------------------------------
PROJECT(camera_calib)

IF(CMAKE_MRPT_HAS_WXWIDGETS)

	# ---------------------------------------------
	# TARGET:
	# ---------------------------------------------
	SET(SRCS 
		camera_calib_guiApp.cpp   camera_calib_guiApp.h
		camera_calib_guiMain.cpp  camera_calib_guiMain.h
		CAboutBox.h  CAboutBox.cpp
		CDlgCalibWizardOnline.cpp  CDlgCalibWizardOnline.h
		MyGLCanvas.cpp MyGLCanvas.h
	 	../wx-common/CMyRedirector.h
	 )
	 
	IF(WIN32)
		SET(SRCS ${SRCS} resource.rc)
	ENDIF(WIN32)

	# Define the executable target:
	ADD_EXECUTABLE(camera-calib WIN32 ${SRCS} ${MRPT_VERSION_RC_FILE})

	SET(TMP_TARGET_NAME "camera-calib")


	

	# Add the required libraries for linking:
	INCLUDE_DIRECTORIES( ${wxWidgets_INCLUDE_DIRS} )
	TARGET_LINK_LIBRARIES(${TMP_TARGET_NAME} ${wxWidgets_LIBRARIES} )
	TARGET_LINK_LIBRARIES(${TMP_TARGET_NAME} ${MRPT_LINKER_LIBS})

	# Dependencies on MRPT libraries:
	#  Just mention the top-level dependency, the rest will be detected automatically, 
	#  and all the needed #include<> dirs added (see the script DeclareAppDependencies.cmake for further details)
	DeclareAppDependencies(${TMP_TARGET_NAME} mrpt-vision mrpt-gui mrpt-hwdrivers)
		
	DeclareAppForInstall(${TMP_TARGET_NAME})

	AppStartMenuLink(${TMP_TARGET_NAME} "Camera calibration GUI")   # Add link to the Start menu (in Windows Installable packages)
		
ELSE()
	# ----------------------------------------------------------------------------------
    # For convenience. When we cannot continue, inform the user
	# ----------------------------------------------------------------------------------
    MESSAGE(STATUS "wxWidgets not found!: **This target will not be included in the Makefiles**")
ENDIF()


