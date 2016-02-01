INCLUDE(../../cmakemodules/AssureCMakeRootFile.cmake) # Avoid user mistake in CMake source directory

#-----------------------------------------------------------------
# CMake file for the MRPT application:  kinect-calibrate
#
#  Run with "cmake ." at the root directory
#
#  March 2012, Jose Luis Blanco <joseluisblancoc@gmail.com>
#-----------------------------------------------------------------
PROJECT(kinect_calibrate)

IF(CMAKE_MRPT_HAS_WXWIDGETS)
	# ---------------------------------------------
	# TARGET:
	# ---------------------------------------------
	SET(SRCS
		kinect_calibrate_guiApp.cpp kinect_calibrate_guiApp.h
		kinect_calibrate_guiMain.cpp kinect_calibrate_guiMain.h
		CAboutBox.cpp  CAboutBox.h
		../wx-common/CMyRedirector.h
		MyGLCanvas.cpp MyGLCanvas.h
		)

	IF(WIN32)
		SET(SRCS ${SRCS} resource.rc)
	ENDIF(WIN32)

	# Define the executable target:
	ADD_EXECUTABLE(kinect-stereo-calib WIN32 ${SRCS} ${MRPT_VERSION_RC_FILE})

	SET(TMP_TARGET_NAME "kinect-stereo-calib")


	

	# Add the required libraries for linking:
	INCLUDE_DIRECTORIES( ${wxWidgets_INCLUDE_DIRS} )
	TARGET_LINK_LIBRARIES(${TMP_TARGET_NAME} ${wxWidgets_LIBRARIES} )
	TARGET_LINK_LIBRARIES(${TMP_TARGET_NAME} ${MRPT_LINKER_LIBS})

	# Dependencies on MRPT libraries:
	#  Just mention the top-level dependency, the rest will be detected automatically,
	#  and all the needed #include<> dirs added (see the script DeclareAppDependencies.cmake for further details)
	DeclareAppDependencies(${TMP_TARGET_NAME} mrpt-hwdrivers mrpt-gui mrpt-vision)

	DeclareAppForInstall(${TMP_TARGET_NAME})

	AppStartMenuLink(${TMP_TARGET_NAME} "Kinect (and generic stereo cams) calibration tool")   # Add link to the Start menu (in Windows Installable packages)

ELSE()
	# ----------------------------------------------------------------------------------
	# For convenience. When we cannot continue, inform the user
	# ----------------------------------------------------------------------------------
	MESSAGE(STATUS "wxWidgets not found!: **This target will not be included in the Makefiles**")
ENDIF()


