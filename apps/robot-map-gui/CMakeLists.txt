include(../../cmakemodules/AssureCMakeRootFile.cmake) # Avoid user mistake in CMake source directory

#-----------------------------------------------------------------
# CMake file for the MRPT application:  robot-map-gui
#
#  Run with "cmake ." at the root directory
#
#  May 2017, LisGein <alred402@gmail.com>
#-----------------------------------------------------------------
project(robot-map-gui)

set(CMAKE_INCLUDE_CURRENT_DIR ON)

if(CMAKE_MRPT_HAS_Qt5 AND CMAKE_MRPT_HAS_OPENGL_GLUT)

	set(CPP_FILES
		robot-map-gui_main.cpp
		CDocument.cpp

		gui/CMainWindow.cpp
		gui/CGLWidget.cpp
		gui/observationTree/CObservationTreeModel.cpp
		gui/observationTree/CNode.cpp
		gui/observationTree/CRootNode.cpp
		gui/observationTree/CPosesNode.cpp
		gui/observationTree/CPairNode.cpp
		gui/observationTree/CObservationsNode.cpp
		gui/observationTree/CBaseObservationNode.cpp
		gui/observationTree/CRangeScanNode.cpp
		gui/configWidget/CConfigWidget.cpp
		gui/configWidget/CSelectType.cpp
		gui/configWidget/CBaseConfig.cpp
		gui/configWidget/CPointsConfig.cpp
		gui/configWidget/CBeaconConfig.cpp
		gui/configWidget/COccupancyConfig.cpp
		gui/configWidget/CGasGridConfig.cpp
		gui/configWidget/CLandmarksConfig.cpp
		TypeOfConfig.h
		)


	set(FILES_TO_MOC
		gui/CMainWindow.h
		gui/CGLWidget.h
		gui/configWidget/CConfigWidget.h
		)

	set(FILES_TO_UIC
		gui/CMainWindow.ui
		gui/configWidget/CConfigWidget.ui
		gui/configWidget/CSelectType.ui
		gui/configWidget/CPointsConfig.ui
		gui/configWidget/COccupancyConfig.ui
		gui/configWidget/CBeaconConfig.ui
		gui/configWidget/CGasGridConfig.ui
		gui/configWidget/CLandmarksConfig.ui
		)
	qt5_wrap_cpp(moc_cpps ${FILES_TO_MOC})
	qt5_wrap_ui(uic_cpps ${FILES_TO_UIC})

	# ---------------------------------------------
	# TARGET:
	# ---------------------------------------------
	# Define the executable target:
	add_executable(${PROJECT_NAME}
		${CPP_FILES}
		${moc_cpps}
		${uic_cpps}
		${MRPT_VERSION_RC_FILE}
		)

	# Add the required libraries for linking:
	qt5_use_modules(${PROJECT_NAME} Widgets OpenGL)

	target_link_libraries(${PROJECT_NAME} ${MRPT_LINKER_LIBS})

	# Dependencies on MRPT libraries:
	#  Just mention the top-level dependency, the rest will be detected automatically,
	#  and all the needed #include<> dirs added (see the script DeclareAppDependencies.cmake for further details)
	DeclareAppDependencies(${PROJECT_NAME} mrpt-slam mrpt-gui mrpt-opengl mrpt-vision)

	DeclareAppForInstall(${PROJECT_NAME})

endif(CMAKE_MRPT_HAS_Qt5 AND CMAKE_MRPT_HAS_OPENGL_GLUT)
