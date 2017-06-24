INCLUDE(../../cmakemodules/AssureCMakeRootFile.cmake)
INCLUDE(../../cmakemodules/script_qt.cmake)

project(benchmarkingImageFeatures_GUI)



if(CMAKE_MRPT_HAS_Qt5)

        set(CPP_FILES src/main.cpp
        src/mainwindow.cpp
	src/visualizedialog.cpp 
	)


        set(FILES_TO_MOC
                src/mainwindow.h
                src/visualizedialog.h
                )

        qt5_wrap_cpp(moc_cpps ${FILES_TO_MOC})

        # ---------------------------------------------
        # TARGET:
        # ---------------------------------------------
        # Define the executable target:
        add_executable(${PROJECT_NAME}
                ${CPP_FILES}
                ${moc_cpps}
                ${MRPT_VERSION_RC_FILE}
                )

        # Add the required libraries for linking:
        qt5_use_modules(${PROJECT_NAME} Widgets OpenGL)

        target_link_libraries(${PROJECT_NAME} ${MRPT_LINKER_LIBS} ${OpenCV_LIBS})

        # Dependencies on MRPT libraries:
        #  Just mention the top-level dependency, the rest will be detected automatically,
        #  and all the needed #include<> dirs added (see the script DeclareAppDependencies.cmake for further details)
        DeclareAppDependencies(${PROJECT_NAME} mrpt-gui mrpt-vision)
        DeclareAppForInstall(${PROJECT_NAME})

endif(CMAKE_MRPT_HAS_Qt5)
