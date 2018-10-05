include(../../cmakemodules/AssureCMakeRootFile.cmake)

project(benchmarkingImageFeatures_GUI)



if(CMAKE_MRPT_HAS_Qt5 AND CMAKE_MRPT_HAS_OPENCV AND NOT "${MRPT_OPENCV_VERSION}" VERSION_LESS "3.2")
        set(CPP_FILES src/main.cpp
            src/mainwindow.cpp
            src/my_qlabel.cpp
            src/visual_odometry.h
            src/visual_odometry.cpp
            src/tracker.cpp src/tracker.h
            src/place_recognition.cpp src/place_recognition.h)


        set(FILES_TO_MOC
                src/mainwindow.h
                src/my_qlabel.h
                src/visual_odometry.h
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

endif()
