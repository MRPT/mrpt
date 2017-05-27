include(../../cmakemodules/AssureCMakeRootFile.cmake) # Avoid user mistake in CMake source directory

#-----------------------------------------------------------------
# CMake file for the MRPT application:  robot-map-gui
#
#  Run with "cmake ." at the root directory
#
#  May 2017, LisGein <alred402@gmail.com>
#-----------------------------------------------------------------
project(robot-map-gui)


if(CMAKE_MRPT_HAS_Qt5)

    set(CPP_FILES
        robot-map-gui_main.cpp
        gui/cmainwindow.cpp
        )


    set(FILES_TO_MOC
        gui/cmainwindow.h
        )


    qt5_wrap_cpp(moc_cpps ${FILES_TO_MOC})

    # ---------------------------------------------
    # TARGET:
    # ---------------------------------------------
    # Define the executable target:

    add_executable(${PROJECT_NAME}
        ${CPP_FILES}
        ${moc_cpps}
        ${MRPT_VERSION_RC_FILE})

    # Add the required libraries for linking:
    target_link_libraries(${PROJECT_NAME}
        Qt5::Widgets
        ${MRPT_LINKER_LIBS}
        )

    # Dependencies on MRPT libraries:
    #  Just mention the top-level dependency, the rest will be detected automatically,
    #  and all the needed #include<> dirs added (see the script DeclareAppDependencies.cmake for further details)
    DeclareAppDependencies(${PROJECT_NAME} mrpt-slam  )

    DeclareAppForInstall(${PROJECT_NAME})

endif(CMAKE_MRPT_HAS_Qt5)
