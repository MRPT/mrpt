include(../../cmakemodules/AssureCMakeRootFile.cmake) # Avoid user mistake in CMake source directory
project(icp-slam-live)

add_executable(${PROJECT_NAME}
               icp-slam-live_main.cpp
			    ${MRPT_VERSION_RC_FILE})

# Add the required libraries for linking:
target_link_libraries(${PROJECT_NAME} ${MRPT_LINKER_LIBS})

# Dependencies on MRPT libraries:
#  Just mention the top-level dependency, the rest will be detected automatically,
#  and all the needed #include<> dirs added (see the script DeclareAppDependencies.cmake for further details)
DeclareAppDependencies(${PROJECT_NAME} mrpt-gui mrpt-slam mrpt-hwdrivers)

DeclareAppForInstall(${PROJECT_NAME})
