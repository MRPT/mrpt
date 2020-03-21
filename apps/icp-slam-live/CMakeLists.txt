
project(icp-slam-live)

add_executable(${PROJECT_NAME}
               icp-slam-live_main.cpp
			    ${MRPT_VERSION_RC_FILE})

# Add the required libraries for linking:


# Dependencies on MRPT libraries:
#  Just mention the top-level dependency, the rest will be detected automatically,
#  and all the needed #include<> dirs added (see the script DeclareAppDependencies.cmake for further details)
DeclareAppDependencies(${PROJECT_NAME} mrpt::apps)

DeclareAppForInstall(${PROJECT_NAME})
