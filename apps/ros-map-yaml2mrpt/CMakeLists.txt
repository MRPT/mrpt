
project(ros-map-yaml2mrpt)

# Define the executable target:
add_executable(${PROJECT_NAME}
               ros-map-yaml2mrpt.cpp
		${MRPT_VERSION_RC_FILE})

# Add the required libraries for linking:

# Dependencies on MRPT libraries:
#  Just mention the top-level dependency, the rest will be detected automatically,
#  and all the needed #include<> dirs added (see the script DeclareAppDependencies.cmake for further details)
DeclareAppDependencies(${PROJECT_NAME} mrpt::maps mrpt::tclap)

DeclareAppForInstall(${PROJECT_NAME})
