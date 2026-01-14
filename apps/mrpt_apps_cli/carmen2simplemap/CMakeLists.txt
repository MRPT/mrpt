

project(carmen2simplemap)

# ================================================
# TARGET: carmen2simplemap
# ================================================
# Define the executable target:
add_executable(${PROJECT_NAME}
        carmen2simplemap_main.cpp
		 ${MRPT_VERSION_RC_FILE}
        )

# Add the required libraries for linking:

DeclareAppForInstall(${PROJECT_NAME})
# Dependencies on MRPT libraries: Just mention the top-level dependency, the rest will be detected automatically,
#  and all the needed #include<> dirs added (see the script DeclareAppDependencies.cmake for further details)
DeclareAppDependencies(${PROJECT_NAME} mrpt::obs mrpt::tclap)
