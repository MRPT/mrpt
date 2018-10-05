include(../../cmakemodules/AssureCMakeRootFile.cmake) # Avoid user mistake in CMake source directory

#-----------------------------------------------------------------
# CMake file for the MRPT application:  HMT-SLAM
#
#  Run with "cmake ." at the root directory
#
#  December 2007, Jose Luis Blanco <jlblanco@ctima.uma.es>
#-----------------------------------------------------------------
project(hmt-slam)



# ---------------------------------------------
# TARGET:
# ---------------------------------------------
# Define the executable target:
add_executable(${PROJECT_NAME}
               hmt-slam_main.cpp
			    ${MRPT_VERSION_RC_FILE})

# Add the required libraries for linking:
target_link_libraries(${PROJECT_NAME} ${MRPT_LINKER_LIBS_RELorDEB} )


# Dependencies on MRPT libraries:
#  Just mention the top-level dependency, the rest will be detected automatically,
#  and all the needed #include<> dirs added (see the script DeclareAppDependencies.cmake for further details)
DeclareAppDependencies(${PROJECT_NAME} mrpt-hmtslam mrpt-slam mrpt-bayes)


# ----------------------------------------------------
#   FabMap module for HMT-SLAM? (optional)
# ----------------------------------------------------
if(FabMap_FOUND)
	target_link_libraries(${PROJECT_NAME} ${FabMap_LIBRARIES})
endif()


DeclareAppForInstall(${PROJECT_NAME})
