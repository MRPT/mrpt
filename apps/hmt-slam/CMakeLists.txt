INCLUDE(../../cmakemodules/AssureCMakeRootFile.cmake) # Avoid user mistake in CMake source directory

#-----------------------------------------------------------------
# CMake file for the MRPT application:  HMT-SLAM
#
#  Run with "cmake ." at the root directory
#
#  December 2007, Jose Luis Blanco <jlblanco@ctima.uma.es>
#-----------------------------------------------------------------
PROJECT(hmt_slam)

#MESSAGE(STATUS "Makefile for application: /apps/hmt-slam ")

# ---------------------------------------------
# TARGET:
# ---------------------------------------------
# Define the executable target:
ADD_EXECUTABLE(hmt-slam
               hmt-slam_main.cpp)

SET(TMP_TARGET_NAME "hmt-slam")

SET_TARGET_PROPERTIES(${TMP_TARGET_NAME} PROPERTIES PROJECT_LABEL "(APP) ${TMP_TARGET_NAME}")

# Add the required libraries for linking:
TARGET_LINK_LIBRARIES(${TMP_TARGET_NAME} ${MRPT_LINKER_LIBS_RELorDEB} ) 


# Dependencies on MRPT libraries:
#  Just mention the top-level dependency, the rest will be detected automatically, 
#  and all the needed #include<> dirs added (see the script DeclareAppDependencies.cmake for further details)
DeclareAppDependencies(${TMP_TARGET_NAME} mrpt-hmtslam)


# ----------------------------------------------------
#   FabMap module for HMT-SLAM? (optional)
# ----------------------------------------------------
IF(FabMap_FOUND)
	TARGET_LINK_LIBRARIES(${TMP_TARGET_NAME} ${FabMap_LIBRARIES})
ENDIF(FabMap_FOUND)


# ----------------------------------------------------------------------------
#   				INSTALL
# ----------------------------------------------------------------------------
INSTALL(TARGETS hmt-slam
		RUNTIME DESTINATION ${mrpt_apps_INSTALL_PREFIX}bin
		LIBRARY DESTINATION ${mrpt_apps_INSTALL_PREFIX}lib${LIB_SUFFIX}
		ARCHIVE DESTINATION ${mrpt_apps_INSTALL_PREFIX}lib${LIB_SUFFIX})

