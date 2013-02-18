INCLUDE(../../cmakemodules/AssureCMakeRootFile.cmake) # Avoid user mistake in CMake source directory

#-----------------------------------------------------------------
# CMake file for the MRPT application:  kinect-3d-view
#
#  Run with "cmake ." at the root directory
#-----------------------------------------------------------------
PROJECT(kinect_3d_view)

# ---------------------------------------------
# TARGET:
# ---------------------------------------------
# Define the executable target:
ADD_EXECUTABLE(kinect-3d-view
               kinect-3d-view_main.cpp)

SET(TMP_TARGET_NAME "kinect-3d-view")



# Add the required libraries for linking:
TARGET_LINK_LIBRARIES(${TMP_TARGET_NAME} ${MRPT_LINKER_LIBS})

# Dependencies on MRPT libraries:
#  Just mention the top-level dependency, the rest will be detected automatically, 
#  and all the needed #include<> dirs added (see the script DeclareAppDependencies.cmake for further details)
DeclareAppDependencies(${TMP_TARGET_NAME} mrpt-gui mrpt-hwdrivers mrpt-maps)

DeclareAppForInstall(${TMP_TARGET_NAME})

