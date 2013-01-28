INCLUDE(../../cmakemodules/AssureCMakeRootFile.cmake) # Avoid user mistake in CMake source directory

#-----------------------------------------------------------------
# CMake file for the MRPT application:  srba-slam
#  Run with "cmake ." at the root directory
#  Jan 2013, Jose Luis Blanco
#-----------------------------------------------------------------
PROJECT(srba_slam)

# ---------------------------------------------
# TARGET:
# ---------------------------------------------
# Define the executable target:
ADD_EXECUTABLE(srba-slam
	srba-slam_main.cpp
	CDatasetParserBase.h
	CDatasetParser_RangeBearing2D.h
	CDatasetParser_Stereo.h
	)

SET(TMP_TARGET_NAME "srba-slam")

SET_TARGET_PROPERTIES(${TMP_TARGET_NAME} PROPERTIES PROJECT_LABEL "(APP) ${TMP_TARGET_NAME}")

# Add the required libraries for linking:
TARGET_LINK_LIBRARIES(${TMP_TARGET_NAME} ${MRPT_LINKER_LIBS})

# Dependencies on MRPT libraries:
#  Just mention the top-level dependency, the rest will be detected automatically,
#  and all the needed #include<> dirs added (see the script DeclareAppDependencies.cmake for further details)
DeclareAppDependencies(${TMP_TARGET_NAME} mrpt-srba mrpt-gui)

DeclareAppForInstall(${TMP_TARGET_NAME})
