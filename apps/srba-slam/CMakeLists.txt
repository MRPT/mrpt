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
FILE(GLOB LST_SRCS "*.cpp")
FILE(GLOB LST_HDRS "*.h")


ADD_EXECUTABLE(srba-slam
	${LST_SRCS}
	${LST_HDRS}
	)

SET(TMP_TARGET_NAME "srba-slam")



# Add the required libraries for linking:
TARGET_LINK_LIBRARIES(${TMP_TARGET_NAME} ${MRPT_LINKER_LIBS})

# Dependencies on MRPT libraries:
#  Just mention the top-level dependency, the rest will be detected automatically,
#  and all the needed #include<> dirs added (see the script DeclareAppDependencies.cmake for further details)
DeclareAppDependencies(${TMP_TARGET_NAME} mrpt-srba mrpt-gui mrpt-vision)

DeclareAppForInstall(${TMP_TARGET_NAME})
