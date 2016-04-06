INCLUDE(../../cmakemodules/AssureCMakeRootFile.cmake) # Avoid user mistake in CMake source directory

PROJECT(image2gridmap)

# Define the executable target:
ADD_EXECUTABLE(image2gridmap
               image2gridmap.cpp
		${MRPT_VERSION_RC_FILE})

SET(TMP_TARGET_NAME "image2gridmap")

# Add the required libraries for linking:
TARGET_LINK_LIBRARIES(${TMP_TARGET_NAME} ${MRPT_LINKER_LIBS})

# Dependencies on MRPT libraries:
#  Just mention the top-level dependency, the rest will be detected automatically, 
#  and all the needed #include<> dirs added (see the script DeclareAppDependencies.cmake for further details)
DeclareAppDependencies(${TMP_TARGET_NAME} mrpt-maps )

DeclareAppForInstall(${TMP_TARGET_NAME})
