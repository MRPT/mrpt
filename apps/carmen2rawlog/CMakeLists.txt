INCLUDE(../../cmakemodules/AssureCMakeRootFile.cmake) # Avoid user mistake in CMake source directory

PROJECT(carmen2rawlog)

# ================================================
# TARGET: carmen2rawlog
# ================================================
# Define the executable target:
ADD_EXECUTABLE(carmen2rawlog
	carmen2rawlog_main.cpp
	 ${MRPT_VERSION_RC_FILE}
	)
SET(TMP_TARGET_NAME "carmen2rawlog")

# Add the required libraries for linking:
TARGET_LINK_LIBRARIES(${TMP_TARGET_NAME} ${MRPT_LINKER_LIBS})
DeclareAppForInstall(${TMP_TARGET_NAME})
# Dependencies on MRPT libraries: Just mention the top-level dependency, the rest will be detected automatically,
#  and all the needed #include<> dirs added (see the script DeclareAppDependencies.cmake for further details)
DeclareAppDependencies(${TMP_TARGET_NAME} mrpt-obs)

