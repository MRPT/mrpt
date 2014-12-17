INCLUDE(../../cmakemodules/AssureCMakeRootFile.cmake) # Avoid user mistake in CMake source directory

#-----------------------------------------------------------------
# CMake file for the MRPT application:  features_matching
#
#  Run with "cmake ." at the root directory
#
#  Feb 2008, Jose Luis Blanco <jlblanco@ctima.uma.es>
#-----------------------------------------------------------------
PROJECT(features_matching)

# ---------------------------------------------
# TARGET:
# ---------------------------------------------
# Define the executable target:
ADD_EXECUTABLE(features-matching
	features_matching_main.cpp
	../common/sample_image1.h
	../common/sample_image2.h
	 ${MRPT_VERSION_RC_FILE}
)

SET(TMP_TARGET_NAME "features-matching")



# Add the required libraries for linking:
TARGET_LINK_LIBRARIES(${TMP_TARGET_NAME} ${MRPT_LINKER_LIBS} ${OpenCV_LIBRARIES})

# Dependencies on MRPT libraries:
#  Just mention the top-level dependency, the rest will be detected automatically, 
#  and all the needed #include<> dirs added (see the script DeclareAppDependencies.cmake for further details)
DeclareAppDependencies(${TMP_TARGET_NAME} mrpt-vision mrpt-gui)

DeclareAppForInstall(${TMP_TARGET_NAME})

AppStartMenuLink(${TMP_TARGET_NAME} "Visual feature matching demo")   # Add link to the Start menu (in Windows Installable packages)
