include(../../cmakemodules/AssureCMakeRootFile.cmake) # Avoid user mistake in CMake source directory

#-----------------------------------------------------------------
# CMake file for the MRPT application:  features_matching
#
#  Run with "cmake ." at the root directory
#
#  Feb 2008, Jose Luis Blanco <jlblanco@ctima.uma.es>
#-----------------------------------------------------------------
project(features-matching)

# ---------------------------------------------
# TARGET:
# ---------------------------------------------
# Define the executable target:
add_executable(${PROJECT_NAME}
	features_matching_main.cpp
	../common/sample_image1.h
	../common/sample_image2.h
	 ${MRPT_VERSION_RC_FILE}
)

# Add the required libraries for linking:
target_link_libraries(${PROJECT_NAME} ${MRPT_LINKER_LIBS} ${OpenCV_LIBRARIES})

# Dependencies on MRPT libraries:
#  Just mention the top-level dependency, the rest will be detected automatically,
#  and all the needed #include<> dirs added (see the script DeclareAppDependencies.cmake for further details)
DeclareAppDependencies(${PROJECT_NAME} mrpt-vision mrpt-gui)

DeclareAppForInstall(${PROJECT_NAME})

AppStartMenuLink(${PROJECT_NAME} "Visual feature matching demo")   # Add link to the Start menu (in Windows Installable packages)
