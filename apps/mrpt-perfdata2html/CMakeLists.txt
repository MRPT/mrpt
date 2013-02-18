#-----------------------------------------------------------------
# CMake file for the MRPT application:  mrpt-perfdata2html
#
#  Run with "cmake ." at the root directory
#
#  October 2007, Jose Luis Blanco <jlblanco@ctima.uma.es>
#-----------------------------------------------------------------
PROJECT(mrpt_perfdata2html)

# ---------------------------------------------
# TARGET:
# ---------------------------------------------
# Define the executable target:
ADD_EXECUTABLE(mrpt-perfdata2html
	mrpt-perfdata2html-main.cpp
	)

SET(TMP_TARGET_NAME "mrpt-perfdata2html")



# Add the required libraries for linking:
TARGET_LINK_LIBRARIES(${TMP_TARGET_NAME} ${MRPT_LINKER_LIBS})

# Dependencies on MRPT libraries:
#  Just mention the top-level dependency, the rest will be detected automatically,
#  and all the needed #include<> dirs added (see the script DeclareAppDependencies.cmake for further details)
DeclareAppDependencies(${TMP_TARGET_NAME} mrpt-base)

DeclareAppForInstall(${TMP_TARGET_NAME})
