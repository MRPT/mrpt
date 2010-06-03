INCLUDE(../../cmakemodules/AssureCMakeRootFile.cmake) # Avoid user mistake in CMake source directory

#-----------------------------------------------------------------
# CMake file for the MRPT application:  vOdometry
#
#  Run with "cmake ." at the root directory
#
#  October 2007, Jose Luis Blanco <jlblanco@ctima.uma.es>
#-----------------------------------------------------------------
PROJECT(vOdometry)

#MESSAGE(STATUS "Makefile for application: /apps/vOdometry")

# ---------------------------------------------
# TARGET:
# ---------------------------------------------
# Define the executable target:
ADD_EXECUTABLE(vOdometry Main.cpp)

SET(TMP_TARGET_NAME "vOdometry")

SET_TARGET_PROPERTIES(${TMP_TARGET_NAME} PROPERTIES PROJECT_LABEL "(APP) ${TMP_TARGET_NAME}")

# Add the required libraries for linking:
TARGET_LINK_LIBRARIES(${TMP_TARGET_NAME} ${MRPT_LINKER_LIBS})

# Dependencies on MRPT libraries:
#  Just mention the top-level dependency, the rest will be detected automatically, 
#  and all the needed #include<> dirs added (see the script DeclareAppDependencies.cmake for further details)
DeclareAppDependencies(${TMP_TARGET_NAME} mrpt-vision mrpt-stereoslam)

IF(CMAKE_COMPILER_IS_GNUCXX)
	TARGET_LINK_LIBRARIES(${TMP_TARGET_NAME} mrpt-stereoslam${MRPT_LINKER_LIBS_POSTFIX})
ENDIF(CMAKE_COMPILER_IS_GNUCXX)


# ----------------------------------------------------------------------------
#   				INSTALL
# ----------------------------------------------------------------------------
INSTALL(TARGETS vOdometry
	RUNTIME DESTINATION ${mrpt_apps_INSTALL_PREFIX}bin
	LIBRARY DESTINATION ${mrpt_apps_INSTALL_PREFIX}lib${LIB_SUFFIX}
	ARCHIVE DESTINATION ${mrpt_apps_INSTALL_PREFIX}lib${LIB_SUFFIX})

