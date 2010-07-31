INCLUDE(../../cmakemodules/AssureCMakeRootFile.cmake) # Avoid user mistake in CMake source directory

#-----------------------------------------------------------------
# CMake file for the MRPT application:  mrpt-performance
#
#  Run with "cmake ." at the root directory
#
#  October 2007, Jose Luis Blanco <jlblanco@ctima.uma.es>
#-----------------------------------------------------------------
PROJECT(mrpt_performance)


# ---------------------------------------------
# TARGET:
# ---------------------------------------------

ADD_DEFINITIONS(-DMRPT_DATASET_DIR="${MRPT_SOURCE_DIR}/share/mrpt/datasets")

# Define the executable target:
ADD_EXECUTABLE(mrpt-performance
	perf-main.cpp
	perf-matrix.h
	perf-gridmaps.h
	perf-random.h
	perf-math.h
	perf-icp.h
	perf-pointmaps.h
	perf-images.h
	perf-scan_matching.h
	perf-feature_extraction.h
	perf-feature_matching.h
	)

SET(TMP_TARGET_NAME "mrpt-performance")

SET_TARGET_PROPERTIES(${TMP_TARGET_NAME} PROPERTIES PROJECT_LABEL "(APP) ${TMP_TARGET_NAME}")

# Add the required libraries for linking:
TARGET_LINK_LIBRARIES(${TMP_TARGET_NAME} ${MRPT_LINKER_LIBS})

# Dependencies on MRPT libraries:
#  Just mention the top-level dependency, the rest will be detected automatically, 
#  and all the needed #include<> dirs added (see the script DeclareAppDependencies.cmake for further details)
DeclareAppDependencies(${TMP_TARGET_NAME} mrpt-slam mrpt-gui mrpt-scanmatching)


# ----------------------------------------------------------------------------
#   				INSTALL
# ----------------------------------------------------------------------------
INSTALL(TARGETS mrpt-performance
	RUNTIME DESTINATION ${mrpt_apps_INSTALL_PREFIX}bin
	LIBRARY DESTINATION ${mrpt_apps_INSTALL_PREFIX}lib${LIB_SUFFIX}
	ARCHIVE DESTINATION ${mrpt_apps_INSTALL_PREFIX}lib${LIB_SUFFIX})

