INCLUDE(../../cmakemodules/AssureCMakeRootFile.cmake) # Avoid user mistake in CMake source directory

#-----------------------------------------------------------------
# CMake file for the MRPT application:  mrpt-performance
#
#  Run with "cmake ." at the root directory
#
#  October 2007, Jose Luis Blanco <jlblanco@ctima.uma.es>
#-----------------------------------------------------------------
PROJECT(mrpt-performance)


# ---------------------------------------------
# TARGET:
# ---------------------------------------------

ADD_DEFINITIONS(-DMRPT_DATASET_DIR="${MRPT_SOURCE_DIR}/share/mrpt/datasets")
ADD_DEFINITIONS(-DMRPT_DOC_PERF_DIR="${MRPT_SOURCE_DIR}/doc/perf-data")

# Define the executable target:
ADD_EXECUTABLE(${PROJECT_NAME}
	perf-main.cpp
	common.h
	run_build_tables.h
	# Test files:
	perf-feature_extraction.cpp
	perf-feature_matching.cpp
	perf-graph.cpp
	perf-graphslam.cpp
	perf-gridmaps.cpp
	perf-icp.cpp
	perf-images.cpp
	perf-math.cpp
	perf-matrix1.cpp perf-matrix2.cpp
	perf-pointmaps.cpp
	perf-poses.cpp
	perf-pose-interp.cpp
	perf-random.cpp
	perf-scan_matching.cpp
	perf-CObservation3DRangeScan.cpp
	perf-atan2lut.cpp
	perf-strings.cpp
	${MRPT_VERSION_RC_FILE}
	)

# Add the required libraries for linking:
TARGET_LINK_LIBRARIES(${PROJECT_NAME} ${MRPT_LINKER_LIBS})

# Dependencies on MRPT libraries:
#  Just mention the top-level dependency, the rest will be detected automatically,
#  and all the needed #include<> dirs added (see the script DeclareAppDependencies.cmake for further details)
DeclareAppDependencies(${PROJECT_NAME} mrpt-slam mrpt-gui mrpt-tfest mrpt-graphs mrpt-graphslam)


DeclareAppForInstall(${PROJECT_NAME})
