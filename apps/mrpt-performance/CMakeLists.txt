include(../../cmakemodules/AssureCMakeRootFile.cmake) # Avoid user mistake in CMake source directory

#-----------------------------------------------------------------
# CMake file for the MRPT application:  mrpt-performance
#
#  Run with "cmake ." at the root directory
#
#  October 2007, Jose Luis Blanco <jlblanco@ctima.uma.es>
#-----------------------------------------------------------------
project(mrpt-performance)


# ---------------------------------------------
# TARGET:
# ---------------------------------------------

add_definitions(-DMRPT_DATASET_DIR="${MRPT_SOURCE_DIR}/share/mrpt/datasets")
add_definitions(-DMRPT_DOC_PERF_DIR=".")

# Define the executable target:
add_executable(${PROJECT_NAME}
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
target_link_libraries(${PROJECT_NAME} ${MRPT_LINKER_LIBS})

# Dependencies on MRPT libraries:
#  Just mention the top-level dependency, the rest will be detected automatically,
#  and all the needed #include<> dirs added (see the script DeclareAppDependencies.cmake for further details)
DeclareAppDependencies(${PROJECT_NAME} mrpt-slam mrpt-gui mrpt-tfest mrpt-graphs mrpt-graphslam mrpt-img mrpt-tclap)


DeclareAppForInstall(${PROJECT_NAME})
