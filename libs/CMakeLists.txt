project(ALL_MRPT_LIBS)

# ------------------------------------------
# List of MRPT "modules" or "libraries":
# ------------------------------------------
add_subdirectory(base)
add_subdirectory(scanmatching)
add_subdirectory(graphs)
add_subdirectory(bayes)
add_subdirectory(opengl)
add_subdirectory(gui)
add_subdirectory(topography)
add_subdirectory(obs)
add_subdirectory(hwdrivers)
add_subdirectory(maps)
add_subdirectory(vision)
add_subdirectory(slam)
add_subdirectory(graphslam)
add_subdirectory(reactivenav)
add_subdirectory(hmtslam)
add_subdirectory(detectors)

