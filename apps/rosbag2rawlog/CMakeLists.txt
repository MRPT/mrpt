INCLUDE(../../cmakemodules/AssureCMakeRootFile.cmake) # Avoid user mistake in CMake source directory

if(CMAKE_MRPT_HAS_YAMLCPP AND
	roscpp_FOUND AND
	rosbag_FOUND AND
	cv_bridge_FOUND AND
	sensor_msgs_FOUND AND
	tf2_msgs_FOUND AND
	tf2_FOUND)

PROJECT(rosbag2rawlog)

# ================================================
# TARGET: gps2rawlog
# ================================================
# Define the executable target:
ADD_EXECUTABLE(${PROJECT_NAME}
	# Main:
	rosbag2rawlog_main.cpp
	${MRPT_VERSION_RC_FILE}
 	)

# Add the required libraries for linking:
TARGET_LINK_LIBRARIES(${PROJECT_NAME} ${MRPT_LINKER_LIBS} ${rosbag_LIBRARIES} ${cv_bridge_LIBRARIES} ${tf2_LIBRARIES} ${LIBYAMLCPP_LIBRARIES})
DeclareAppForInstall(${PROJECT_NAME})
# Dependencies on MRPT libraries: Just mention the top-level dependency, the rest will be detected automatically,
#  and all the needed #include<> dirs added (see the script DeclareAppDependencies.cmake for further details)
DeclareAppDependencies(${PROJECT_NAME} mrpt-system mrpt-obs mrpt-io mrpt-tclap mrpt-serialization)
endif()
