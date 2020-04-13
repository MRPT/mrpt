

if(NOT (CMAKE_MRPT_HAS_YAMLCPP AND TARGET ros1bridge))
	return()
endif()

project(rosbag2rawlog)

# ================================================
# TARGET: gps2rawlog
# ================================================
# Define the executable target:
add_executable(${PROJECT_NAME}
	# Main:
	rosbag2rawlog_main.cpp
	${MRPT_VERSION_RC_FILE}
 	)

# Add the required libraries for linking:
target_link_libraries(${PROJECT_NAME}
	mrpt::ros1bridge
	${rosbag_storage_LIBRARIES}
	${cv_bridge_LIBRARIES}
	${tf2_LIBRARIES}
	${LIBYAMLCPP_LIBRARIES}
)

target_include_directories(${PROJECT_NAME}
	SYSTEM
	PRIVATE
	${roscpp_INCLUDE_DIRS}
	${rosbag_storage_INCLUDE_DIRS}
	${cv_bridge_INCLUDE_DIRS}
	${tf2_INCLUDE_DIRS}
)

DeclareAppForInstall(${PROJECT_NAME})
# Dependencies on MRPT libraries: Just mention the top-level dependency, the rest will be detected automatically,
#  and all the needed #include<> dirs added (see the script DeclareAppDependencies.cmake for further details)
DeclareAppDependencies(${PROJECT_NAME} mrpt::system mrpt::obs mrpt::io mrpt::tclap mrpt::serialization)
