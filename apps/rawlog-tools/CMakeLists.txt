INCLUDE(../../cmakemodules/AssureCMakeRootFile.cmake) # Avoid user mistake in CMake source directory

#-----------------------------------------------------------------
# CMake file for rawlog conversion tools
#
#  Run with "cmake ." at the root directory
#-----------------------------------------------------------------
PROJECT(rawlog_tools)

# ================================================
# TARGET: rawlog-edit
# ================================================
# Define the executable target:
ADD_EXECUTABLE(rawlog-edit
	# Main:
	rawlog-edit_main.cpp
	# Headers
	rawlog-edit-declarations.h
	CRawlogProcessor.h
	# Ops:
	rawlog-edit_gps.cpp
	rawlog-edit_info.cpp
	rawlog-edit_externalize.cpp
	rawlog-edit_filters.cpp
	rawlog-edit_cuts.cpp
	rawlog-edit_sensor-poses.cpp
	rawlog-edit_camera-params.cpp
	rawlog-edit_list-images.cpp
	rawlog-edit_list-rangbear.cpp
	rawlog-edit_generate-3d-pointclouds.cpp
	rawlog-edit_generate-pcd.cpp
	rawlog-edit_stereo-rectify.cpp
	rawlog-edit_rename_externals.cpp
	rawlog-edit_list-timestamps.cpp
	rawlog-edit_remap_timestamps.cpp
	)
SET(TMP_TARGET_NAME "rawlog-edit")

# Add the required libraries for linking:
TARGET_LINK_LIBRARIES(${TMP_TARGET_NAME} ${MRPT_LINKER_LIBS})
DeclareAppForInstall(${TMP_TARGET_NAME})
# Dependencies on MRPT libraries: Just mention the top-level dependency, the rest will be detected automatically,
#  and all the needed #include<> dirs added (see the script DeclareAppDependencies.cmake for further details)
DeclareAppDependencies(${TMP_TARGET_NAME} mrpt-obs mrpt-topography mrpt-maps mrpt-vision)



# ================================================
# TARGET: carmen2rawlog
# ================================================
# Define the executable target:
ADD_EXECUTABLE(carmen2rawlog
	carmen2rawlog_main.cpp
	)
SET(TMP_TARGET_NAME "carmen2rawlog")

# Add the required libraries for linking:
TARGET_LINK_LIBRARIES(${TMP_TARGET_NAME} ${MRPT_LINKER_LIBS})
DeclareAppForInstall(${TMP_TARGET_NAME})
# Dependencies on MRPT libraries: Just mention the top-level dependency, the rest will be detected automatically,
#  and all the needed #include<> dirs added (see the script DeclareAppDependencies.cmake for further details)
DeclareAppDependencies(${TMP_TARGET_NAME} mrpt-obs)


# ================================================
# TARGET: carmen2simplemap
# ================================================
# Define the executable target:
ADD_EXECUTABLE(carmen2simplemap
        carmen2simplemap_main.cpp
        )
SET(TMP_TARGET_NAME "carmen2simplemap")

# Add the required libraries for linking:
TARGET_LINK_LIBRARIES(${TMP_TARGET_NAME} ${MRPT_LINKER_LIBS})
DeclareAppForInstall(${TMP_TARGET_NAME})
# Dependencies on MRPT libraries: Just mention the top-level dependency, the rest will be detected automatically,
#  and all the needed #include<> dirs added (see the script DeclareAppDependencies.cmake for further details)
DeclareAppDependencies(${TMP_TARGET_NAME} mrpt-obs)



if(0) # FUTURE WORK!!
# ================================================
# TARGET: rawlog2carmen
# ================================================
# Define the executable target:
ADD_EXECUTABLE(rawlog2carmen
	rawlog2carmen_main.cpp
	)
SET(TMP_TARGET_NAME "rawlog2carmen")

# Add the required libraries for linking:
TARGET_LINK_LIBRARIES(${TMP_TARGET_NAME} ${MRPT_LINKER_LIBS})
DeclareAppForInstall(${TMP_TARGET_NAME})
# Dependencies on MRPT libraries: Just mention the top-level dependency, the rest will be detected automatically,
#  and all the needed #include<> dirs added (see the script DeclareAppDependencies.cmake for further details)
DeclareAppDependencies(${TMP_TARGET_NAME} mrpt-obs)

endif(0)

