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
	rawlog-edit_list-images.cpp
	rawlog-edit_list-rangbear.cpp
	rawlog-edit_generate-3d-pointclouds.cpp
	rawlog-edit_generate-pcd.cpp
	)
SET(TMP_TARGET_NAME "rawlog-edit")
SET_TARGET_PROPERTIES(${TMP_TARGET_NAME} PROPERTIES PROJECT_LABEL "(APP) ${TMP_TARGET_NAME}")
# Add the required libraries for linking:
TARGET_LINK_LIBRARIES(${TMP_TARGET_NAME} ${MRPT_LINKER_LIBS})
INSTALL(TARGETS ${TMP_TARGET_NAME}
	RUNTIME DESTINATION ${mrpt_apps_INSTALL_PREFIX}bin
	LIBRARY DESTINATION ${mrpt_apps_INSTALL_PREFIX}lib${LIB_SUFFIX}
	ARCHIVE DESTINATION ${mrpt_apps_INSTALL_PREFIX}lib${LIB_SUFFIX})
# Dependencies on MRPT libraries: Just mention the top-level dependency, the rest will be detected automatically,
#  and all the needed #include<> dirs added (see the script DeclareAppDependencies.cmake for further details)
DeclareAppDependencies(${TMP_TARGET_NAME} mrpt-obs mrpt-topography mrpt-maps)



# ================================================
# TARGET: carmen2rawlog
# ================================================
# Define the executable target:
ADD_EXECUTABLE(carmen2rawlog
	carmen2rawlog_main.cpp
	)
SET(TMP_TARGET_NAME "carmen2rawlog")
SET_TARGET_PROPERTIES(${TMP_TARGET_NAME} PROPERTIES PROJECT_LABEL "(APP) ${TMP_TARGET_NAME}")
# Add the required libraries for linking:
TARGET_LINK_LIBRARIES(${TMP_TARGET_NAME} ${MRPT_LINKER_LIBS})
INSTALL(TARGETS ${TMP_TARGET_NAME}
	RUNTIME DESTINATION ${mrpt_apps_INSTALL_PREFIX}bin
	LIBRARY DESTINATION ${mrpt_apps_INSTALL_PREFIX}lib${LIB_SUFFIX}
	ARCHIVE DESTINATION ${mrpt_apps_INSTALL_PREFIX}lib${LIB_SUFFIX})
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
SET_TARGET_PROPERTIES(${TMP_TARGET_NAME} PROPERTIES PROJECT_LABEL "(APP) ${TMP_TARGET_NAME}")
# Add the required libraries for linking:
TARGET_LINK_LIBRARIES(${TMP_TARGET_NAME} ${MRPT_LINKER_LIBS})
INSTALL(TARGETS ${TMP_TARGET_NAME}
        RUNTIME DESTINATION ${mrpt_apps_INSTALL_PREFIX}bin
        LIBRARY DESTINATION ${mrpt_apps_INSTALL_PREFIX}lib${LIB_SUFFIX}
        ARCHIVE DESTINATION ${mrpt_apps_INSTALL_PREFIX}lib${LIB_SUFFIX})
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
SET_TARGET_PROPERTIES(${TMP_TARGET_NAME} PROPERTIES PROJECT_LABEL "(APP) ${TMP_TARGET_NAME}")
# Add the required libraries for linking:
TARGET_LINK_LIBRARIES(${TMP_TARGET_NAME} ${MRPT_LINKER_LIBS})
INSTALL(TARGETS ${TMP_TARGET_NAME}
	RUNTIME DESTINATION ${mrpt_apps_INSTALL_PREFIX}bin
	LIBRARY DESTINATION ${mrpt_apps_INSTALL_PREFIX}lib${LIB_SUFFIX}
	ARCHIVE DESTINATION ${mrpt_apps_INSTALL_PREFIX}lib${LIB_SUFFIX})
# Dependencies on MRPT libraries: Just mention the top-level dependency, the rest will be detected automatically,
#  and all the needed #include<> dirs added (see the script DeclareAppDependencies.cmake for further details)
DeclareAppDependencies(${TMP_TARGET_NAME} mrpt-obs)

endif(0)

