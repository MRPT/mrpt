INCLUDE(../../cmakemodules/AssureCMakeRootFile.cmake) # Avoid user mistake in CMake source directory

#-----------------------------------------------------------------
# CMake file for the MRPT application:  srba-slam
#  Run with "cmake ." at the root directory
#  Jan 2013, Jose Luis Blanco
#-----------------------------------------------------------------
PROJECT(srba_slam)

# ---------------------------------------------
# TARGET:
# ---------------------------------------------
# Define the executable target:
FILE(GLOB LST_SRCS "*.cpp")
FILE(GLOB LST_HDRS "*.h")


ADD_EXECUTABLE(srba-slam
	${LST_SRCS}
	${LST_HDRS}
	 ${MRPT_VERSION_RC_FILE}
	)
	
# Precompiled hdrs for MSVC:
IF(MRPT_ENABLE_PRECOMPILED_HDRS AND MSVC)
	# The "use precomp.headr" for all the files...
	set_target_properties(srba-slam
		PROPERTIES
		COMPILE_FLAGS "/Yusrba-run-generic-impl.h")

	# But for the file used to build the precomp. header:
	set_source_files_properties("${CMAKE_CURRENT_SOURCE_DIR}/instance_relative_graph_slam_se2.cpp"
		PROPERTIES
		COMPILE_FLAGS "/Ycsrba-run-generic-impl.h")
		
	# Skip the main.cpp from precompiled headers:
	set_source_files_properties("${CMAKE_CURRENT_SOURCE_DIR}/srba-slam_main.cpp" PROPERTIES COMPILE_FLAGS "/Y-")
		
ENDIF(MRPT_ENABLE_PRECOMPILED_HDRS AND MSVC)


SET(TMP_TARGET_NAME "srba-slam")

# Add the required libraries for linking:
TARGET_LINK_LIBRARIES(${TMP_TARGET_NAME} ${MRPT_LINKER_LIBS})

# Dependencies on MRPT libraries:
#  Just mention the top-level dependency, the rest will be detected automatically,
#  and all the needed #include<> dirs added (see the script DeclareAppDependencies.cmake for further details)
DeclareAppDependencies(${TMP_TARGET_NAME} mrpt-srba mrpt-gui mrpt-vision)

DeclareAppForInstall(${TMP_TARGET_NAME})


