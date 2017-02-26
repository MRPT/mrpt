INCLUDE(../../cmakemodules/AssureCMakeRootFile.cmake) # Avoid user mistake in CMake source directory

#-----------------------------------------------------------------
# CMake file for the MRPT application:  RawLogGrabber
#
#  Run with "cmake ." at the root directory
#
#  October 2007, Jose Luis Blanco <jlblanco@ctima.uma.es>
#-----------------------------------------------------------------
PROJECT(rawlog-grabber)




# Allow optional rawlog-grabber (compile-time) plug-ins:
#  Described in: http://www.mrpt.org/Application:rawlog-grabber
#  Plug-in directory must contain:
#    DIR/register.cpp  - with function "void rawlog_grabber_plugin_register()"
#    DIR/rawloggrabber_plugin.h  - with includes to all necesary classes.
#    DIR/*.cpp & DIR/*.h  - All .cpp and .h files will be included in rawlog-grabber project.
#    DIR/rawloggrabber_plugin.cmake  - Define the variable RAWLOGGRABBER_PLUGIN_LIBS
#
#  The preprocessor define "RAWLOGGRABBER_PLUGIN" will be defined.
#
SET(RAWLOGGRABBER_PLUGIN_DIR "" CACHE PATH "Path to a rawlog-grabber plug-in directory (empty: don't use)")
MARK_AS_ADVANCED(RAWLOGGRABBER_PLUGIN_DIR)

if (NOT "${RAWLOGGRABBER_PLUGIN_DIR}" STREQUAL "")
	if (EXISTS "${RAWLOGGRABBER_PLUGIN_DIR}")
		if (NOT EXISTS "${RAWLOGGRABBER_PLUGIN_DIR}/register.cpp")
			MESSAGE(FATAL_ERROR "Selected rawlog-grabber plug-in directory (${RAWLOGGRABBER_PLUGIN_DIR}) does not contain a register.cpp!")
		else (NOT EXISTS "${RAWLOGGRABBER_PLUGIN_DIR}/register.cpp")
			# OK:
			FILE(GLOB aux_srcs "${RAWLOGGRABBER_PLUGIN_DIR}/*.cpp")
			FILE(GLOB aux_hdrs "${RAWLOGGRABBER_PLUGIN_DIR}/*.h")
			INCLUDE_DIRECTORIES("${RAWLOGGRABBER_PLUGIN_DIR}")
			ADD_DEFINITIONS(-DRAWLOGGRABBER_PLUGIN)
			IF (EXISTS "${RAWLOGGRABBER_PLUGIN_DIR}/rawloggrabber_plugin.cmake")
				INCLUDE("${RAWLOGGRABBER_PLUGIN_DIR}/rawloggrabber_plugin.cmake")
				MESSAGE(STATUS "rawlog-grabber plug-in libs: ${RAWLOGGRABBER_PLUGIN_LIBS}")
			ENDIF (EXISTS "${RAWLOGGRABBER_PLUGIN_DIR}/rawloggrabber_plugin.cmake")

			MESSAGE(STATUS "Successful detection of rawlog-grabber plug-in directory: ${RAWLOGGRABBER_PLUGIN_DIR}")
		endif (NOT EXISTS "${RAWLOGGRABBER_PLUGIN_DIR}/register.cpp")
	else (EXISTS "${RAWLOGGRABBER_PLUGIN_DIR}")
		MESSAGE(FATAL_ERROR "Selected rawlog-grabber plug-in directory does not exist!")
	endif (EXISTS "${RAWLOGGRABBER_PLUGIN_DIR}")
endif (NOT "${RAWLOGGRABBER_PLUGIN_DIR}" STREQUAL "")



# ---------------------------------------------
# TARGET:
# ---------------------------------------------
# Define the executable target:
ADD_EXECUTABLE(${PROJECT_NAME}
               rawloggrabber_main.cpp
			   ${aux_srcs}
			   ${aux_hdrs}
			    ${MRPT_VERSION_RC_FILE}
			   )

# Add the required libraries for linking:
TARGET_LINK_LIBRARIES(${PROJECT_NAME} ${MRPT_LINKER_LIBS_RELorDEB} )
TARGET_LINK_LIBRARIES(${PROJECT_NAME} mrpt-hwdrivers${MRPT_LINKER_LIBS_POSTFIX} ${RAWLOGGRABBER_PLUGIN_LIBS} )

# Dependencies on MRPT libraries:
#  Just mention the top-level dependency, the rest will be detected automatically,
#  and all the needed #include<> dirs added (see the script DeclareAppDependencies.cmake for further details)
DeclareAppDependencies(${PROJECT_NAME} mrpt-hwdrivers mrpt-obs)

DeclareAppForInstall(${PROJECT_NAME})
