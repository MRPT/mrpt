include(../../cmakemodules/AssureCMakeRootFile.cmake) # Avoid user mistake in CMake source directory

#-----------------------------------------------------------------
# CMake file for the MRPT application:  RawLogGrabber
#
#  Run with "cmake ." at the root directory
#
#  October 2007, Jose Luis Blanco <jlblanco@ctima.uma.es>
#-----------------------------------------------------------------
project(rawlog-grabber)




# Allow optional rawlog-grabber (compile-time) plug-ins:
#  Described in: http://www.mrpt.org/list-of-mrpt-apps/application-rawlog-grabber
#  Plug-in directory must contain:
#    DIR/register.cpp  - with function "void rawlog_grabber_plugin_register()"
#    DIR/rawloggrabber_plugin.h  - with includes to all necesary classes.
#    DIR/*.cpp & DIR/*.h  - All .cpp and .h files will be included in rawlog-grabber project.
#    DIR/rawloggrabber_plugin.cmake  - Define the variable RAWLOGGRABBER_PLUGIN_LIBS
#
#  The preprocessor define "RAWLOGGRABBER_PLUGIN" will be defined.
#
set(RAWLOGGRABBER_PLUGIN_DIR "" CACHE PATH "Path to a rawlog-grabber plug-in directory (empty: don't use)")
mark_as_advanced(RAWLOGGRABBER_PLUGIN_DIR)

if (NOT "${RAWLOGGRABBER_PLUGIN_DIR}" STREQUAL "")
	if (EXISTS "${RAWLOGGRABBER_PLUGIN_DIR}")
		if (NOT EXISTS "${RAWLOGGRABBER_PLUGIN_DIR}/register.cpp")
			message(FATAL_ERROR "Selected rawlog-grabber plug-in directory (${RAWLOGGRABBER_PLUGIN_DIR}) does not contain a register.cpp!")
		else (NOT EXISTS "${RAWLOGGRABBER_PLUGIN_DIR}/register.cpp")
			# OK:
			file(GLOB aux_srcs "${RAWLOGGRABBER_PLUGIN_DIR}/*.cpp")
			file(GLOB aux_hdrs "${RAWLOGGRABBER_PLUGIN_DIR}/*.h")
			include_directories("${RAWLOGGRABBER_PLUGIN_DIR}")
			add_definitions(-DRAWLOGGRABBER_PLUGIN)
			if (EXISTS "${RAWLOGGRABBER_PLUGIN_DIR}/rawloggrabber_plugin.cmake")
				include("${RAWLOGGRABBER_PLUGIN_DIR}/rawloggrabber_plugin.cmake")
				message(STATUS "rawlog-grabber plug-in libs: ${RAWLOGGRABBER_PLUGIN_LIBS}")
			endif (EXISTS "${RAWLOGGRABBER_PLUGIN_DIR}/rawloggrabber_plugin.cmake")

			message(STATUS "Successful detection of rawlog-grabber plug-in directory: ${RAWLOGGRABBER_PLUGIN_DIR}")
		endif (NOT EXISTS "${RAWLOGGRABBER_PLUGIN_DIR}/register.cpp")
	else (EXISTS "${RAWLOGGRABBER_PLUGIN_DIR}")
		message(FATAL_ERROR "Selected rawlog-grabber plug-in directory does not exist!")
	endif (EXISTS "${RAWLOGGRABBER_PLUGIN_DIR}")
endif (NOT "${RAWLOGGRABBER_PLUGIN_DIR}" STREQUAL "")



# ---------------------------------------------
# TARGET:
# ---------------------------------------------
# Define the executable target:
add_executable(${PROJECT_NAME}
               rawloggrabber_main.cpp
			   ${aux_srcs}
			   ${aux_hdrs}
			    ${MRPT_VERSION_RC_FILE}
			   )

# Add the required libraries for linking:
target_link_libraries(${PROJECT_NAME} ${MRPT_LINKER_LIBS_RELorDEB} )
target_link_libraries(${PROJECT_NAME} mrpt-hwdrivers${MRPT_LINKER_LIBS_POSTFIX} ${RAWLOGGRABBER_PLUGIN_LIBS} )

# Dependencies on MRPT libraries:
#  Just mention the top-level dependency, the rest will be detected automatically,
#  and all the needed #include<> dirs added (see the script DeclareAppDependencies.cmake for further details)
DeclareAppDependencies(${PROJECT_NAME} mrpt-hwdrivers mrpt-obs)

DeclareAppForInstall(${PROJECT_NAME})
