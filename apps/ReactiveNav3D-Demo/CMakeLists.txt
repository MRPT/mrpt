include(../../cmakemodules/AssureCMakeRootFile.cmake) # Avoid user mistake in CMake source directory

#-----------------------------------------------------------------
# CMake file for the MRPT application:  ReactiveNav3D-Demo
#
#  Run with "cmake ." at the root directory
#
#  Juin 2013, Mariano Jaimez Tarifa <marianojt@uma.es>
#-----------------------------------------------------------------
project(ReactiveNav3D-Demo)



# ---------------------------------------------
# TARGET:
# ---------------------------------------------
# Define the executable target:
add_executable(${PROJECT_NAME}
               ReactiveNav3D_demo_main.cpp
			   ReactiveNav3D_demo.h
			    ${MRPT_VERSION_RC_FILE})

set(TMP_TARGET_NAME "ReactiveNav3D-Demo")



# Add the required libraries for linking:
target_link_libraries(${PROJECT_NAME} ${MRPT_LINKER_LIBS})

# Dependencies on MRPT libraries:
#  Just mention the top-level dependency, the rest will be detected automatically,
#  and all the needed #include<> dirs added (see the script DeclareAppDependencies.cmake for further details)
DeclareAppDependencies(${PROJECT_NAME} mrpt-gui mrpt-slam mrpt-nav)

DeclareAppForInstall(${PROJECT_NAME})



#find_package( MRPT REQUIRED base gui opengl slam maps reactivenav)
