# Check for Matlab (Only in Linux, there is no
#  Windows version yet...)
# ===================================================
set(CMAKE_MRPT_HAS_MATLAB 0)

# Natural option to set ON the building of Matlab wrapper
# --------------------------------------------------------
if(MRPT_WITH_MATLAB_WRAPPER)

# Set sensible initial path for Matlab
if(NOT MATLAB_ROOT)
        if(WINDOWS) # In Windows
                set(MATLAB_ROOT "C:")
        else() # In Linux
                set(MATLAB_ROOT "/usr/local/MATLAB")
        endif()

        if(IS_DIRECTORY ${MATLAB_ROOT}) # Search installed versions in default directory
                file(GLOB LIST_MATLAB_INSTALLS RELATIVE ${MATLAB_ROOT} ${MATLAB_ROOT}/*)
                # Use first subdirectory (usually newest version)
                list(SORT LIST_MATLAB_INSTALLS) # Sort alphabetically
                list(REVERSE LIST_MATLAB_INSTALLS) # Reverse order
                list(GET LIST_MATLAB_INSTALLS 0 MATLAB_VERSION) # Take first element (highest version)

                set(MATLAB_ROOT "${MATLAB_ROOT}/${MATLAB_VERSION}")
        endif()

        set(MATLAB_ROOT ${MATLAB_ROOT} CACHE PATH "Path to the MATLAB installation directory (e.g. /usr/local/MATLAB/R2012b, C:/... (TODO)")
        set(MATLAB_VERSION ${MATLAB_VERSION} CACHE STRING "R...-like version to use from installed ones")
endif()

# DISABLE_MATLAB
# ---------------------
option(DISABLE_MATLAB "Force not using Matlab" "OFF")
mark_as_advanced(DISABLE_MATLAB)
if(NOT DISABLE_MATLAB)

# Use CMAKE module if Matlab's not been detected yet:
if(NOT CMAKE_MRPT_HAS_MATLAB)
# TODO: This behaviour does not allow to update found libraries if MATLAB_ROOT is changed
		find_package(Matlab)
		if(MATLAB_FOUND)
				set(CMAKE_MRPT_HAS_MATLAB 1)

        list(APPEND MATLAB_LINK_LIBRARIES ${MATLAB_LIBRARIES} )

				# ----------------------------------------------------
				# Windows & MSVC: Mark Matlab DLLs as "delay-load", so
				#  non-mex apps can be run standalone without MATLAB:
				# ----------------------------------------------------
				if (MSVC)
          list(APPEND MATLAB_LINK_LIBRARIES "delayimp.lib")
					# Flags /DELAYLOAD:... added in DeclareMRPTLib.cmake
				endif (MSVC)

				# MEXPLUS header-only lib to handle mxArray class:
				add_subdirectory("${MRPT_SOURCE_DIR}/3rdparty/mexplus/")
				include_directories("${MRPT_SOURCE_DIR}/3rdparty/mexplus/")
		else()
			message("MATLAB not found. Either set MATLAB_ROOT correctly, or set MRPT_WITH_MATLAB_WRAPPER=OFF")
		endif()
endif()

# It seems it works with dynamic libraries too now!
## Set special options for Matlab wrapper compatibility
#if(BUILD_SHARED_LIBS)
#        message(SEND_ERROR
#"BUILD_SHARED_LIBS is activated.
#Static libraries are needed for MEX libraries due to TLS limitation in Matlab. Deactivate BUILD_SHARED_LIBRARIES.")
#endif()

## Since MEX libraries are dynamic but MRPT libraries need to be static, the static libraries must be Position Independent Code (PIC)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# Copy all .m files (classes, samples, helpers...) to the build directory
file(COPY ${MRPT_SOURCE_DIR}/mex/+mrpt   DESTINATION ${CMAKE_BINARY_DIR}/mex)
file(COPY ${MRPT_SOURCE_DIR}/mex/samples DESTINATION ${CMAKE_BINARY_DIR}/mex)
# Copy setup file and README
file(COPY ${MRPT_SOURCE_DIR}/mex/mrpt_setup.m DESTINATION ${CMAKE_BINARY_DIR}/mex)
file(COPY ${MRPT_SOURCE_DIR}/mex/README.txt   DESTINATION ${CMAKE_BINARY_DIR}/mex)

endif()

endif()
