# Check for Matlab (Only in Linux, there is no
#  Windows version yet...)
# ===================================================
SET(CMAKE_MRPT_HAS_MATLAB 0)

IF(NOT UNIX)
    MESSAGE(WARNING "MRPT-MEX wrapper has been tested only in Linux. No Windows support exists yet.")
    SET(BUILD_MATLAB 0)
ENDIF(NOT UNIX)

# Natural option to set ON the building of Matlab wrapper
# --------------------------------------------------------
IF(BUILD_MATLAB)
SET(MATLAB_ROOT "/usr/local/MATLAB/" CACHE PATH "Path to the MATLAB installation directory (e.g. /usr/local/MATLAB/R2012b)")

# DISABLE_MATLAB
# ---------------------
OPTION(DISABLE_MATLAB "Force not using Matlab" "OFF")
MARK_AS_ADVANCED(DISABLE_MATLAB)
IF(NOT DISABLE_MATLAB)

# Use CMAKE module if opencv's not been detected yet:
IF(NOT CMAKE_MRPT_HAS_MATLAB)
        FIND_PACKAGE(Matlab)
        IF(MATLAB_FOUND)
                SET(CMAKE_MRPT_HAS_MATLAB 1)
                APPEND_MRPT_LIBS( ${MATLAB_LIBRARIES} )

                # MEXPLUS header-only lib to handle mxArray class:
                INCLUDE_DIRECTORIES("${MRPT_SOURCE_DIR}/otherlibs/mexplus/")
        ENDIF(MATLAB_FOUND)
ENDIF(NOT CMAKE_MRPT_HAS_MATLAB)

# It seems it works with dynamic libraries too now!
## Set special options for Matlab wrapper compatibility
#IF(BUILD_SHARED_LIBS)
#        MESSAGE(SEND_ERROR
#"BUILD_SHARED_LIBS is activated.
#Static libraries are needed for MEX libraries due to TLS limitation in Matlab. Deactivate BUILD_SHARED_LIBRARIES.")
#ENDIF(BUILD_SHARED_LIBS)

## Since MEX libraries are dynamic but MRPT libraries need to be static, the static libraries must be Position Independent Code (PIC)
#SET(EXTRA_CPP_FLAGS "${EXTRA_CPP_FLAGS} -fPIC")

# Copy all .m files (classes, samples, helpers...) to the build directory
FILE(COPY ${CMAKE_SOURCE_DIR}/mex/+mrpt DESTINATION ${CMAKE_BINARY_DIR}/mex)
FILE(COPY ${CMAKE_SOURCE_DIR}/mex/samples DESTINATION ${CMAKE_BINARY_DIR}/mex)

ENDIF(NOT DISABLE_MATLAB)

ENDIF(BUILD_MATLAB)
