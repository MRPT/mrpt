# Check for Matlab (Only in Linux, there is no
#  Windows version yet...)
# ===================================================
SET(CMAKE_MRPT_HAS_MATLAB 0)

# DISABLE_MATLAB
# ---------------------
OPTION(DISABLE_MATLAB "Force not using Matlab" "OFF")
MARK_AS_ADVANCED(DISABLE_MATLAB)
IF(NOT DISABLE_MATLAB)

# Use CMAKE module if opencv's not been detected yet:
IF(NOT CMAKE_MRPT_HAS_MATLAB)
        FIND_PACKAGE(Matlab)
        IF(MATLAB_FOUND)
                MESSAGE(STATUS "MATLAB found in script_matlab.cmake")

                # TODO: Make this option and detail more clear to user!
                # If MEX libraries are going to be generated, static PIC libraries should be built for MRPT
                SET(CMAKE_MRPT_BUILD_STATIC_PIC_ONOFF 1)

                SET(CMAKE_MRPT_HAS_MATLAB 1)
        ENDIF(MATLAB_FOUND)
ENDIF(NOT CMAKE_MRPT_HAS_MATLAB)

# libs for Matlab MEX:
IF (MATLAB_FOUND)
        APPEND_MRPT_LIBS( ${MATLAB_LIBRARIES} )
ENDIF (MATLAB_FOUND)

ENDIF(NOT DISABLE_MATLAB)
