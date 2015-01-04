# - this module looks for Matlab
# Defines:
#  MATLAB_INCLUDE_DIR: include path for mex.h
#  MATLAB_LIBRARIES:   required libraries: libmex, libmx
#  MATLAB_MEX_LIBRARY: path to libmex
#  MATLAB_MX_LIBRARY:  path to libmx
#  MATLAB_FOUND
#  MATLAB_ROOT:        path to installed Matlab used
#  MATLAB_VERSION:     the R...-like version of used Matlab

SET(MATLAB_FOUND 0)

FIND_PATH(MATLAB_INCLUDE_DIR mex.h
          ${MATLAB_ROOT}/extern/include)

IF(MATLAB_INCLUDE_DIR) # Protect against #include'ing undefined dirs.
        INCLUDE_DIRECTORIES(${MATLAB_INCLUDE_DIR})
ENDIF(MATLAB_INCLUDE_DIR)

FIND_LIBRARY( MATLAB_MEX_LIBRARY
              NAMES libmex mex
              PATHS ${MATLAB_ROOT}/bin ${MATLAB_ROOT}/extern/lib
              PATH_SUFFIXES glnxa64 glnx86 win64/microsoft win32/microsoft)

FIND_LIBRARY( MATLAB_MX_LIBRARY
              NAMES libmx mx
              PATHS ${MATLAB_ROOT}/bin ${MATLAB_ROOT}/extern/lib
              PATH_SUFFIXES glnxa64 glnx86 win64/microsoft win32/microsoft)

# This is common to UNIX and Win32:
SET(MATLAB_LIBRARIES
  ${MATLAB_MEX_LIBRARY}
  ${MATLAB_MX_LIBRARY}
)

IF(MATLAB_INCLUDE_DIR AND MATLAB_LIBRARIES)
        SET(MATLAB_FOUND 1)
#        GET_FILENAME_COMPONENT(MATLAB_VERSION ${MATLAB_ROOT} NAME) # Find MATLAB version
#        SET(MATLAB_VERSION ${MATLAB_VERSION} CACHE STRING "" FORCE) # Force value over GUI
ENDIF(MATLAB_INCLUDE_DIR AND MATLAB_LIBRARIES)

MARK_AS_ADVANCED(
  MATLAB_LIBRARIES
  MATLAB_MEX_LIBRARY
  MATLAB_MX_LIBRARY
  MATLAB_INCLUDE_DIR
  MATLAB_FOUND
  MATLAB_ROOT
  MATLAB_VERSION
)
