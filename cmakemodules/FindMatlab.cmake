# - this module looks for Matlab
# Defines:
#  MATLAB_INCLUDE_DIR: include path for mex.h
#  MATLAB_LIBRARIES:   required libraries: libmex, libmx
#  MATLAB_MEX_LIBRARY: path to libmex
#  MATLAB_MX_LIBRARY:  path to libmx
#  MATLAB_FOUND
#  MATLAB_ROOT:        path to installed Matlab used
#  MATLAB_VERSION:     the R...-like version of used Matlab

set(MATLAB_FOUND 0)

find_path(MATLAB_INCLUDE_DIR mex.h
          ${MATLAB_ROOT}/extern/include)

if(MATLAB_INCLUDE_DIR) # Protect against #include'ing undefined dirs.
        include_directories(${MATLAB_INCLUDE_DIR})
endif(MATLAB_INCLUDE_DIR)

find_library( MATLAB_MEX_LIBRARY
              NAMES libmex mex
              PATHS ${MATLAB_ROOT}/bin ${MATLAB_ROOT}/extern/lib
              PATH_SUFFIXES glnxa64 glnx86 win64/microsoft win32/microsoft)

find_library( MATLAB_MX_LIBRARY
              NAMES libmx mx
              PATHS ${MATLAB_ROOT}/bin ${MATLAB_ROOT}/extern/lib
              PATH_SUFFIXES glnxa64 glnx86 win64/microsoft win32/microsoft)

# This is common to UNIX and Win32:
set(MATLAB_LIBRARIES
  ${MATLAB_MEX_LIBRARY}
  ${MATLAB_MX_LIBRARY}
)

if(MATLAB_INCLUDE_DIR AND MATLAB_LIBRARIES)
        set(MATLAB_FOUND 1)
#        get_filename_component(MATLAB_VERSION ${MATLAB_ROOT} NAME) # Find MATLAB version
#        set(MATLAB_VERSION ${MATLAB_VERSION} CACHE STRING "" FORCE) # Force value over GUI
endif(MATLAB_INCLUDE_DIR AND MATLAB_LIBRARIES)

mark_as_advanced(
  MATLAB_LIBRARIES
  MATLAB_MEX_LIBRARY
  MATLAB_MX_LIBRARY
  MATLAB_INCLUDE_DIR
  MATLAB_FOUND
  MATLAB_ROOT
  MATLAB_VERSION
)
