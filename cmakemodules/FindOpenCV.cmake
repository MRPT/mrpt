# - Try to find OpenCV library installation
# See http://sourceforge.net/projects/opencvlibrary/
#
# The follwoing variables are optionally searched for defaults
#  OpenCV_ROOT_DIR:            Base directory of OpenCv tree to use.
#  OpenCV_FIND_REQUIRED_COMPONENTS : find_package(OpenCV COMPONENTS ..) 
#    compatible interface. typically  CV CXCORE CVAUX HIGHGUI CVCAM .. etc.
#
# The following are set after configuration is done: 
#  OpenCV_FOUND
#  OpenCV_INCLUDE_DIR
#  OpenCV_LIBRARIES
#  OpenCV_LINK_DIRECTORIES
#
# deprecated:
#  OPENCV_* uppercase replaced by case sensitive OpenCV_*
#  OPENCV_EXE_LINKER_FLAGS
#  OPENCV_INCLUDE_DIR : replaced by plural *_DIRS
# 
# 2004/05 Jan Woetzel, Friso, Daniel Grest 
# 2006/01 complete rewrite by Jan Woetzel
# 1006/09 2nd rewrite introducing ROOT_DIR and PATH_SUFFIXES 
#   to handle multiple installed versions gracefully by Jan Woetzel
#
# tested with:
# -OpenCV 0.97 (beta5a):  MSVS 7.1, gcc 3.3, gcc 4.1
# -OpenCV 0.99 (1.0rc1):  MSVS 7.1, MSVS 8.0
#
# 20 Nov 2008: Changes by Jose Luis Blanco Claraco <http://www.isa.uma.es/jlblanco>
#    - Updated for finding OpenCV1.1.0-pre under Windows in the system 
#        registry (It can be installed along 1.0.0 without problems)
#    - Finds and uses debug libraries and DLLs (Windows).
#
# www.mip.informatik.uni-kiel.de/~jw
# --------------------------------

# NOTES
#  NO_CMAKE_SYSTEM_PATH is required to bypass searching in CMAKE_SYSTEM_LIBRARY_PATH
#  because it defaults to $ENV{ProgramFiles} in WindowsPaths.cmake 
#  which corrupts the order of BIAS_EXTERN_LIBS  (JW 02/2007)


macro(DBG_MSG _MSG)
  #message(STATUS "${CMAKE_CURRENT_LIST_FILE}(${CMAKE_CURRENT_LIST_LINE}):\n${_MSG}")
endmacro(DBG_MSG)



# required cv components with header and library if COMPONENTS unspecified
if    (NOT OpenCV_FIND_COMPONENTS)
  # default
  set(OpenCV_FIND_REQUIRED_COMPONENTS   CV CXCORE CVAUX HIGHGUI )
  if   (WIN32)
#    list(APPEND OpenCV_FIND_REQUIRED_COMPONENTS  CVCAM ) # WIN32 only actually
  endif(WIN32)  
endif (NOT OpenCV_FIND_COMPONENTS)


# typical root dirs of installations, exactly one of them is used
set (OpenCV_POSSIBLE_ROOT_DIRS
  "${OpenCV_ROOT_DIR}"
  "$ENV{OpenCV_ROOT_DIR}"  
  "$ENV{OPENCV_DIR}"  # only for backward compatibility deprecated by ROOT_DIR
  "$ENV{OPENCV_HOME}" # only for backward compatibility  
  "$ENV{EXTERN_LIBS_DIR}/OpenCV" # central precompiled libs
  "[HKEY_LOCAL_MACHINE\\SOFTWARE\\Microsoft\\Windows\\CurrentVersion\\Uninstall\\Open Source Computer Vision Library_is1;Inno Setup: App Path]" # works with OpenCV 1.1.0pre
  "[HKEY_LOCAL_MACHINE\\SOFTWARE\\Microsoft\\Windows\\CurrentVersion\\Uninstall\\Intel(R) Open Source Computer Vision Library_is1;Inno Setup: App Path]" # works with OpenCV beta5a
  "$ENV{ProgramFiles}/OpenCV"
  "$ENV{ProgramFiles}/OpenCV_10rc1"
  /usr/local
  /usr
  )


# MIP Uni Kiel /opt/net network installation 
# get correct prefix for current gcc compiler version for gcc 3.x  4.x
if    (${CMAKE_COMPILER_IS_GNUCXX})
  if    (NOT OpenCV_FIND_QUIETLY)
    message(STATUS "Checking GNUCXX version 3/4 to determine  OpenCV /opt/net/ path")
  endif (NOT OpenCV_FIND_QUIETLY)
  exec_program(${CMAKE_CXX_COMPILER} ARGS --version OUTPUT_VARIABLE CXX_COMPILER_VERSION)  
  if   (CXX_COMPILER_VERSION MATCHES ".*3\\.[0-9].*")
    set(IS_GNUCXX3 TRUE)
    list(APPEND OpenCV_POSSIBLE_ROOT_DIRS /opt/net/gcc33/OpenCV )
  endif(CXX_COMPILER_VERSION MATCHES ".*3\\.[0-9].*")  
  if   (CXX_COMPILER_VERSION MATCHES ".*4\\.[0-1].*")
    set(IS_GNUCXX4 TRUE)
    list(APPEND OpenCV_POSSIBLE_ROOT_DIRS /opt/net/gcc41/OpenCV )
  endif(CXX_COMPILER_VERSION MATCHES ".*4\\.[0-1].*")
  #SuSE 10.3, gcc4.2.1 needs OpenCV-1.0.0
  if   (CXX_COMPILER_VERSION MATCHES ".*4\\.[2-9].*")
    set(IS_GNUCXX4 TRUE)
    list(APPEND OpenCV_POSSIBLE_ROOT_DIRS /opt/net/gcc41/OpenCV-1.0.0)
  endif(CXX_COMPILER_VERSION MATCHES ".*4\\.[2-9].*")
endif (${CMAKE_COMPILER_IS_GNUCXX})

#DBG_MSG("OpenCV_POSSIBLE_ROOT_DIRS=${OpenCV_POSSIBLE_ROOT_DIRS}")

#
# select exactly ONE OpenCV base directory/tree 
# to avoid mixing different version headers and libs
#
find_path(OpenCV_ROOT_DIR 
  NAMES 
  cv/include/cv.h     # windows
  include/opencv/cv.h # linux /opt/net
  include/cv/cv.h 
  include/cv.h 
  PATHS ${OpenCV_POSSIBLE_ROOT_DIRS})
DBG_MSG("OpenCV_ROOT_DIR=${OpenCV_ROOT_DIR}")


# header include dir suffixes appended to OpenCV_ROOT_DIR
set(OpenCV_INCDIR_SUFFIXES
  include
  include/cv
  include/opencv
  cv/include
  cxcore/include
  cvaux/include
  otherlibs/cvcam/include
  otherlibs/highgui
  otherlibs/highgui/include
  otherlibs/_graphics/include
  )
DBG_MSG("OpenCV_INCDIR_SUFFIXES=${OpenCV_INCDIR_SUFFIXES}")

# library linkdir suffixes appended to OpenCV_ROOT_DIR 
set(OpenCV_LIBDIR_SUFFIXES
  lib
  OpenCV/lib
  otherlibs/_graphics/lib
  )
DBG_MSG("OpenCV_LIBDIR_SUFFIXES=${OpenCV_LIBDIR_SUFFIXES}")


#
# find incdir for each lib
#
find_path(OpenCV_CV_INCLUDE_DIR
  NAMES cv.h      
  PATHS ${OpenCV_ROOT_DIR} 
  PATH_SUFFIXES ${OpenCV_INCDIR_SUFFIXES} 
  NO_CMAKE_SYSTEM_PATH )
DBG_MSG("OpenCV_CV_INCLUDE_DIR=${OpenCV_CV_INCLUDE_DIR}")
find_path(OpenCV_CXCORE_INCLUDE_DIR   
  NAMES cxcore.h
  PATHS ${OpenCV_ROOT_DIR} 
  PATH_SUFFIXES ${OpenCV_INCDIR_SUFFIXES} 
  NO_CMAKE_SYSTEM_PATH )
DBG_MSG("OpenCV_CXCORE_INCLUDE_DIR=${OpenCV_CXCORE_INCLUDE_DIR}")
find_path(OpenCV_CVAUX_INCLUDE_DIR    
  NAMES cvaux.h
  PATHS ${OpenCV_ROOT_DIR} 
  PATH_SUFFIXES ${OpenCV_INCDIR_SUFFIXES} 
  NO_CMAKE_SYSTEM_PATH )
DBG_MSG("OpenCV_CVAUX_INCLUDE_DIR=${OpenCV_CVAUX_INCLUDE_DIR}")
find_path(OpenCV_HIGHGUI_INCLUDE_DIR  
  NAMES highgui.h 
  PATHS ${OpenCV_ROOT_DIR} 
  PATH_SUFFIXES ${OpenCV_INCDIR_SUFFIXES} 
  NO_CMAKE_SYSTEM_PATH )
DBG_MSG("OpenCV_HIGHGUI_INCLUDE_DIR=${OpenCV_HIGHGUI_INCLUDE_DIR}")
find_path(OpenCV_CVCAM_INCLUDE_DIR    
  NAMES cvcam.h 
  PATHS ${OpenCV_ROOT_DIR} 
  PATH_SUFFIXES ${OpenCV_INCDIR_SUFFIXES} 
  NO_CMAKE_SYSTEM_PATH )
DBG_MSG("OpenCV_CVCAM_INCLUDE_DIR=${OpenCV_CVCAM_INCLUDE_DIR}")

#
# find sbsolute path to all libraries 
# some are optionally, some may not exist on Linux
#
find_library(OpenCV_CV_LIBRARY   
  NAMES cv opencv cv0.9.7
  PATHS ${OpenCV_ROOT_DIR}  
  PATH_SUFFIXES  ${OpenCV_LIBDIR_SUFFIXES} 
  NO_CMAKE_SYSTEM_PATH )
DBG_MSG("OpenCV_CV_LIBRARY=${OpenCV_CV_LIBRARY}")
find_library(OpenCV_CVAUX_LIBRARY
  NAMES cvaux cvaux0.9.7
  PATHS ${OpenCV_ROOT_DIR}  PATH_SUFFIXES ${OpenCV_LIBDIR_SUFFIXES}
  NO_CMAKE_SYSTEM_PATH )
DBG_MSG("OpenCV_CVAUX_LIBRARY=${OpenCV_CVAUX_LIBRARY}")
find_library(OpenCV_CVCAM_LIBRARY   
  NAMES cvcam
  PATHS ${OpenCV_ROOT_DIR}  PATH_SUFFIXES ${OpenCV_LIBDIR_SUFFIXES} 
  NO_CMAKE_SYSTEM_PATH ) 
DBG_MSG("OpenCV_CVCAM_LIBRARY=${OpenCV_CVCAM_LIBRARY}")
find_library(OpenCV_CVHAARTRAINING_LIBRARY
  NAMES cvhaartraining
  PATHS ${OpenCV_ROOT_DIR}  PATH_SUFFIXES ${OpenCV_LIBDIR_SUFFIXES} 
  NO_CMAKE_SYSTEM_PATH ) 
DBG_MSG("OpenCV_CVHAARTRAINING_LIBRARY=${OpenCV_CVHAARTRAINING_LIBRARY}")
find_library(OpenCV_CXCORE_LIBRARY  
  NAMES cxcore cxcore0.9.7
  PATHS ${OpenCV_ROOT_DIR}  PATH_SUFFIXES ${OpenCV_LIBDIR_SUFFIXES} 
  NO_CMAKE_SYSTEM_PATH )
DBG_MSG("OpenCV_CXCORE_LIBRARY=${OpenCV_CXCORE_LIBRARY}")
find_library(OpenCV_CXTS_LIBRARY   
  NAMES cxts
  PATHS ${OpenCV_ROOT_DIR}  PATH_SUFFIXES ${OpenCV_LIBDIR_SUFFIXES} 
  NO_CMAKE_SYSTEM_PATH )
DBG_MSG("OpenCV_CXTS_LIBRARY=${OpenCV_CXTS_LIBRARY}")
find_library(OpenCV_HIGHGUI_LIBRARY  
  NAMES highgui highgui0.9.7
  PATHS ${OpenCV_ROOT_DIR}  PATH_SUFFIXES ${OpenCV_LIBDIR_SUFFIXES} 
  NO_CMAKE_SYSTEM_PATH )
DBG_MSG("OpenCV_HIGHGUI_LIBRARY=${OpenCV_HIGHGUI_LIBRARY}")
find_library(OpenCV_ML_LIBRARY  
  NAMES ml
  PATHS ${OpenCV_ROOT_DIR}  PATH_SUFFIXES ${OpenCV_LIBDIR_SUFFIXES} 
  NO_CMAKE_SYSTEM_PATH )
DBG_MSG("OpenCV_ML_LIBRARY=${OpenCV_ML_LIBRARY}")
find_library(OpenCV_TRS_LIBRARY  
  NAMES trs
  PATHS ${OpenCV_ROOT_DIR}  PATH_SUFFIXES ${OpenCV_LIBDIR_SUFFIXES} 
  NO_CMAKE_SYSTEM_PATH )
DBG_MSG("OpenCV_TRS_LIBRARY=${OpenCV_TRS_LIBRARY}")

# DEBUG VERSIONS:
find_library(OpenCV_CVd_LIBRARY   
  NAMES cvd opencvd cvd0.9.7
  PATHS ${OpenCV_ROOT_DIR}  
  PATH_SUFFIXES  ${OpenCV_LIBDIR_SUFFIXES} 
  NO_CMAKE_SYSTEM_PATH )
DBG_MSG("OpenCV_CVd_LIBRARY=${OpenCV_CVd_LIBRARY}")
find_library(OpenCV_CVAUXd_LIBRARY
  NAMES cvauxd cvauxd0.9.7
  PATHS ${OpenCV_ROOT_DIR}  PATH_SUFFIXES ${OpenCV_LIBDIR_SUFFIXES}
  NO_CMAKE_SYSTEM_PATH )
DBG_MSG("OpenCV_CVAUXd_LIBRARY=${OpenCV_CVAUXd_LIBRARY}")
find_library(OpenCV_CVCAMd_LIBRARY   
  NAMES cvcamd
  PATHS ${OpenCV_ROOT_DIR}  PATH_SUFFIXES ${OpenCV_LIBDIR_SUFFIXES} 
  NO_CMAKE_SYSTEM_PATH ) 
DBG_MSG("OpenCV_CVCAMd_LIBRARY=${OpenCV_CVCAMd_LIBRARY}")
find_library(OpenCV_CVHAARTRAININGd_LIBRARY
  NAMES cvhaartrainingd
  PATHS ${OpenCV_ROOT_DIR}  PATH_SUFFIXES ${OpenCV_LIBDIR_SUFFIXES} 
  NO_CMAKE_SYSTEM_PATH ) 
DBG_MSG("OpenCV_CVHAARTRAININGd_LIBRARY=${OpenCV_CVHAARTRAININGd_LIBRARY}")
find_library(OpenCV_CXCOREd_LIBRARY  
  NAMES cxcored cxcored0.9.7
  PATHS ${OpenCV_ROOT_DIR}  PATH_SUFFIXES ${OpenCV_LIBDIR_SUFFIXES} 
  NO_CMAKE_SYSTEM_PATH )
DBG_MSG("OpenCV_CXCOREd_LIBRARY=${OpenCV_CXCOREd_LIBRARY}")
find_library(OpenCV_CXTSd_LIBRARY   
  NAMES cxtsd
  PATHS ${OpenCV_ROOT_DIR}  PATH_SUFFIXES ${OpenCV_LIBDIR_SUFFIXES} 
  NO_CMAKE_SYSTEM_PATH )
DBG_MSG("OpenCV_CXTSd_LIBRARY=${OpenCV_CXTSd_LIBRARY}")
find_library(OpenCV_HIGHGUId_LIBRARY  
  NAMES highguid highguid0.9.7
  PATHS ${OpenCV_ROOT_DIR}  PATH_SUFFIXES ${OpenCV_LIBDIR_SUFFIXES} 
  NO_CMAKE_SYSTEM_PATH )
DBG_MSG("OpenCV_HIGHGUId_LIBRARY=${OpenCV_HIGHGUId_LIBRARY}")
find_library(OpenCV_MLd_LIBRARY  
  NAMES mld
  PATHS ${OpenCV_ROOT_DIR}  PATH_SUFFIXES ${OpenCV_LIBDIR_SUFFIXES} 
  NO_CMAKE_SYSTEM_PATH )
DBG_MSG("OpenCV_MLd_LIBRARY=${OpenCV_MLd_LIBRARY}")
find_library(OpenCV_TRSd_LIBRARY  
  NAMES trs
  PATHS ${OpenCV_ROOT_DIR}  PATH_SUFFIXES ${OpenCV_LIBDIR_SUFFIXES} 
  NO_CMAKE_SYSTEM_PATH )
DBG_MSG("OpenCV_TRSd_LIBRARY=${OpenCV_TRSd_LIBRARY}")



#
# Logic selecting required libs and headers
#
set(OpenCV_FOUND ON)
DBG_MSG("OpenCV_FIND_REQUIRED_COMPONENTS=${OpenCV_FIND_REQUIRED_COMPONENTS}")
foreach(NAME ${OpenCV_FIND_REQUIRED_COMPONENTS} )

  # only good if header and library both found   
  if    (OpenCV_${NAME}_INCLUDE_DIR AND OpenCV_${NAME}_LIBRARY)
    list(APPEND OpenCV_INCLUDE_DIRS ${OpenCV_${NAME}_INCLUDE_DIR} )
	if (EXISTS "${OpenCV_${NAME}_LIBRARY}" AND EXISTS "${OpenCV_${NAME}d_LIBRARY}")
		#message(STATUS "DEBUG ***************  ${OpenCV_${NAME}d_LIBRARY}")
		list(APPEND OpenCV_LIBRARIES    debug;${OpenCV_${NAME}d_LIBRARY};optimized;${OpenCV_${NAME}_LIBRARY} )
	else(EXISTS "${OpenCV_${NAME}_LIBRARY}" AND EXISTS "${OpenCV_${NAME}d_LIBRARY}")
		list(APPEND OpenCV_LIBRARIES    ${OpenCV_${NAME}_LIBRARY} )
	endif(EXISTS "${OpenCV_${NAME}_LIBRARY}" AND EXISTS "${OpenCV_${NAME}d_LIBRARY}")
    DBG_MSG("appending for NAME=${NAME} ${OpenCV_${NAME}_INCLUDE_DIR} and ${OpenCV_${NAME}_LIBRARY}" )
  else  (OpenCV_${NAME}_INCLUDE_DIR AND OpenCV_${NAME}_LIBRARY)
    DBG_MSG("OpenCV component NAME=${NAME} not found! "
      "\nOpenCV_${NAME}_INCLUDE_DIR=${OpenCV_${NAME}_INCLUDE_DIR} "
      "\nOpenCV_${NAME}_LIBRARY=${OpenCV_${NAME}_LIBRARY} ")
    set(OpenCV_FOUND OFF)
  endif (OpenCV_${NAME}_INCLUDE_DIR AND OpenCV_${NAME}_LIBRARY)
  
endforeach(NAME)

DBG_MSG("OpenCV_INCLUDE_DIRS=${OpenCV_INCLUDE_DIRS}")
DBG_MSG("OpenCV_LIBRARIES=${OpenCV_LIBRARIES}")

# get the link directory for rpath to be used with LINK_DIRECTORIES: 
if (OpenCV_CV_LIBRARY)
  get_filename_component(OpenCV_LINK_DIRECTORIES ${OpenCV_CV_LIBRARY} PATH)
endif (OpenCV_CV_LIBRARY)

mark_as_advanced(
  OpenCV_ROOT_DIR
  OpenCV_INCLUDE_DIRS
  OpenCV_CV_INCLUDE_DIR
  OpenCV_CXCORE_INCLUDE_DIR
  OpenCV_CVAUX_INCLUDE_DIR
  OpenCV_CVCAM_INCLUDE_DIR
  OpenCV_HIGHGUI_INCLUDE_DIR
  OpenCV_LIBRARIES
  OpenCV_CV_LIBRARY
  OpenCV_CXCORE_LIBRARY
  OpenCV_CVAUX_LIBRARY
  OpenCV_CVCAM_LIBRARY
  OpenCV_CVHAARTRAINING_LIBRARY
  OpenCV_CXTS_LIBRARY
  OpenCV_HIGHGUI_LIBRARY
  OpenCV_ML_LIBRARY
  OpenCV_TRS_LIBRARY
  OpenCV_CVd_LIBRARY
  OpenCV_CXCOREd_LIBRARY
  OpenCV_CVAUXd_LIBRARY
  OpenCV_CVCAMd_LIBRARY
  OpenCV_CVHAARTRAININGd_LIBRARY
  OpenCV_CXTSd_LIBRARY
  OpenCV_HIGHGUId_LIBRARY
  OpenCV_MLd_LIBRARY
  OpenCV_TRSd_LIBRARY
  )


# be backward compatible:
set(OPENCV_LIBRARIES   ${OpenCV_LIBRARIES} )
set(OPENCV_INCLUDE_DIR ${OpenCV_INCLUDE_DIRS} )
set(OPENCV_FOUND       ${OpenCV_FOUND})



# display help message
if(NOT OpenCV_FOUND)
  # make FIND_PACKAGE friendly
  if(NOT OpenCV_FIND_QUIETLY)
    if(OpenCV_FIND_REQUIRED)
      message(FATAL_ERROR
        "OpenCV required but some headers or libs not found. Please specify it's location with OpenCV_ROOT_DIR env. variable.")
    else(OpenCV_FIND_REQUIRED)
      message(STATUS 
        "ERROR: OpenCV was not found.")
    endif(OpenCV_FIND_REQUIRED)
  endif(NOT OpenCV_FIND_QUIETLY)
endif(NOT OpenCV_FOUND)

# make FIND_PACKAGE case sensitive compatible
set(OpenCV_FOUND       ${OPENCV_FOUND})
set(OpenCV_LIBRARIES   ${OPENCV_LIBRARIES})
set(OpenCV_INCLUDE_DIR ${OPENCV_INCLUDE_DIR})
