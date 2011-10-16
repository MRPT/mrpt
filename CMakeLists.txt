# ----------------------------------------------------------------------------
# Root CMake file for the MRPT libraries and applications
#
#  Run with "cmake ." at the root directory to build the makefiles for
#   the MRPT C++ library, the samples, and the applications. Some scripts
#   for generating the documentation, etc... are also updated.
#
#  For compiling instructions for all compilers and platforms, see
#   http://www.mrpt.org/Building_and_Installing_Instructions
#
#  2007-2011, Jose Luis Blanco <jlblanco@ctima.uma.es>
#
#  NOTE: CMake can be obtained at http://www.cmake.org/
# ----------------------------------------------------------------------------

PROJECT(MRPT)
if(MSVC)
	CMAKE_MINIMUM_REQUIRED(VERSION 2.6.4)
else(MSVC)
	CMAKE_MINIMUM_REQUIRED(VERSION 2.6)
endif(MSVC)

# --------------------------------------------------------------
# Indicate CMake 2.7 and above that we don't want to mix relative
#  and absolute paths in linker lib lists.
# Run "cmake --help-policy CMP0003" for more information.
# --------------------------------------------------------------
if(COMMAND cmake_policy)
      cmake_policy(SET CMP0003 NEW)
endif(COMMAND cmake_policy)


# ----------------------------------------------------------------------------
#  Loads the current version number:
# ----------------------------------------------------------------------------
FILE(READ "${CMAKE_CURRENT_SOURCE_DIR}/version_prefix.txt" CMAKE_MRPT_VERSION_NUMBER)

# For example: "0.5.1"
STRING(REGEX MATCHALL "[0-9]+" CMAKE_MRPT_VERSION_PARTS "${CMAKE_MRPT_VERSION_NUMBER}")

LIST(GET CMAKE_MRPT_VERSION_PARTS 0 CMAKE_MRPT_VERSION_NUMBER_MAJOR)
LIST(GET CMAKE_MRPT_VERSION_PARTS 1 CMAKE_MRPT_VERSION_NUMBER_MINOR)
LIST(GET CMAKE_MRPT_VERSION_PARTS 2 CMAKE_MRPT_VERSION_NUMBER_PATCH)
SET(CMAKE_MRPT_FULL_VERSION "${CMAKE_MRPT_VERSION_NUMBER_MAJOR}.${CMAKE_MRPT_VERSION_NUMBER_MINOR}.${CMAKE_MRPT_VERSION_NUMBER_PATCH}")

IF(WIN32)
	SET(MRPT_DLL_VERSION_POSTFIX "${CMAKE_MRPT_VERSION_NUMBER_MAJOR}${CMAKE_MRPT_VERSION_NUMBER_MINOR}${CMAKE_MRPT_VERSION_NUMBER_PATCH}")
ELSE(WIN32)
	SET(MRPT_DLL_VERSION_POSTFIX "")
ENDIF(WIN32)

# ----------------------------------------------------------------------------
# The root directory for all MRPT libraries/modules:
# ----------------------------------------------------------------------------
SET(MRPT_LIBS_ROOT "${CMAKE_SOURCE_DIR}/libs" CACHE INTERNAL "")  # This emulates global vars

LIST(APPEND CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmakemodules")

# Useful macros:
INCLUDE(cmakemodules/UtilsMacros.cmake REQUIRED)

# Detect machine architecture, on UNIX:
if(UNIX)
	execute_process(COMMAND uname -m
		OUTPUT_VARIABLE CMAKE_MRPT_ARCH
                OUTPUT_STRIP_TRAILING_WHITESPACE)
	message(STATUS "Architecture uname -m: ${CMAKE_MRPT_ARCH}")
endif(UNIX)

# ----------------------------------------------------------------------------
# Detect GNU version:
# ----------------------------------------------------------------------------
IF(CMAKE_COMPILER_IS_GNUCXX)
	execute_process(COMMAND ${CMAKE_CXX_COMPILER} --version
		          OUTPUT_VARIABLE CMAKE_MRPT_GCC_VERSION_FULL
		          OUTPUT_STRIP_TRAILING_WHITESPACE)

	# Typical output in CMAKE_MRPT_GCC_VERSION_FULL: "c++ (whatever) 4.2.3 (...)"
	#  Look for the version number
	STRING(REGEX MATCH "[0-9]+.[0-9]+.[0-9]+" CMAKE_GCC_REGEX_VERSION "${CMAKE_MRPT_GCC_VERSION_FULL}")
	#MESSAGE(STATUS "match: ${CMAKE_GCC_REGEX_VERSION}")

	# Split the three parts:
	STRING(REGEX MATCHALL "[0-9]+" CMAKE_MRPT_GCC_VERSIONS "${CMAKE_GCC_REGEX_VERSION}")

	LIST(GET CMAKE_MRPT_GCC_VERSIONS 0 CMAKE_MRPT_GCC_VERSION_MAJOR)
	LIST(GET CMAKE_MRPT_GCC_VERSIONS 1 CMAKE_MRPT_GCC_VERSION_MINOR)

	SET(CMAKE_MRPT_GCC_VERSION ${CMAKE_MRPT_GCC_VERSION_MAJOR}${CMAKE_MRPT_GCC_VERSION_MINOR})
ENDIF(CMAKE_COMPILER_IS_GNUCXX)


# ----------------------------------------------------------------------------
# Include the "CPack" package generator
# ----------------------------------------------------------------------------
SET(CMAKE_INSTALL_DEBUG_LIBRARIES 1)
INCLUDE(InstallRequiredSystemLibraries)

SET(CPACK_PACKAGE_DESCRIPTION_SUMMARY "The Mobile Robot Programming Toolkit (MRPT)")
SET(CPACK_PACKAGE_VENDOR "Jose Luis Blanco Claraco")
SET(CPACK_PACKAGE_CONTACT "Jose Luis Blanco Claraco <jlblanco@ctima.uma.es>")

SET(CPACK_PACKAGE_DESCRIPTION_FILE "${CMAKE_CURRENT_SOURCE_DIR}/README.txt")
SET(CPACK_RESOURCE_FILE_WELCOME "${CMAKE_CURRENT_SOURCE_DIR}/README.txt")
SET(CPACK_PACKAGE_DESCRIPTION_SUMMARY "MRPT is a set of C++ libraries and applications for mobile robot software development.")
FILE(READ ${CPACK_PACKAGE_DESCRIPTION_FILE} CPACK_DESCRIPTION_TEXT)

SET(CPACK_PACKAGE_VERSION_MAJOR "${CMAKE_MRPT_VERSION_NUMBER_MAJOR}")
SET(CPACK_PACKAGE_VERSION_MINOR "${CMAKE_MRPT_VERSION_NUMBER_MINOR}")
SET(CPACK_PACKAGE_VERSION_PATCH "${CMAKE_MRPT_VERSION_NUMBER_PATCH}")

SET(CPACK_RESOURCE_FILE_LICENSE "${CMAKE_CURRENT_SOURCE_DIR}/COPYING")
SET(CPACK_RESOURCE_FILE_README "${CMAKE_CURRENT_SOURCE_DIR}/README.txt")

SET(CPACK_SOURCE_GENERATOR "TGZ")

SET(CPACK_PACKAGE_INSTALL_DIRECTORY "mrpt-${CMAKE_MRPT_VERSION_NUMBER_MAJOR}.${CMAKE_MRPT_VERSION_NUMBER_MINOR}.${CMAKE_MRPT_VERSION_NUMBER_PATCH}")

IF(WIN32)
	# --------------------------------
	# Packages for Windows
	# --------------------------------
	SET(CPACK_SOURCE_IGNORE_FILES ".svn/;.*~;build;CMakeCache.txt;_CPack_Pakages/;CMakeFiles/;install/;Makefile;*.cmake")

	# There is a bug in NSI that does not handle full unix paths properly. Make
	# sure there is at least one set of four (4) backlasshes.
	SET(CPACK_NSIS_MUI_ICON "${CMAKE_CURRENT_SOURCE_DIR}/share/pixmaps\\\\mrpt_icon.ico")
	SET(CPACK_NSIS_MUI_UNIICON "${CMAKE_CURRENT_SOURCE_DIR}/share/pixmaps\\\\mrpt_icon.ico")
	SET(CPACK_PACKAGE_ICON "${CMAKE_CURRENT_SOURCE_DIR}/apps/wx-common\\\\mrpt_logo.png")

	SET(CPACK_NSIS_INSTALLED_ICON_NAME "${CMAKE_CURRENT_SOURCE_DIR}/share/pixmaps\\\\mrpt_icon.ico")

	SET(CPACK_NSIS_HELP_LINK "http:\\\\\\\\mrpt.sourceforge.net")
	SET(CPACK_NSIS_URL_INFO_ABOUT "http:\\\\\\\\www.isa.uma.es\\\\jlblanco")
	SET(CPACK_NSIS_CONTACT "jlblanco@ctima.uma.es")

	# Add mrpt/bin dir to system PATH
	SET(CPACK_NSIS_MODIFY_PATH ON)

	# Install header and source files:
	# ---------------------------------------------
	INSTALL(DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/libs/"
		DESTINATION libs
		PATTERN ".svn" EXCLUDE
		PATTERN "*~" EXCLUDE)

	INSTALL(DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/apps/"
		DESTINATION apps
		PATTERN ".svn" EXCLUDE
		PATTERN "*~" EXCLUDE)

	INSTALL(FILES
		cmake_uninstall.cmake.in
		CMakeLists.txt
		config_file_examples.txt
		configure
		COPYING
		DATASETS.txt
		INSTALL_COMPILE.html
		MRPTConfig.cmake.in
		NEWS.html
		README
		README.Debian
		README.txt
		version_prefix.txt
		version_prefix_README.txt
	DESTINATION .)

	SET(CPACK_PACKAGE_EXECUTABLES
		"camera-calib;Camera calibration GUI;features-matching;Visual feature matching;GridmapNavSimul;Gridmap simulator;RawLogViewer;RawLog Viewer;ReactiveNavigationDemo;Reactive navigation demo;prrt-navigator-demo;PRRT robot navigator demo;SceneViewer3D;Scene Viewer 3D;2d-slam-demo;2D KF-SLAM Simulator;navlog-viewer;Viewer of navigation log files;kinect-3d-slam;Simple live 3D-SLAM with Kinect")

	IF (EXISTS "${MRPT_SOURCE_DIR}/apps/hmt-slam-gui")
		SET(CPACK_PACKAGE_EXECUTABLES "${CPACK_PACKAGE_EXECUTABLES};hmt-slam-gui;HMT-SLAM visual interface")
	ENDIF (EXISTS "${MRPT_SOURCE_DIR}/apps/hmt-slam-gui")


	SET(CPACK_NSIS_MENU_LINKS
	    "doc;Documentation directory;bin;Directory of executables (bin);doc/chm/libMRPT-@CMAKE_MRPT_VERSION_NUMBER_MAJOR@.@CMAKE_MRPT_VERSION_NUMBER_MINOR@.@CMAKE_MRPT_VERSION_NUMBER_PATCH@.chm;MRPT libraries reference (CHM);README.txt;Read me;http://www.mrpt.org/;Online help;doc/mrpt-book.pdf;The MRPT book (PDF)")

	# File types association:
	SET(CPACK_NSIS_DEFINES "
		!include registerExtension.nsh
		")

	SET(CPACK_NSIS_EXTRA_INSTALL_COMMANDS "
		\\\${registerExtension} \\\"$INSTDIR\\\\bin\\\\RawLogViewer.exe\\\" \\\".rawlog\\\" \\\"Robotic Dataset File\\\"
		\\\${registerExtension} \\\"$INSTDIR\\\\bin\\\\SceneViewer3D.exe\\\" \\\".3Dscene\\\" \\\"Robotic 3D scene\\\"
		")
	SET(CPACK_NSIS_EXTRA_UNINSTALL_COMMANDS "
		\\\${unregisterExtension} \\\".rawlog\\\" \\\"Robotic Dataset File\\\"
		\\\${unregisterExtension} \\\".3Dscene\\\" \\\"Robotic 3D scene\\\"
		")

	# Install to "Program files (x86)" or "Program files" correctly:
	if(CMAKE_SIZEOF_VOID_P EQUAL 8)
	  set(CPACK_NSIS_PROGRAMFILES PROGRAMFILES64)
	else(CMAKE_SIZEOF_VOID_P EQUAL 8)
	  set(CPACK_NSIS_PROGRAMFILES PROGRAMFILES)
	endif(CMAKE_SIZEOF_VOID_P EQUAL 8)

ENDIF(WIN32)
IF(UNIX)
	# ------------------------------------------------------------------
	# Packages for linux: Not supported, use scritps/prepare_* instead
	# ------------------------------------------------------------------
ENDIF(UNIX)

IF(APPLE)
	SET(CPACK_GENERATOR "TGZ;TBZ2;OSXX11")
Endif(APPLE)

INCLUDE(CPack)

# ------------------------------------------
# The debug post-fix of .dll /.so libs
# ------------------------------------------
set(CMAKE_DEBUG_POSTFIX  "-dbg")
# Libs are: "libmrpt-xxx"
#            ---
IF(MSVC)
	set(MRPT_LIB_PREFIX "lib")
ENDIF(MSVC)

# In case of Makefiles if the user does not setup CMAKE_BUILD_TYPE, assume it's Release:
if (${CMAKE_GENERATOR} MATCHES ".*Makefiles")
    if("${CMAKE_BUILD_TYPE}" STREQUAL "")
        set(CMAKE_BUILD_TYPE Release)
    endif()
endif()


# ----------------------------------------------------------------------------
# 		CHECK FOR SYSTEM LIBRARIES, OPTIONS, ETC..
# ----------------------------------------------------------------------------

# Build (or not) some apps:
# ===================================================
SET(BUILD_MONOSLAM OFF CACHE BOOL "Build library mrpt-monoslam")
SET(BUILD_STEREOSLAM OFF CACHE BOOL "Build library mrpt-stereoslam")
MARK_AS_ADVANCED(BUILD_MONOSLAM)
MARK_AS_ADVANCED(BUILD_STEREOSLAM )

# If some app is not in this package, do NOT build it:
# =====================================================
IF (NOT EXISTS "${MRPT_SOURCE_DIR}/libs/monoslam/src")
	SET(BUILD_MONOSLAM OFF CACHE INTERNAL "" FORCE)
ENDIF (NOT EXISTS "${MRPT_SOURCE_DIR}/libs/monoslam/src")

IF (NOT EXISTS "${MRPT_SOURCE_DIR}/libs/stereoslam/src")
	SET(BUILD_STEREOSLAM OFF CACHE INTERNAL "" FORCE)
ENDIF (NOT EXISTS "${MRPT_SOURCE_DIR}/libs/stereoslam/src")


# Build static or dynamic libs?
# ===================================================
# Default: dynamic libraries:
SET(BUILD_SHARED_LIBS ON CACHE BOOL "Build shared libraries (.dll/.so) instead of static ones (.lib/.a)")
IF(BUILD_SHARED_LIBS)
	SET(CMAKE_MRPT_BUILD_SHARED_LIB "#define MRPT_BUILT_AS_DLL")
	SET(CMAKE_MRPT_BUILD_SHARED_LIB_ONOFF 1)
ELSE(BUILD_SHARED_LIBS)
	SET(CMAKE_MRPT_BUILD_SHARED_LIB "/* #define MRPT_BUILT_AS_DLL */")
	SET(CMAKE_MRPT_BUILD_SHARED_LIB_ONOFF 0)
ENDIF(BUILD_SHARED_LIBS)

# Only for Unix: use pkg-config to find libraries
# ===================================================
INCLUDE(FindPkgConfig OPTIONAL)
IF(PKG_CONFIG_FOUND)
	INCLUDE(FindPkgConfig)
ENDIF(PKG_CONFIG_FOUND)


# Build the ARIA library
# ===================================================
SET(BUILD_ARIA ON CACHE BOOL "Build an embedded version of ActivMedia ARIA library (interface to mobile robots)")
#MARK_AS_ADVANCED(BUILD_ARIA)
IF(BUILD_ARIA)
	SET(CMAKE_MRPT_HAS_ARIA 1)
ELSE(BUILD_ARIA)
	SET(CMAKE_MRPT_HAS_ARIA 0)
ENDIF(BUILD_ARIA)

# Build the xSENS library
# ===================================================
SET(BUILD_XSENS ON CACHE BOOL "Build an embedded version of xSens library (interface to xSens MTi device)")
#MARK_AS_ADVANCED(BUILD_XSENS)

IF(BUILD_XSENS)
	SET(CMAKE_MRPT_HAS_xSENS 1)
ELSE(BUILD_XSENS)
	SET(CMAKE_MRPT_HAS_xSENS 0)
ENDIF(BUILD_XSENS)


# Enable GCC profiling (GCC only)
# ===================================================
IF(CMAKE_COMPILER_IS_GNUCXX)
	SET(MRPT_ENABLE_PROFILING OFF CACHE BOOL "Enable profiling in the GCC compiler (Add flags: -g -pg)")
ENDIF(CMAKE_COMPILER_IS_GNUCXX)

# Enable precompiled headers:
# ===================================================
IF(MSVC)
	SET(DEFAULT_VAL_PRECOMP_HDR ON)
ELSE(MSVC)
	SET(DEFAULT_VAL_PRECOMP_HDR OFF)
ENDIF(MSVC)

SET(MRPT_ENABLE_PRECOMPILED_HDRS ${DEFAULT_VAL_PRECOMP_HDR} CACHE BOOL "Enable precompiled headers (thru 'mrpt/LIB.h')")
MARK_AS_ADVANCED(MRPT_ENABLE_PRECOMPILED_HDRS)

# Eigen library plugins:
# ===================================================
SET(EIGEN_MATRIXBASE_PLUGIN "<mrpt/math/eigen_plugins.h>" CACHE STRING "Eigen plugin header")
SET(EIGEN_MATRIXBASE_PLUGIN_POST_IMPL "<mrpt/math/eigen_plugins_impl.h>" CACHE STRING "Eigen plugin implementation header")
SET(EIGEN_EMBEDDED_INCLUDE_DIR "${MRPT_SOURCE_DIR}/otherlibs/eigen3/" CACHE PATH "Eigen path for embedded use")

MARK_AS_ADVANCED(EIGEN_MATRIXBASE_PLUGIN)
MARK_AS_ADVANCED(EIGEN_MATRIXBASE_PLUGIN_POST_IMPL)
MARK_AS_ADVANCED(EIGEN_EMBEDDED_INCLUDE_DIR)

# By default: Use system version if pkg-config says it exists:
SET(DEFAULT_EIGEN_USE_EMBEDDED_VERSION ON)
IF(PKG_CONFIG_FOUND)
	PKG_CHECK_MODULES(PKG_EIGEN3 eigen3)	# Find eigen3 with pkg-config:
	IF(PKG_EIGEN3_FOUND)
		SET(DEFAULT_EIGEN_USE_EMBEDDED_VERSION OFF)
	ENDIF(PKG_EIGEN3_FOUND)
ENDIF(PKG_CONFIG_FOUND)

SET(EIGEN_USE_EMBEDDED_VERSION ${DEFAULT_EIGEN_USE_EMBEDDED_VERSION} CACHE BOOL "Use embedded Eigen3 version or system version")
IF (EIGEN_USE_EMBEDDED_VERSION)
	# Include embedded version headers:
	SET(MRPT_EIGEN_INCLUDE_DIR "${EIGEN_EMBEDDED_INCLUDE_DIR}")
ELSE(EIGEN_USE_EMBEDDED_VERSION)
	# Find Eigen headers in the system:
	IF(NOT PKG_CONFIG_FOUND)
		MESSAGE(SEND_ERROR "pkg-config is required for this operation!")
	ELSE(NOT PKG_CONFIG_FOUND)
		# Find eigen3 with pkg-config:
		PKG_CHECK_MODULES(PKG_EIGEN3 eigen3)
		IF(PKG_EIGEN3_FOUND)
			SET(MRPT_EIGEN_INCLUDE_DIR "${PKG_EIGEN3_INCLUDE_DIRS}")
		ELSE(PKG_EIGEN3_FOUND)
			MESSAGE(SEND_ERROR "pkg-config was unable to find eigen3: install libeigen3-dev or enable EIGEN_USE_EMBEDDED_VERSION")
		ENDIF(PKG_EIGEN3_FOUND)
	ENDIF(NOT PKG_CONFIG_FOUND)
ENDIF(EIGEN_USE_EMBEDDED_VERSION)

INCLUDE_DIRECTORIES("${MRPT_EIGEN_INCLUDE_DIR}")
IF(EXISTS "${MRPT_EIGEN_INCLUDE_DIR}/unsupported/")
	INCLUDE_DIRECTORIES("${MRPT_EIGEN_INCLUDE_DIR}/unsupported/")
ENDIF(EXISTS "${MRPT_EIGEN_INCLUDE_DIR}/unsupported/")

# Create variables just for the final summary of the configuration (see bottom of this file):
SET(CMAKE_MRPT_HAS_EIGEN 1)        # Always, it's a fundamental dep.!

# Create numeric (0/1) variable EIGEN_USE_EMBEDDED_VERSION_BOOL for the .cmake.in file:
IF(EIGEN_USE_EMBEDDED_VERSION)
	SET(EIGEN_USE_EMBEDDED_VERSION_BOOL 1)
	SET(CMAKE_MRPT_HAS_EIGEN_SYSTEM 0)
ELSE(EIGEN_USE_EMBEDDED_VERSION)
	SET(EIGEN_USE_EMBEDDED_VERSION_BOOL 0)
	SET(CMAKE_MRPT_HAS_EIGEN_SYSTEM 1)
ENDIF(EIGEN_USE_EMBEDDED_VERSION)




# MRPT_TRY_START/END blocks
# ===================================================
SET(MRPT_HAS_STACKED_EXCEPTIONS ON CACHE BOOL "Enable MRPT_TRY_START/END blocks (disable it for speed up).")

# MRPT embedded profiler is enabled only in Debug:
# ===================================================
SET(MRPT_ENABLE_EMBEDDED_GLOBAL_PROFILER OFF CACHE BOOL "Enable MRPT embedded profiler.")

# ASSERT_ blocks
# ===================================================
SET(MRPT_HAS_ASSERT ON CACHE BOOL "Enable ASSERT_ statements (disable it for speed up).")


# Enable libstdc++ parallel mode  (GCC only)
# ===================================================
IF(CMAKE_COMPILER_IS_GNUCXX)
	SET(MRPT_ENABLE_LIBSTD_PARALLEL_MODE OFF CACHE BOOL "Enable parallel mode in libstdc++ (requires GCC 4.2.2+)")
ENDIF(CMAKE_COMPILER_IS_GNUCXX)

# Enable Intel Threading Building Blocks (TBB) lib
# ===================================================
SET(MRPT_HAS_TBB OFF CACHE BOOL "Enable Intel TBB threading library for parallelization.")

SET(CMAKE_MRPT_HAS_TBB  0)

# The following code block comes from OpenCV (BSD)
if (MRPT_HAS_TBB)
    if (UNIX AND NOT APPLE AND NOT ANDROID)
        PKG_CHECK_MODULES(TBB tbb)
        message(STATUS "TBB detected: ${TBBLIB_FOUND}")

        if (TBB_FOUND)
            set(CMAKE_MRPT_HAS_TBB 1)
            if(NOT ${TBB_INCLUDE_DIRS} STREQUAL "")
                include_directories(${TBB_INCLUDE_DIRS})
            endif(NOT ${TBB_INCLUDE_DIRS} STREQUAL "")
            link_directories(${TBB_LIBRARY_DIRS})
            APPEND_MRPT_LIBS(${TBB_LIBRARIES})
        endif(TBB_FOUND)
    endif(UNIX AND NOT APPLE AND NOT ANDROID)

    if (NOT CMAKE_MRPT_HAS_TBB)
        set(TBB_DEFAULT_INCLUDE_DIRS "/opt/intel/tbb" "/usr/local/include" "/usr/include" "C:/Program Files/Intel/TBB" "C:/Program Files (x86)/Intel/TBB" "C:/Program Files (x86)/TBB" "${CMAKE_INSTALL_PREFIX}/include")

        find_path(TBB_INCLUDE_DIR "tbb/tbb.h" PATHS ${TBB_DEFAULT_INCLUDE_DIRS} DOC "The path to TBB headers")
        if (TBB_INCLUDE_DIR)
            if (UNIX)
                set(TBB_LIB_DIR "${TBB_INCLUDE_DIR}/../lib" CACHE PATH "Full path of TBB library directory")
                link_directories("${TBB_LIB_DIR}")
            endif(UNIX)
            if (APPLE)
                APPEND_MRPT_LIBS(libtbb.dylib)
            elseif (ANDROID)
                APPEND_MRPT_LIBS(tbbmalloc tbb)
                add_definitions(-DTBB_USE_GCC_BUILTINS)
            elseif (UNIX)
                APPEND_MRPT_LIBS(tbb)
            elseif (WIN32)
                if (CMAKE_COMPILER_IS_GNUCXX)
                    set(TBB_LIB_DIR "${TBB_INCLUDE_DIR}/../lib" CACHE PATH "Full path of TBB library directory")
                    link_directories("${TBB_LIB_DIR}")
                    APPEND_MRPT_LIBS(tbb)
                else(CMAKE_COMPILER_IS_GNUCXX)
                    get_filename_component(_TBB_LIB_PATH "${TBB_INCLUDE_DIR}/../lib" ABSOLUTE)

                    if(${CMAKE_SYSTEM_PROCESSOR} MATCHES amd64*|x86_64*)
                        set(_TBB_LIB_PATH "${_TBB_LIB_PATH}/intel64")
                    endif(${CMAKE_SYSTEM_PROCESSOR} MATCHES amd64*|x86_64*)
                    if(${CMAKE_SYSTEM_PROCESSOR} MATCHES x86*|i386*|i686*)
                        set(_TBB_LIB_PATH "${_TBB_LIB_PATH}/ia32")
                    endif(${CMAKE_SYSTEM_PROCESSOR} MATCHES x86*|i386*|i686*)

                    if (MSVC80)
                       set(_TBB_LIB_PATH "${_TBB_LIB_PATH}/vc8")
                    elseif(MSVC90)
                       set(_TBB_LIB_PATH "${_TBB_LIB_PATH}/vc9")
                    elseif(MSVC10)
                        set(_TBB_LIB_PATH "${_TBB_LIB_PATH}/vc10")
                    endif(MSVC80)
                    set(TBB_LIB_DIR "${_TBB_LIB_PATH}" CACHE PATH "Full path of TBB library directory")
                    link_directories("${TBB_LIB_DIR}")
                endif(CMAKE_COMPILER_IS_GNUCXX)
            endif(UNIX)

            set(CMAKE_MRPT_HAS_TBB 1)
            if(NOT "${TBB_INCLUDE_DIR}" STREQUAL "")
                include_directories("${TBB_INCLUDE_DIR}")
            endif(NOT "${TBB_INCLUDE_DIR}" STREQUAL "")
        endif()
    endif(NOT CMAKE_MRPT_HAS_TBB)
endif(MRPT_HAS_TBB)


# Occupancy grid cellsize in bits: 8 or 16
# ===================================================
SET(MRPT_OCCUPANCY_GRID_CELLSIZE 8 CACHE STRING "Cell size for mrpt::slam::COccupancyGridMap2D (either 8 or 16 bits)")
IF(NOT MRPT_OCCUPANCY_GRID_CELLSIZE EQUAL 8 AND NOT MRPT_OCCUPANCY_GRID_CELLSIZE EQUAL 16)
	MESSAGE("MRPT_OCCUPANCY_GRID_CELLSIZE can have the values 8 or 16 only.")
ENDIF(NOT MRPT_OCCUPANCY_GRID_CELLSIZE EQUAL 8 AND NOT MRPT_OCCUPANCY_GRID_CELLSIZE EQUAL 16)


# Support for Bumblebee stereo camera (actually: PGR Digiclops/Triclops Windows libraries)
# ==========================================================================================
IF(UNIX)
	SET( MRPT_HAS_BUMBLEBEE OFF CACHE INTERNAL "Has PGR Digiclops/Triclops Windows libraries?" FORCE)
ELSE(UNIX)
	SET( MRPT_HAS_BUMBLEBEE OFF CACHE BOOL "Has PGR Digiclops/Triclops Windows libraries?")

	IF(MRPT_HAS_BUMBLEBEE)
		SET( BUMBLEBEE_PGR_FLYCAPTURE_ROOT_DIR "" CACHE PATH "Path to PGR FlyCapture library root directory")
		SET( BUMBLEBEE_TRICLOPS_ROOT_DIR "" CACHE PATH "Path to triclops library root directory")

		IF(UNIX)
			MESSAGE("Sorry! Bumblebee camera is supported only for Windows yet. Set MRPT_HAS_BUMBLEBEE to OFF")
			SET(CMAKE_MRPT_HAS_BUMBLEBEE 0)
		ELSE(UNIX)
			# Set to 1, next check for missing things and set to 0 on any error & report message:
			SET(CMAKE_MRPT_HAS_BUMBLEBEE 1)

			IF(NOT EXISTS ${BUMBLEBEE_PGR_FLYCAPTURE_ROOT_DIR})
				SET(CMAKE_MRPT_HAS_BUMBLEBEE 0)
				MESSAGE("The directory 'BUMBLEBEE_PGR_FLYCAPTURE_ROOT_DIR' does not exists. Turn off BUMBLEBEE support or provide the correct path.")
			ENDIF(NOT EXISTS ${BUMBLEBEE_PGR_FLYCAPTURE_ROOT_DIR})

			IF(NOT EXISTS ${BUMBLEBEE_PGR_FLYCAPTURE_ROOT_DIR}/include/PGRFlyCapture.h)
				SET(CMAKE_MRPT_HAS_BUMBLEBEE 0)
				MESSAGE("The directory 'BUMBLEBEE_PGR_FLYCAPTURE_ROOT_DIR' does not contain include/PGRFlyCapture.h. Turn off BUMBLEBEE support or provide the correct path.")
			ENDIF(NOT EXISTS ${BUMBLEBEE_PGR_FLYCAPTURE_ROOT_DIR}/include/PGRFlyCapture.h)

			IF(NOT EXISTS ${BUMBLEBEE_PGR_FLYCAPTURE_ROOT_DIR}/lib/PGRFlyCapture.lib)
				SET(CMAKE_MRPT_HAS_BUMBLEBEE 0)
				MESSAGE("The directory 'BUMBLEBEE_PGR_FLYCAPTURE_ROOT_DIR' does not contain lib/PGRFlyCapture.LIB. Turn off BUMBLEBEE support or provide the correct path.")
			ENDIF(NOT EXISTS ${BUMBLEBEE_PGR_FLYCAPTURE_ROOT_DIR}/lib/PGRFlyCapture.lib)

			# ----

			IF(NOT EXISTS ${BUMBLEBEE_TRICLOPS_ROOT_DIR})
				SET(CMAKE_MRPT_HAS_BUMBLEBEE 0)
				MESSAGE("The directory 'BUMBLEBEE_TRICLOPS_ROOT_DIR' does not exists. Turn off BUMBLEBEE support or provide the correct path.")
			ENDIF(NOT EXISTS ${BUMBLEBEE_TRICLOPS_ROOT_DIR})

			IF(NOT EXISTS ${BUMBLEBEE_TRICLOPS_ROOT_DIR}/include/triclops.h)
				SET(CMAKE_MRPT_HAS_BUMBLEBEE 0)
				MESSAGE("The directory 'BUMBLEBEE_TRICLOPS_ROOT_DIR' does not contain include/triclops.h. Turn off BUMBLEBEE support or provide the correct path.")
			ENDIF(NOT EXISTS ${BUMBLEBEE_TRICLOPS_ROOT_DIR}/include/triclops.h)

			IF(NOT EXISTS ${BUMBLEBEE_TRICLOPS_ROOT_DIR}/lib/triclops.lib)
				SET(CMAKE_MRPT_HAS_BUMBLEBEE 0)
				MESSAGE("The directory 'BUMBLEBEE_TRICLOPS_ROOT_DIR' does not contain lib/triclops.lib. Turn off BUMBLEBEE support or provide the correct path.")
			ENDIF(NOT EXISTS ${BUMBLEBEE_TRICLOPS_ROOT_DIR}/lib/triclops.lib)
		ENDIF(UNIX)

	ENDIF(MRPT_HAS_BUMBLEBEE)
ENDIF(UNIX)

# Support for phidget Interface Kit with proximity sensor device :
# ===================================================
SET( MRPT_HAS_PHIDGET OFF CACHE BOOL "Add supprt for Phidget Interface kit with range sensors")
IF(MRPT_HAS_PHIDGET)
	SET(CMAKE_MRPT_HAS_PHIDGET 1)
	SET(PHIDGET_ROOT_DIR "/usr/local" CACHE PATH "Path where libPhidget21 was installed, if phidget21.h is in /usr/local/include, set this variable to /usr/local.")
	IF(UNIX)
		IF(NOT EXISTS ${PHIDGET_ROOT_DIR}/include/phidget21.h)
			SET(CMAKE_MRPT_HAS_PHIDGET 0)
			MESSAGE("Can't find incude file phidget21.h, please set a correct PHIDGET_ROOT_DIR")
		ENDIF(NOT EXISTS ${PHIDGET_ROOT_DIR}/include/phidget21.h)
		IF(NOT EXISTS ${PHIDGET_ROOT_DIR}/lib/libphidget21.so)
			SET(CMAKE_MRPT_HAS_PHIDGET 0)
			MESSAGE("Can't find incude file libphidget21.so, please set a correct PHIDGET_ROOT_DIR")
		ENDIF(NOT EXISTS ${PHIDGET_ROOT_DIR}/lib/libphidget21.so)
	ELSE(UNIX)
		SET(CMAKE_MRPT_HAS_PHIDGET 0)
		MESSAGE("The MRPT Phidget library interface is only supported on linux, please turn off MRPT_HAS_PHIDGET")
	ENDIF(UNIX)
ENDIF(MRPT_HAS_PHIDGET)

# Support for Videre Design stereo camera:
# ===================================================
SET( MRPT_HAS_SVS OFF CACHE BOOL "Add support for STOC Stereo Camera?")

IF(MRPT_HAS_SVS)
	SET( SVS_ROOT_DIR "" CACHE PATH "Path to SVS library root directory")


	IF(UNIX)
		# Set to 1, next check for missing things and set to 0 on any error & report message:
		SET(CMAKE_MRPT_HAS_SVS 1)

		IF(NOT EXISTS ${SVS_ROOT_DIR})
			SET(CMAKE_MRPT_HAS_SVS 0)
			MESSAGE("The directory 'SVS_ROOT_DIR' does not exists. Turn off SVS support or provide the correct path.")
		ENDIF(NOT EXISTS ${SVS_ROOT_DIR})

		IF(NOT EXISTS ${SVS_ROOT_DIR}/src/svsclass.h)
			SET(CMAKE_MRPT_HAS_SVS 0)
			MESSAGE("The directory 'SVS_ROOT_DIR' does not contain src/svsclass.h. Turn off SVS support or provide the correct path.")
		ENDIF(NOT EXISTS ${SVS_ROOT_DIR}/src/svsclass.h)

		IF(NOT EXISTS ${SVS_ROOT_DIR}/src/dcs.h)
			SET(CMAKE_MRPT_HAS_SVS 0)
			MESSAGE("The directory 'SVS_ROOT_DIR' does not contain src/dcs.h. Turn off SVS support or provide the correct path.")
		ENDIF(NOT EXISTS ${SVS_ROOT_DIR}/src/dcs.h)
	ELSE(UNIX)


		MESSAGE("Sorry! STOC camera is supported only for LINUX yet. Set MRPT_HAS_SVS to OFF")
		SET(CMAKE_MRPT_HAS_SVS 0)


	ENDIF(UNIX)

ENDIF(MRPT_HAS_SVS)

#   FabMap module for HMT-SLAM? (optional)
# =======================================
FIND_PACKAGE(FabMap QUIET)
IF(NOT FabMap_DIR)
	MARK_AS_ADVANCED(FabMap_DIR)
ENDIF(NOT FabMap_DIR)

IF(FabMap_FOUND)
	SET(CMAKE_MRPT_HAS_FABMAP  1)
ELSE(FabMap_FOUND)
	SET(CMAKE_MRPT_HAS_FABMAP  0)
ENDIF(FabMap_FOUND)


# Check for STXXL: (Not yet)
# =====================================
SET(CMAKE_MRPT_HAS_STXXL 0)

# Check for the FTDI headers (Linux only, in win32
#  we use built-in header & dynamic DLL load):
# ===================================================
IF(UNIX)
	FIND_FILE(FTDI_CONFIG_FILE libftdi-config)
	IF(FTDI_CONFIG_FILE)
		MARK_AS_ADVANCED(FTDI_CONFIG_FILE)

		SET(CMAKE_MRPT_HAS_FTDI 1)
		SET(CMAKE_MRPT_HAS_FTDI_SYSTEM 1)

		# Get the config params:
		EXECUTE_PROCESS(COMMAND ${FTDI_CONFIG_FILE} --libs
			RESULT_VARIABLE CMAKE_FTDI_CONFIG_RES
			OUTPUT_VARIABLE CMAKE_FTDI_LIBS
			OUTPUT_STRIP_TRAILING_WHITESPACE
			)
		IF(${CMAKE_FTDI_CONFIG_RES})
			MESSAGE("Error invoking FTDI config file:\n ${FTDI_CONFIG_FILE} --libs")
		ENDIF(${CMAKE_FTDI_CONFIG_RES})

		EXECUTE_PROCESS(COMMAND ${FTDI_CONFIG_FILE} --cflags
			RESULT_VARIABLE CMAKE_FTDI_CONFIG_RES
			OUTPUT_VARIABLE CMAKE_FTDI_CFLAGS
			OUTPUT_STRIP_TRAILING_WHITESPACE
			)
		IF(${CMAKE_FTDI_CONFIG_RES})
			MESSAGE("Error invoking FTDI config file:\n ${FTDI_CONFIG_FILE} --cflags")
		ENDIF(${CMAKE_FTDI_CONFIG_RES})

		ADD_DEFINITIONS(${CMAKE_FTDI_CFLAGS})

	ELSE(FTDI_CONFIG_FILE)
		SET(CMAKE_MRPT_HAS_FTDI 0)
	ENDIF(FTDI_CONFIG_FILE)
	# This option will be available only on Linux, hence it's declared here:
	OPTION(DISABLE_FTDI "Do not use the USB driver for FTDI chips" 0)
	MARK_AS_ADVANCED(DISABLE_FTDI)
ELSE(UNIX)
	# In windows we always have FTDI support (at compile time at least...)
	SET(CMAKE_MRPT_HAS_FTDI 1)
ENDIF(UNIX)

# Check for the sys/inotify headers (Linux only, in win32
#  we use the equivalent API for file system monitoring):
# =======================================================
IF(UNIX)
	FIND_FILE(INOTIFY_HEADER_FILE sys/inotify.h HINTS /usr/include/x86_64-linux-gnu/)
	IF(INOTIFY_HEADER_FILE)
		SET(CMAKE_MRPT_HAS_INOTIFY 1)
		MARK_AS_ADVANCED(INOTIFY_HEADER_FILE)
	ELSE(INOTIFY_HEADER_FILE)
		SET(CMAKE_MRPT_HAS_INOTIFY 0)
	ENDIF(INOTIFY_HEADER_FILE)
ELSE(UNIX)
	# In windows there is no INOTIFY api.
	SET(CMAKE_MRPT_HAS_INOTIFY 0)
ENDIF(UNIX)

# Check for the GL,GLUT libraries (Linux only, in
#  Windows use the build in .h & .lib):
# ===================================================
FIND_PACKAGE(OpenGL)
IF(OPENGL_FOUND AND "${OPENGL_INCLUDE_DIR}")
	INCLUDE_DIRECTORIES("${OPENGL_INCLUDE_DIR}")
ENDIF(OPENGL_FOUND AND "${OPENGL_INCLUDE_DIR}")

IF(UNIX)
	FIND_PACKAGE(GLUT)

	IF (GLUT_FOUND AND OPENGL_gl_LIBRARY AND OPENGL_glu_LIBRARY AND GLUT_glut_LIBRARY)
		INCLUDE_DIRECTORIES("${GLUT_INCLUDE_DIR}")
		SET(CMAKE_MRPT_HAS_OPENGL_GLUT 1)
		SET(CMAKE_MRPT_HAS_GLUT_SYSTEM 1)
	ELSE(GLUT_FOUND AND OPENGL_gl_LIBRARY AND OPENGL_glu_LIBRARY AND GLUT_glut_LIBRARY)
		MESSAGE(STATUS "**Warning**: OpenGL and/or GLUT not found! OpenGL capabilities will be disabled.")
		MESSAGE(STATUS "              	OPENGL_gl_LIBRARY: ${OPENGL_gl_LIBRARY}")
		MESSAGE(STATUS "              	OPENGL_glu_LIBRARY: ${OPENGL_glu_LIBRARY}")
		MESSAGE(STATUS "              	GLUT_glut_LIBRARY: ${GLUT_glut_LIBRARY}")
		MESSAGE(STATUS "              	OPENGL_INCLUDE_DIR: ${OPENGL_INCLUDE_DIR}")

		SET(CMAKE_MRPT_HAS_OPENGL_GLUT 0)
		SET(CMAKE_MRPT_HAS_GLUT_SYSTEM 0)
	ENDIF(GLUT_FOUND AND OPENGL_gl_LIBRARY AND OPENGL_glu_LIBRARY AND GLUT_glut_LIBRARY)
ELSE(UNIX)
	SET(CMAKE_MRPT_HAS_OPENGL_GLUT 1)
	SET(CMAKE_MRPT_HAS_GLUT_SYSTEM 0)
ENDIF(UNIX)


# Check for the OpenCV libraries:
#  pkg-config if available (Linux), otherwise CMake module
# =========================================================
SET(CMAKE_MRPT_HAS_OPENCV 0)
SET(MRPT_OPENCV_VERSION 0.0.0)
SET(MRPT_OPENCV_VERSION_HEX "0x000")
SET(MRPT_OPENCV_SRC_DIR "")
SET(OpenCV_IGNORE_PKGCONFIG OFF CACHE BOOL "Forces using OpenCVConfig.cmake to find OpenCV")
MARK_AS_ADVANCED(OpenCV_IGNORE_PKGCONFIG)

# Invoke pkg-config for getting the configuration:
IF(PKG_CONFIG_FOUND AND NOT OpenCV_IGNORE_PKGCONFIG)
	PKG_CHECK_MODULES(OPENCV opencv)
	IF(OPENCV_FOUND)
		SET(CMAKE_MRPT_HAS_OPENCV 1)
		SET(MRPT_OPENCV_VERSION ${OPENCV_VERSION})

		IF ("${OPENCV_INCLUDEDIR}")
			INCLUDE_DIRECTORIES(${OPENCV_INCLUDEDIR})
		ENDIF("${OPENCV_INCLUDEDIR}")
		IF ("${OPENCV_LIBDIR}")
			LINK_DIRECTORIES(${OPENCV_LIBDIR})
		ENDIF ("${OPENCV_LIBDIR}")
		SET(OpenCV_LIBRARIES ${OPENCV_LIBRARIES})

		MESSAGE(STATUS " opencv include: ${OPENCV_INCLUDE_DIRS} (Version: ${OPENCV_VERSION})")
		INCLUDE_DIRECTORIES(${OPENCV_INCLUDE_DIRS})
	 	SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}")  # ${OPENCV_CFLAGS}: JL (Jul2010) seems not needed and causes problems... -I/dirs already done with line above.

#		SET(MRPT_LINKER_LIBS_FOR_PKGCONFIG "${MRPT_LINKER_LIBS_FOR_PKGCONFIG} ${opencv_LDFLAGS}")
	ENDIF(OPENCV_FOUND)
ENDIF(PKG_CONFIG_FOUND AND NOT OpenCV_IGNORE_PKGCONFIG)

# Use CMAKE module if opencv's not been detected yet:
IF(NOT CMAKE_MRPT_HAS_OPENCV)
	# 1st: Try to find OpenCV config file (NO_MODULE: Don't find a module, but OpenCVConfig.cmake):
	FIND_PACKAGE(OpenCV  QUIET NO_MODULE)
	IF(OpenCV_FOUND)
		SET(MRPT_OPENCV_VERSION ${OpenCV_VERSION})
		SET(OpenCV_LIBRARIES ${OpenCV_LIBS})
		SET(OPENCV_LIBDIR ${OpenCV_LIB_DIR})
		if (NOT "${BASEDIR}" STREQUAL "")
			SET(MRPT_OPENCV_SRC_DIR "${BASEDIR}")
		endif (NOT "${BASEDIR}" STREQUAL "")
		MESSAGE(STATUS "OpenCV ${OpenCV_VERSION} found through OpenCVConfig.cmake")

		SET(CMAKE_MRPT_HAS_OPENCV 1)
	ENDIF(OpenCV_FOUND)
ENDIF(NOT CMAKE_MRPT_HAS_OPENCV)


IF(NOT CMAKE_MRPT_HAS_OPENCV)
	# 2nd: OK, let's use the module:
	FIND_PACKAGE(OpenCV)
	IF(OpenCV_FOUND)
		# MRPT_OPENCV_VERSION
		IF($ENV{VERBOSE})
			MESSAGE(STATUS "OPENCV_EXE_LINKER_FLAGS: ${OPENCV_EXE_LINKER_FLAGS}")
			MESSAGE(STATUS "OPENCV_INCLUDE_DIR: ${OPENCV_INCLUDE_DIR}")
			MESSAGE(STATUS "OpenCV_LIBRARIES: ${OpenCV_LIBRARIES}")
		ENDIF($ENV{VERBOSE})

		FILE(GLOB_RECURSE CV_VER_H "${OpenCV_CXCORE_INCLUDE_DIR}/cvver.h")
		file(READ "${CV_VER_H}" STR_CV_VERSION)

		# Extract the CV version from the cvver.h file, lines "#define CV_MAJOR_VERSION  XX", etc...

		#STRING(REGEX MATCHALL "[0-9]+.[0-9]+.[0-9]+" MRPT_OPENCV_VERSION "${STR_CV_VERSION}")
		STRING(REGEX MATCH "CV_MAJOR_VERSION[ ]+[0-9]+" CMAKE_OPENCV_VERSION_NUMBER_MAJOR "${STR_CV_VERSION}")
		STRING(REGEX MATCH "[0-9]+" CMAKE_OPENCV_VERSION_NUMBER_MAJOR "${CMAKE_OPENCV_VERSION_NUMBER_MAJOR}")

		STRING(REGEX MATCH "CV_MINOR_VERSION[ ]+[0-9]+" CMAKE_OPENCV_VERSION_NUMBER_MINOR "${STR_CV_VERSION}")
		STRING(REGEX MATCH "[0-9]+" CMAKE_OPENCV_VERSION_NUMBER_MINOR "${CMAKE_OPENCV_VERSION_NUMBER_MINOR}")

		STRING(REGEX MATCH "CV_SUBMINOR_VERSION[ ]+[0-9]+" CMAKE_OPENCV_VERSION_NUMBER_PATCH "${STR_CV_VERSION}")
		STRING(REGEX MATCH "[0-9]+" CMAKE_OPENCV_VERSION_NUMBER_PATCH "${CMAKE_OPENCV_VERSION_NUMBER_PATCH}")

		SET(MRPT_OPENCV_VERSION "${CMAKE_OPENCV_VERSION_NUMBER_MAJOR}.${CMAKE_OPENCV_VERSION_NUMBER_MINOR}.${CMAKE_OPENCV_VERSION_NUMBER_PATCH}")

		MESSAGE(STATUS "OpenCV version detected: ${MRPT_OPENCV_VERSION}")

		SET(CMAKE_MRPT_HAS_OPENCV 1)
	ENDIF(OpenCV_FOUND)
ENDIF(NOT CMAKE_MRPT_HAS_OPENCV)


# Opencv version as Hex. number:
STRING(REGEX MATCHALL "[0-9]+" CMAKE_OPENCV_VERSION_PARTS "${MRPT_OPENCV_VERSION}")
LIST(GET CMAKE_OPENCV_VERSION_PARTS 0 CMAKE_OPENCV_VERSION_NUMBER_MAJOR)
LIST(GET CMAKE_OPENCV_VERSION_PARTS 1 CMAKE_OPENCV_VERSION_NUMBER_MINOR)
LIST(GET CMAKE_OPENCV_VERSION_PARTS 2 CMAKE_OPENCV_VERSION_NUMBER_PATCH)
SET(MRPT_OPENCV_VERSION_HEX "0x${CMAKE_OPENCV_VERSION_NUMBER_MAJOR}${CMAKE_OPENCV_VERSION_NUMBER_MINOR}${CMAKE_OPENCV_VERSION_NUMBER_PATCH}")


# Check for ffmpeg C libraries: libavcodec, libavutil, libavformat, libswscale
#  These libs are linked against mrpt-hwdrivers only (in shared libs,
#  in static all user apps will have to link against this)
# ====================================================================
SET(CMAKE_MRPT_HAS_FFMPEG 0)
SET(CMAKE_MRPT_HAS_FFMPEG_SYSTEM 0)
SET(MRPT_FFMPEG_LIBS_TO_LINK "")

IF(PKG_CONFIG_FOUND)
	PKG_CHECK_MODULES(LIBAVCODEC  libavcodec)
	PKG_CHECK_MODULES(LIBAVUTIL   libavutil)
	PKG_CHECK_MODULES(LIBAVFORMAT libavformat)
	PKG_CHECK_MODULES(LIBSWSCALE  libswscale)
	IF(LIBAVCODEC_FOUND AND LIBAVUTIL_FOUND AND LIBAVFORMAT_FOUND AND LIBSWSCALE_FOUND)
		SET(CMAKE_MRPT_HAS_FFMPEG 1)
		SET(CMAKE_MRPT_HAS_FFMPEG_SYSTEM 1)

		INCLUDE_DIRECTORIES("${LIBAVCODEC_INCLUDEDIR}/")
		INCLUDE_DIRECTORIES("${LIBAVCODEC_INCLUDEDIR}/ffmpeg")
		INCLUDE_DIRECTORIES("${LIBAVCODEC_INCLUDEDIR}/libavcodec")

		INCLUDE_DIRECTORIES("${LIBAVUTIL_INCLUDEDIR}")

		INCLUDE_DIRECTORIES("${LIBAVFORMAT_INCLUDEDIR}")
		INCLUDE_DIRECTORIES("${LIBAVFORMAT_INCLUDEDIR}/ffmpeg")
		INCLUDE_DIRECTORIES("${LIBAVFORMAT_INCLUDEDIR}/libavformat")

		INCLUDE_DIRECTORIES("${LIBSWSCALE_INCLUDEDIR}")
		INCLUDE_DIRECTORIES("${LIBSWSCALE_INCLUDEDIR}/ffmpeg")
		INCLUDE_DIRECTORIES("${LIBSWSCALE_INCLUDEDIR}/libswscale")

		LINK_DIRECTORIES("${LIBAVCODEC_LIBDIR}")
		LINK_DIRECTORIES("${LIBAVUTIL_LIBDIR}")
		LINK_DIRECTORIES("${LIBAVFORMAT_LIBDIR}")
		LINK_DIRECTORIES("${LIBSWSCALE_LIBDIR}")

		SET(MRPT_FFMPEG_LIBS_TO_LINK "${MRPT_FFMPEG_LIBS_TO_LINK}" "${LIBAVCODEC_LIBRARIES}" "${LIBAVFORMAT_LIBRARIES}" "${LIBAVUTIL_LIBRARIES}" "${LIBSWSCALE_LIBRARIES}")

		IF($ENV{VERBOSE})
			MESSAGE(STATUS " ffmpeg libs: ${MRPT_FFMPEG_LIBS_TO_LINK}")
		ENDIF($ENV{VERBOSE})
	ENDIF(LIBAVCODEC_FOUND AND LIBAVUTIL_FOUND AND LIBAVFORMAT_FOUND AND LIBSWSCALE_FOUND)
ENDIF(PKG_CONFIG_FOUND)

IF(MSVC)
	SET( MRPT_HAS_FFMPEG_WIN32 OFF CACHE BOOL "Add support for IP cameras and all FFmpeg-capable video sources")
ENDIF(MSVC)

IF(MRPT_HAS_FFMPEG_WIN32)
	SET( FFMPEG_WIN32_ROOT_DIR "" CACHE PATH "Path to FFmpeg win32 build root directory (See http://ffmpeg.arrozcru.org/builds/)")

	# Set to 1, next check for missing things and set to 0 on any error & report message:
	SET(CMAKE_MRPT_HAS_FFMPEG 1)
	SET(CMAKE_MRPT_HAS_FFMPEG_SYSTEM 1)

	IF(NOT EXISTS "${FFMPEG_WIN32_ROOT_DIR}")
		SET(CMAKE_MRPT_HAS_FFMPEG 0)
		SET(CMAKE_MRPT_HAS_FFMPEG_SYSTEM 0)
		MESSAGE("The directory 'FFMPEG_WIN32_ROOT_DIR' does not exists. Turn off FFmpeg support or provide the correct path.")
	ENDIF(NOT EXISTS "${FFMPEG_WIN32_ROOT_DIR}")

	IF(NOT EXISTS "${FFMPEG_WIN32_ROOT_DIR}/include/libavcodec" OR NOT EXISTS "${FFMPEG_WIN32_ROOT_DIR}/include/libavformat" OR NOT EXISTS "${FFMPEG_WIN32_ROOT_DIR}/include/libavutil" OR NOT EXISTS "${FFMPEG_WIN32_ROOT_DIR}/include/libswscale")
		SET(CMAKE_MRPT_HAS_FFMPEG 0)
		SET(CMAKE_MRPT_HAS_FFMPEG_SYSTEM 0)
		MESSAGE("The directory 'FFMPEG_WIN32_ROOT_DIR' does not contain include/{libavcodec,libavformat,libavutil,libswscale}. Turn off FFmpeg support or provide the correct path.")
	ENDIF(NOT EXISTS "${FFMPEG_WIN32_ROOT_DIR}/include/libavcodec" OR NOT EXISTS "${FFMPEG_WIN32_ROOT_DIR}/include/libavformat" OR NOT EXISTS "${FFMPEG_WIN32_ROOT_DIR}/include/libavutil" OR NOT EXISTS "${FFMPEG_WIN32_ROOT_DIR}/include/libswscale")

	IF(NOT EXISTS "${FFMPEG_WIN32_ROOT_DIR}/lib")
		SET(CMAKE_MRPT_HAS_FFMPEG 0)
		SET(CMAKE_MRPT_HAS_FFMPEG_SYSTEM 0)
		MESSAGE("The directory 'FFMPEG_WIN32_ROOT_DIR' does not contain /lib/. Turn off FFmpeg support or provide the correct path.")
	ENDIF(NOT EXISTS "${FFMPEG_WIN32_ROOT_DIR}/lib")

	# We need the .lib files: avcodec-52.lib, avformat-52.lib, avutil-49.lib, swscale-0.lib
	FILE(GLOB FFMPEG_WIN32_AVCODEC_LIB "${FFMPEG_WIN32_ROOT_DIR}/lib/avcodec*.lib")
	FILE(GLOB FFMPEG_WIN32_AVUTIL_LIB "${FFMPEG_WIN32_ROOT_DIR}/lib/avutil*.lib")
	FILE(GLOB FFMPEG_WIN32_AVFORMAT_LIB "${FFMPEG_WIN32_ROOT_DIR}/lib/avformat*.lib")
	FILE(GLOB FFMPEG_WIN32_SWSCALE_LIB "${FFMPEG_WIN32_ROOT_DIR}/lib/swscale*.lib")

	IF (NOT EXISTS ${FFMPEG_WIN32_AVCODEC_LIB})
		SET(CMAKE_MRPT_HAS_FFMPEG 0)
		SET(CMAKE_MRPT_HAS_FFMPEG_SYSTEM 0)
		MESSAGE("avcodec-XX.lib not found under '${FFMPEG_WIN32_ROOT_DIR}/lib'. Turn off FFmpeg support or provide the correct path.")
	ENDIF (NOT EXISTS ${FFMPEG_WIN32_AVCODEC_LIB})

	IF (NOT EXISTS ${FFMPEG_WIN32_AVUTIL_LIB})
		SET(CMAKE_MRPT_HAS_FFMPEG 0)
		SET(CMAKE_MRPT_HAS_FFMPEG_SYSTEM 0)
		MESSAGE("avutil-XX.lib not found under '${FFMPEG_WIN32_ROOT_DIR}/lib'. Turn off FFmpeg support or provide the correct path.")
	ENDIF (NOT EXISTS ${FFMPEG_WIN32_AVUTIL_LIB})

	IF (NOT EXISTS ${FFMPEG_WIN32_AVFORMAT_LIB})
		SET(CMAKE_MRPT_HAS_FFMPEG 0)
		SET(CMAKE_MRPT_HAS_FFMPEG_SYSTEM 0)
		MESSAGE("avformat-XX.lib not found under '${FFMPEG_WIN32_ROOT_DIR}/lib'. Turn off FFmpeg support or provide the correct path.")
	ENDIF (NOT EXISTS ${FFMPEG_WIN32_AVFORMAT_LIB})

	IF (NOT EXISTS ${FFMPEG_WIN32_SWSCALE_LIB})
		SET(CMAKE_MRPT_HAS_FFMPEG 0)
		SET(CMAKE_MRPT_HAS_FFMPEG_SYSTEM 0)
		MESSAGE("swscale-XX.lib not found under '${FFMPEG_WIN32_ROOT_DIR}/lib'. Turn off FFmpeg support or provide the correct path.")
	ENDIF (NOT EXISTS ${FFMPEG_WIN32_SWSCALE_LIB})

	INCLUDE_DIRECTORIES("${FFMPEG_WIN32_ROOT_DIR}/include")
	INCLUDE_DIRECTORIES("${FFMPEG_WIN32_ROOT_DIR}/include/libavcodec")
	INCLUDE_DIRECTORIES("${FFMPEG_WIN32_ROOT_DIR}/include/libavformat")
	INCLUDE_DIRECTORIES("${FFMPEG_WIN32_ROOT_DIR}/include/libavutil")
	INCLUDE_DIRECTORIES("${FFMPEG_WIN32_ROOT_DIR}/include/libswscale")

	SET(MRPT_FFMPEG_LIBS_TO_LINK ${MRPT_FFMPEG_LIBS_TO_LINK} "${FFMPEG_WIN32_AVCODEC_LIB}" "${FFMPEG_WIN32_AVUTIL_LIB}" "${FFMPEG_WIN32_AVFORMAT_LIB}" "${FFMPEG_WIN32_SWSCALE_LIB}")
ENDIF(MRPT_HAS_FFMPEG_WIN32)


# Check for system lib3ds:
# ===================================================
SET(CMAKE_MRPT_HAS_LIB3DS 1)	# Always present: system or built-in

# Check for system zlib:
# ===================================================
SET(CMAKE_MRPT_HAS_ZLIB 1)	# Always present: system or built-in

# Check for system jpeglib:
# ===================================================
SET(CMAKE_MRPT_HAS_JPEG 1)	# Always present: system or built-in

# Check for wxWidgets + GL
#  If it is found, will set CMAKE_MRPT_HAS_WXWIDGETS=1
# ===================================================
# Here you can define what libraries of wxWidgets you need for your application.
#  You can figure out what libraries you need here;  http://www.wxwidgets.org/manuals/2.8/wx_librarieslist.html
SET(wxWidgets_MRPT_COMPONENTS_TO_SEARCH "base;core;gl;adv;aui" CACHE STRING "Components to search in wxWidgets")

IF(UNIX)
	# Linux:
	IF(CMAKE_BUILD_TYPE MATCHES "Debug")
		SET(wxWidgets_USE_DEBUG ON)
	ENDIF(CMAKE_BUILD_TYPE MATCHES "Debug")
ELSE(UNIX)
	# Windows configuration of wxWidgets:
	SET(wxWidgets_USE_REL_AND_DBG ON)
ENDIF(UNIX)

# Make things easier: If we are building for MSVC, select the vc_lib or vc_dll directory automatically:
if(msvc)
	if(BUILD_SHARED_LIBS)
		set(wxWidgets_LIB_DIR "${wxWidgets_ROOT_DIR}/lib/vc_dll" FORCE)
	else(BUILD_SHARED_LIBS)
		set(wxWidgets_LIB_DIR "${wxWidgets_ROOT_DIR}/lib/vc_lib" FORCE)
	endif(BUILD_SHARED_LIBS)
endif(msvc)


# We need the Find package for wxWidgets to work
FIND_PACKAGE(wxWidgets COMPONENTS ${wxWidgets_MRPT_COMPONENTS_TO_SEARCH})
# Did we find wxWidgets ? This condition will fail for as long as the internal vars do not point to the proper wxWidgets configuration
IF(wxWidgets_FOUND)
	# Check for non-found debug libs:
	IF(UNIX)
		IF(CMAKE_BUILD_TYPE MATCHES "Debug")
			IF(NOT wxWidgets_USE_DEBUG)
				MESSAGE("Warning: The debug libraries for wxWidgets couldn't be found by CMake. Please install them (libwxbase2.8-dev) or build in release.")
			ENDIF(NOT wxWidgets_USE_DEBUG)
		ENDIF(CMAKE_BUILD_TYPE MATCHES "Debug")
	ENDIF(UNIX)

	# Include wxWidgets macros
	INCLUDE(${wxWidgets_USE_FILE})
	# ${wxWidgets_LIBRARIES}  will contain the libraries that should be added through TARGET_LINK_LIBRARIES(...)
	IF($ENV{VERBOSE})
		MESSAGE(STATUS "wxWidgets:")
		MESSAGE(STATUS "WX: ${wxWidgets_LIBRARIES}")
		MESSAGE(STATUS "   wxWidgets_LIBRARY_DIRS  : ${wxWidgets_LIBRARY_DIRS}")
		MESSAGE(STATUS "   wxWidgets_CXX_FLAGS     : ${wxWidgets_CXX_FLAGS}")
		MESSAGE(STATUS "   wxWidgets_INCLUDE_DIRS  : ${wxWidgets_INCLUDE_DIRS}")
	ENDIF($ENV{VERBOSE})

	LINK_DIRECTORIES(${wxWidgets_LIBRARY_DIRS})

	IF(MSVC)
		ADD_DEFINITIONS(-DwxUSE_NO_MANIFEST=1)
	ENDIF(MSVC)

	IF(CMAKE_COMPILER_IS_GNUCXX)
		# Add -isystem for wx headers (seems not to be added automatically for GCC in mac osx)
		FOREACH(WXDIR ${wxWidgets_INCLUDE_DIRS})
			SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -I${WXDIR} -isystem ${WXDIR}")
		ENDFOREACH(WXDIR)
	ENDIF(CMAKE_COMPILER_IS_GNUCXX)

	# For use in the MRPTconfig.cmake
	SET(CMAKE_MRPT_WX_ROOT_DIR ${wxWidgets_ROOT_DIR})

	SET(CMAKE_MRPT_HAS_WXWIDGETS 1)
ELSE(wxWidgets_FOUND)
	# Monolithic?
	IF (WX_mono)
		SET(wxWidgets_MRPT_COMPONENTS_TO_SEARCH mono CACHE STRING "Components to search in wxWidgets" FORCE)
		FIND_PACKAGE(wxWidgets COMPONENTS ${wxWidgets_MRPT_COMPONENTS_TO_SEARCH})

		INCLUDE(${wxWidgets_USE_FILE})
		# ${wxWidgets_LIBRARIES}  will contain the libraries that should be added through TARGET_LINK_LIBRARIES(...)
		#MESSAGE(STATUS "wxWidgets_LIBRARIES: ${wxWidgets_LIBRARIES}")
		#MESSAGE(STATUS "wxWidgets_CXX_FLAGS: ${wxWidgets_CXX_FLAGS}")

		LINK_DIRECTORIES(${wxWidgets_LIBRARY_DIRS})

		IF(MSVC)
			ADD_DEFINITIONS(-DwxUSE_NO_MANIFEST=1)
		ENDIF(MSVC)

		# For use in the MRPTconfig.cmake
		SET(CMAKE_MRPT_WX_ROOT_DIR ${wxWidgets_ROOT_DIR})

		SET(CMAKE_MRPT_HAS_WXWIDGETS 1)

	ELSE(WX_mono)
		MESSAGE("wxWidgets was not found automatically. Please, set wxWidgets_ROOT_DIR to the lib directory to enable it in MRPT.")
		SET(CMAKE_MRPT_HAS_WXWIDGETS 0)
	ENDIF(WX_mono)
ENDIF(wxWidgets_FOUND)


# Let the user disable SSE2
# ===================================================
SET(CMAKE_MRPT_HAS_SSE2 1)
SET(DISABLE_SSE2 OFF CACHE BOOL "Forces compilation WITHOUT SSE2/MMX extensions")
MARK_AS_ADVANCED(DISABLE_SSE2)
IF(DISABLE_SSE2)
	SET(CMAKE_MRPT_HAS_SSE2 0)
ENDIF(DISABLE_SSE2)

# Let the user disable SSE3
# ===================================================
SET(CMAKE_MRPT_HAS_SSE3 1)
SET(DISABLE_SSE3 OFF CACHE BOOL "Forces compilation WITHOUT SSE3 extensions")
MARK_AS_ADVANCED(DISABLE_SSE3)
IF(DISABLE_SSE3)
	SET(CMAKE_MRPT_HAS_SSE3 0)
ENDIF(DISABLE_SSE3)


# Detect support for OpenMP:
# ===================================================
#FIND_PACKAGE(OpenMP QUIET)
#IF (NOT OPENMP_FOUND)
#	SET(CMAKE_MRPT_HAS_OPENMP 0)
#ELSE(NOT OPENMP_FOUND)
#	# Let the user disable OpenMP
#	# ===================================================
#	SET(CMAKE_MRPT_HAS_OPENMP 1)
#	SET(DISABLE_OPENMP OFF CACHE BOOL "Forces compilation WITHOUT OpenMP")
#	MARK_AS_ADVANCED(DISABLE_OPENMP)
#	IF(DISABLE_OPENMP)
#		SET(CMAKE_MRPT_HAS_OPENMP 0)
#	ENDIF(DISABLE_OPENMP)
#ENDIF (NOT OPENMP_FOUND)

#IF(CMAKE_MRPT_HAS_OPENMP)
#	SET(CMAKE_C_FLAGS   "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
#	SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
#ENDIF(CMAKE_MRPT_HAS_OPENMP)


# DISABLE_WXWIDGETS
# ---------------------
SET( DISABLE_WXWIDGETS OFF CACHE BOOL "Forces compilation WITHOUT wxWidgets")
MARK_AS_ADVANCED(DISABLE_WXWIDGETS)
IF(DISABLE_WXWIDGETS)
	SET(CMAKE_MRPT_HAS_WXWIDGETS 0)
	SET(wxWidgets_LIBRARIES "")
ENDIF(DISABLE_WXWIDGETS)

# Check for libdc1394-2 (Only in Linux, there is no
#  Windows version yet...)
# ===================================================
SET(CMAKE_MRPT_HAS_LIBDC1394_2 0)
# Invoke pkg-config for getting the configuration:
IF(PKG_CONFIG_FOUND)
	PKG_CHECK_MODULES(LIBDC1394_2 libdc1394-2)
	IF (LIBDC1394_2_FOUND)
		SET(CMAKE_MRPT_HAS_LIBDC1394_2 1)
		#MESSAGE(STATUS "LIBDC1394_2_LIBRARIES    : ${LIBDC1394_2_LIBRARIES}")
		#MESSAGE(STATUS "LIBDC1394_2_INCLUDE_DIRS : ${LIBDC1394_2_INCLUDE_DIRS}")
		#MESSAGE(STATUS "LIBDC1394_2_LIBRARY_DIRS : ${LIBDC1394_2_LIBRARY_DIRS}")
		#MESSAGE(STATUS "LIBDC1394_2_LDFLAGS      : ${LIBDC1394_2_LDFLAGS}")


		IF ("${LIBDC1394_2_INCLUDE_DIRS}")
			INCLUDE_DIRECTORIES("${LIBDC1394_2_INCLUDE_DIRS}")
		ENDIF ("${LIBDC1394_2_INCLUDE_DIRS}")

		IF ("${LIBDC1394_2_LIBRARY_DIRS}")
			LINK_DIRECTORIES("${LIBDC1394_2_LIBRARY_DIRS}")
		ENDIF ("${LIBDC1394_2_LIBRARY_DIRS}")

#		SET(MRPT_LINKER_LIBS_FOR_PKGCONFIG "${MRPT_LINKER_LIBS_FOR_PKGCONFIG} ${LIBDC1394_2_LDFLAGS}")
	ELSE(LIBDC1394_2_FOUND)
		SET(CMAKE_MRPT_HAS_LIBDC1394_2 0)
	ENDIF (LIBDC1394_2_FOUND)
ENDIF(PKG_CONFIG_FOUND)


# ---------------------------------------------------------------------------
#			OPTIONS
#The options for the user when using "cmakesetup" or "ccmake":
# ---------------------------------------------------------------------------
OPTION(MRPT_ALWAYS_CHECKS_DEBUG "Additional checks even in Release" "OFF")
OPTION(MRPT_ALWAYS_CHECKS_DEBUG_MATRICES "Additional checks even in Release (Only in matrix classes)" "OFF")
OPTION(DISABLE_OPENCV "Disable the OpenCV library" "OFF")
MARK_AS_ADVANCED(DISABLE_OPENCV)


# ---------------------------------------------------------------------------
#  Parse the value of options to disable stuff:
# ---------------------------------------------------------------------------
IF(DISABLE_OPENCV)
	SET(CMAKE_MRPT_HAS_OPENCV 0)
ENDIF(DISABLE_OPENCV)


OPTION(DISABLE_OPENGL "Disable the OpenGL library" "OFF")
MARK_AS_ADVANCED(DISABLE_OPENGL)


# ---------------------------------------------------------------------------
#  Parse the value of options to disable stuff:
# ---------------------------------------------------------------------------
IF(DISABLE_OPENGL)
	SET(CMAKE_MRPT_HAS_OPENGL_GLUT 0)
ENDIF(DISABLE_OPENGL)


# By default, compile this library if the directory exist:
# It will be not present in Debian packages due to legal restrictions:
# ------------------------------------------------------------------------
IF(EXISTS "${MRPT_LIBS_ROOT}/vision/src/sift-hess")
	SET( CMAKE_MRPT_HAS_SIFT_HESS 1)
ELSE(EXISTS "${MRPT_LIBS_ROOT}/vision/src/sift-hess")
	SET( CMAKE_MRPT_HAS_SIFT_HESS 0)
ENDIF(EXISTS "${MRPT_LIBS_ROOT}/vision/src/sift-hess")

IF(NOT CMAKE_MRPT_HAS_OPENCV)
	SET( CMAKE_MRPT_HAS_SIFT_HESS 0)
ENDIF(NOT CMAKE_MRPT_HAS_OPENCV)

OPTION(DISABLE_SIFT_HESS "Disable the Hess' SIFT library" "OFF")
MARK_AS_ADVANCED(DISABLE_SIFT_HESS)
IF(DISABLE_SIFT_HESS)
	SET(CMAKE_MRPT_HAS_SIFT_HESS 0)
ENDIF(DISABLE_SIFT_HESS)



# ---------------------------------------------------------------------------
# Create the minimum list of libraries required by an application that
#  uses the MRPT C++ library. This must be passed to "TARGET_LINK_LIBRARIES"
# NOTE: For the compilers MSVC* & Borland most of the libraries are
#        automatically included through "#pragma"'s ;-)
# ---------------------------------------------------------------------------
IF(WIN32)
    # glut (for windows): Built-in
	INCLUDE_DIRECTORIES("${MRPT_SOURCE_DIR}/include/mrpt/otherlibs/glut/")
ENDIF(WIN32)

IF(MSVC OR BORLAND)
	SET(MRPT_LINKER_LIBS "")
	SET(MRPT_LINKER_LIBS_FOR_PKGCONFIG "")

	IF(CMAKE_MRPT_HAS_ZLIB)
		#INCLUDE_DIRECTORIES("${MRPT_SOURCE_DIR}/src/base/compress/zlib/")
		SET(CMAKE_MRPT_HAS_ZLIB_SYSTEM 0)
	ENDIF(CMAKE_MRPT_HAS_ZLIB)

	IF(CMAKE_MRPT_HAS_JPEG)
		SET(CMAKE_MRPT_HAS_JPEG_SYSTEM 0)
	ENDIF(CMAKE_MRPT_HAS_JPEG)

ELSE(MSVC OR BORLAND)

	# GL, GLU, glut:
	IF(CMAKE_MRPT_HAS_OPENGL_GLUT)
		APPEND_MRPT_LIBS(${OPENGL_gl_LIBRARY} ${OPENGL_glu_LIBRARY} ${GLUT_glut_LIBRARY})
	ENDIF(CMAKE_MRPT_HAS_OPENGL_GLUT)

	# FTDI:
	IF(CMAKE_MRPT_HAS_FTDI)
		IF (UNIX)
			# ${CMAKE_FTDI_LIBS}
			APPEND_MRPT_LIBS(ftdi usb)
		ELSE(UNIX)
			# Nothing...
		ENDIF(UNIX)
	ENDIF(CMAKE_MRPT_HAS_FTDI)

	# ZLIB:
	FIND_PACKAGE(ZLIB)
	IF(ZLIB_FOUND)
			#MESSAGE(STATUS "Found library: zlib - Include: ${ZLIB_INCLUDE_DIR}")
			INCLUDE_DIRECTORIES("${ZLIB_INCLUDE_DIR}")

			APPEND_MRPT_LIBS(z)

			SET(CMAKE_MRPT_HAS_ZLIB_SYSTEM 1)
	ELSE(ZLIB_FOUND)
			#INCLUDE_DIRECTORIES("${MRPT_SOURCE_DIR}/src/base/compress/zlib/")

			# If we are using a static version of wxWidgets we dont need this... for now check if this is MinGW on Windows...
			#IF (NOT WIN32 OR NOT CMAKE_MRPT_HAS_WXWIDGETS)
			#	APPEND_MRPT_LIBS(optimized mrpt-zlib debug mrpt-zlib-dbg)
			#ENDIF(NOT WIN32 OR NOT CMAKE_MRPT_HAS_WXWIDGETS)

			SET(CMAKE_MRPT_HAS_ZLIB_SYSTEM 0)
	ENDIF(ZLIB_FOUND)

	# JPEG:
	FIND_PACKAGE(JPEG)
	IF(JPEG_FOUND)
			#MESSAGE(STATUS "Found library: jpeg  - Include: ${JPEG_INCLUDE_DIR}")
			INCLUDE_DIRECTORIES("${JPEG_INCLUDE_DIR}")

			APPEND_MRPT_LIBS(jpeg)

			SET(CMAKE_MRPT_HAS_JPEG_SYSTEM 1)
	ELSE(JPEG_FOUND)
			SET(CMAKE_MRPT_HAS_JPEG_SYSTEM 0)
	ENDIF(JPEG_FOUND)

	# In Linux, link against librt
	IF(UNIX)
		# Jerome Monceaux : 2011/03/08
		# exclude rt from apple osx linking
		IF(NOT APPLE)
			APPEND_MRPT_LIBS(rt)
		ENDIF(NOT APPLE)
	ENDIF(UNIX)

	# In Windows, link against ole32 & uuid & oleaut32, etc...
	IF(MINGW)
		APPEND_MRPT_LIBS(ole32 uuid oleaut32 gdi32 winmm setupapi ws2_32)
		#ADD_DEFINITIONS(-D__MSVCRT_VERSION__=0x7000)
	ENDIF(MINGW)

ENDIF(MSVC OR BORLAND)

# OpenCV (all compilers):
IF(CMAKE_MRPT_HAS_OPENCV)
	INCLUDE_DIRECTORIES(${OpenCV_INCLUDE_DIR})
	APPEND_MRPT_LIBS(${OpenCV_LIBRARIES})

	IF($ENV{VERBOSE})
		MESSAGE(STATUS "OpenCV:")
		MESSAGE(STATUS "        OpenCV_LIBRARIES:   ${OpenCV_LIBRARIES}")
		MESSAGE(STATUS "        OpenCV_INCLUDE_DIR: ${OpenCV_INCLUDE_DIR}")
	ENDIF($ENV{VERBOSE})

	SET(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OPENCV_EXE_LINKER_FLAGS}")

	SET(CMAKE_MRPT_HAS_OPENCV_SYSTEM 1)

ENDIF(CMAKE_MRPT_HAS_OPENCV)

# libs for libdc1394:
IF (LIBDC1394_2_FOUND)
	APPEND_MRPT_LIBS( ${LIBDC1394_2_LIBRARIES} )
ENDIF (LIBDC1394_2_FOUND)


# Support for SWISSRANGE 3D camera:
# ===================================================
SET(CMAKE_MRPT_HAS_SWISSRANGE 0)

IF(UNIX)
	# Linux: Look for the libMesaSR.h include:
	FIND_FILE(SWR_HEADER_FILE  libMesaSR.h)
	IF(SWR_HEADER_FILE)
		SET(CMAKE_MRPT_HAS_SWISSRANGE 1)
		MARK_AS_ADVANCED(SWR_HEADER_FILE)
		APPEND_MRPT_LIBS(mesasr)
	ELSE(SWR_HEADER_FILE)
		SET(CMAKE_MRPT_HAS_SWISSRANGE 0)
	ENDIF(SWR_HEADER_FILE)
ELSE(UNIX)
	IF (MSVC)
		# Windows: ...
		FIND_PATH(SWR_LIBMESASR_DIR  MesaImaging/Swissranger/libMesaSR/)

		IF(SWR_LIBMESASR_DIR)
			SET(SWR_LIBMESASR_DIR "${SWR_LIBMESASR_DIR}/MesaImaging/Swissranger/libMesaSR/")
			MESSAGE(STATUS "SwissRanger Library found in: ${SWR_LIBMESASR_DIR}")

			# We expect to find there "libMesaSR.lib" & "libMesaSR.h"
			IF (EXISTS "${SWR_LIBMESASR_DIR}/libMesaSR.lib" AND EXISTS "${SWR_LIBMESASR_DIR}/libMesaSR.h")
				SET(CMAKE_MRPT_HAS_SWISSRANGE 1)
				INCLUDE_DIRECTORIES("${SWR_LIBMESASR_DIR}")
				#LINK_DIRECTORIES("${SWR_LIBMESASR_DIR}")
				APPEND_MRPT_LIBS("${SWR_LIBMESASR_DIR}/libMesaSR.lib")
				MARK_AS_ADVANCED(SWR_LIBMESASR_DIR)
			ELSE (EXISTS "${SWR_LIBMESASR_DIR}/libMesaSR.lib" AND EXISTS "${SWR_LIBMESASR_DIR}/libMesaSR.h")
				MESSAGE(STATUS "*** ERROR *** SwissRanger Library directory found but doesn't contain expected files. Not using it.")
				SET(CMAKE_MRPT_HAS_SWISSRANGE 0)
			ENDIF (EXISTS "${SWR_LIBMESASR_DIR}/libMesaSR.lib" AND EXISTS "${SWR_LIBMESASR_DIR}/libMesaSR.h")
		ELSE(SWR_LIBMESASR_DIR)
			SET(CMAKE_MRPT_HAS_SWISSRANGE 0)
		ENDIF(SWR_LIBMESASR_DIR)
	ENDIF(MSVC)
ENDIF(UNIX)

# Leave at the user's choice to disable the SWR libs:
OPTION(DISABLE_SWISSRANGER_3DCAM_LIBS "Disable the usage (if found) of SWR libs" "OFF")
MARK_AS_ADVANCED(DISABLE_SWISSRANGER_3DCAM_LIBS)
IF(DISABLE_SWISSRANGER_3DCAM_LIBS)
	SET(CMAKE_MRPT_HAS_SWISSRANGE 0)
ENDIF(DISABLE_SWISSRANGER_3DCAM_LIBS)

# We have gz-streams is we have zlib:
# ------------------------------------------
SET(CMAKE_MRPT_HAS_GZ_STREAMS ${CMAKE_MRPT_HAS_ZLIB})

# Look for the libusb1 lib
# ------------------------------
IF(UNIX)
	IF(PKG_CONFIG_FOUND)
        	PKG_CHECK_MODULES(PKG_LIBUSB10 libusb-1.0)
	ENDIF(PKG_CONFIG_FOUND)
ENDIF(UNIX)


# Build the XBox Kinect support (via libfreenect)
# ===================================================
# Deduce default ON/OFF state of Kinect support:
IF (UNIX)
	IF (PKG_LIBUSB10_FOUND)
		SET(BUILD_KINECT_DEFAULT ON)
	ELSE(PKG_LIBUSB10_FOUND)
		SET(BUILD_KINECT_DEFAULT OFF)
	ENDIF(PKG_LIBUSB10_FOUND)
ELSE(UNIX)
	SET(BUILD_KINECT_DEFAULT OFF)
ENDIF(UNIX)

SET(CMAKE_MRPT_HAS_KINECT 0)  # Will be set to 1 only if all conditions are OK
SET(CMAKE_MRPT_HAS_CL_NUI 0)
SET(CMAKE_MRPT_HAS_FREENECT 0)
SET(CMAKE_MRPT_HAS_FREENECT_SYSTEM 0) # This means libfreenect is already built somewhere else in the system.

SET(BUILD_KINECT ${BUILD_KINECT_DEFAULT} CACHE BOOL "Build support for Xbox Kinect")
IF(BUILD_KINECT)
	# Check deps on Linux/Windows!!!

	IF(UNIX)
		# Kinect for Linux: Embedded sources of libfreenect ---------
		IF(NOT PKG_CONFIG_FOUND)
			MESSAGE(SEND_ERROR "Kinect support: pkg-config is required! Please install it.")
		ELSE(NOT PKG_CONFIG_FOUND)
			IF(PKG_LIBUSB10_FOUND)
				SET(CMAKE_MRPT_HAS_KINECT 1)
				SET(CMAKE_MRPT_HAS_FREENECT 1)
				#MESSAGE(STATUS "PKG_LIBUSB10_LIBRARIES: ${PKG_LIBUSB10_LIBRARIES}")
				APPEND_MRPT_LIBS(${PKG_LIBUSB10_LIBRARIES})
			ELSE(PKG_LIBUSB10_FOUND)
				MESSAGE(SEND_ERROR "BUILD_KINECT requires libusb-1.0. Install it or disable BUILD_KINECT")
			ENDIF(PKG_LIBUSB10_FOUND)
		ENDIF(NOT PKG_CONFIG_FOUND)
	ELSEIF(WIN32)
		# Kinect for Win32: CL NUI or external libfreenect built by the user ---------

		SET(BUILD_KINECT_USE_CLNUI    OFF CACHE BOOL "Kinect using CL NUI SDK")
		SET(BUILD_KINECT_USE_FREENECT ON  CACHE BOOL "Kinect using OpenKinect's libfreenect")

		IF((NOT BUILD_KINECT_USE_FREENECT AND NOT BUILD_KINECT_USE_CLNUI) OR (BUILD_KINECT_USE_FREENECT AND BUILD_KINECT_USE_CLNUI))
			MESSAGE(SEND_ERROR
				"You must select the Kinect preferred SDK: OpenKinect freenect or CL NUI\n"
				"Mark just one from BUILD_KINECT_USE_CLNUI or BUILD_KINECT_USE_CLNUI, or disable BUILD_KINECT.\n"
				"For more info read: http://www.mrpt.org/Kinect_and_MRPT\n")
		ENDIF((NOT BUILD_KINECT_USE_FREENECT AND NOT BUILD_KINECT_USE_CLNUI) OR (BUILD_KINECT_USE_FREENECT AND BUILD_KINECT_USE_CLNUI))

		# With external Libfreenect:
		IF (BUILD_KINECT_USE_FREENECT)
			# Find packages needed to build library in Windows
			find_package(libusb-1.0 REQUIRED)

			if (LIBUSB_1_FOUND)
				SET(FREENECT_LIBS ${LIBUSB_1_LIBRARIES})

				SET(LIBUSB_1_DLL_FOR_INSTALL "" CACHE PATH "Path to the libusb0.dll file to install along MRPT packages")
				MARK_AS_ADVANCED(LIBUSB_1_DLL_FOR_INSTALL)

				# All OK:
				SET(CMAKE_MRPT_HAS_KINECT 1)
				SET(CMAKE_MRPT_HAS_FREENECT 1)
				SET(CMAKE_MRPT_HAS_FREENECT_SYSTEM 0)  # use embedded version
			else (LIBUSB_1_FOUND)
				# Error:
				MESSAGE(SEND_ERROR "*** ERROR *** Please, set libusb-1 variables or disable BUILD_KINECT.")
			endif (LIBUSB_1_FOUND)

		ENDIF (BUILD_KINECT_USE_FREENECT)

		# With the CL NUI SDK:
		IF (BUILD_KINECT_USE_CLNUI)
			FIND_PATH(BASE_CLNUI_SDK_DIR  "Code Laboratories/CL NUI Platform/SDK/")
			MARK_AS_ADVANCED(BASE_CLNUI_SDK_DIR)

			IF(BASE_CLNUI_SDK_DIR)
				SET(CLNUI_SDK_DIR "${BASE_CLNUI_SDK_DIR}/Code Laboratories/CL NUI Platform/SDK/" CACHE PATH "Path to CL NUI Platform/SDK/ directory")
				MESSAGE(STATUS "Kinect CL NUI SDK found in: ${CLNUI_SDK_DIR}")
			ELSE(BASE_CLNUI_SDK_DIR)
				SET(CLNUI_SDK_DIR "" CACHE PATH "Path to CL NUI Platform/SDK/ directory")
			ENDIF(BASE_CLNUI_SDK_DIR)

			IF (EXISTS "${CLNUI_SDK_DIR}/Include/CLNUIDevice.h")
				SET(CMAKE_MRPT_HAS_KINECT 1)
				SET(CMAKE_MRPT_HAS_CL_NUI 1)

				INCLUDE_DIRECTORIES("${CLNUI_SDK_DIR}/Include")
				APPEND_MRPT_LIBS("${CLNUI_SDK_DIR}/Lib/CLNUIDevice.lib")
			ELSE (EXISTS "${CLNUI_SDK_DIR}/Include/CLNUIDevice.h")
				MESSAGE(SEND_ERROR "*** ERROR *** CL NUI Platform needed for Kinect. Please, install it and set CLNUI_SDK_DIR or disable BUILD_KINECT")
			ENDIF (EXISTS "${CLNUI_SDK_DIR}/Include/CLNUIDevice.h")
		ENDIF (BUILD_KINECT_USE_CLNUI)

	ELSE(UNIX)
		MESSAGE(SEND_ERROR "Sorry! Kinect is supported on Unix or Win32 only. Please, disable BUILD_KINECT.")
	ENDIF(UNIX)

ELSE(BUILD_KINECT)
	SET(CMAKE_MRPT_HAS_KINECT 0)
ENDIF(BUILD_KINECT)

# It seems the PCL find_package changes some wxWidgets variables (wtf!), so make a backup:
set(BCK_wxWidgets_LIB_DIR ${wxWidgets_LIB_DIR})
set(BCK_wxWidgets_LIBRARIES ${wxWidgets_LIBRARIES})

# PCL library:
# --------------------------------------------
SET(CMAKE_MRPT_HAS_PCL 0)
find_package(PCL COMPONENTS io common registration QUIET)
if (PCL_FOUND)
	SET(CMAKE_MRPT_HAS_PCL 1)

	INCLUDE_DIRECTORIES(${PCL_INCLUDE_DIRS})
	link_directories(${PCL_LIBRARY_DIRS})
	add_definitions(${PCL_DEFINITIONS})

	# Add PCL directories as "-isystem" to avoid warnings:
	FOREACH(PCLDIR ${PCL_INCLUDE_DIRS})
		SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -isystem ${PCLDIR}")
	ENDFOREACH(PCLDIR)

endif(PCL_FOUND)

# Undo backups: (read above)
set(wxWidgets_LIB_DIR ${BCK_wxWidgets_LIB_DIR})
set(wxWidgets_LIBRARIES ${BCK_wxWidgets_LIBRARIES})


# WXWIDGETS+GL: Add libraries
# -------------------------------------------
IF(CMAKE_MRPT_HAS_WXWIDGETS)
	LIST(LENGTH wxWidgets_LIBRARIES wxLibsCount)
	if (wxLibsCount GREATER 0)
		APPEND_MRPT_LIBS(${wxWidgets_LIBRARIES})

		FOREACH(WXLIB ${wxWidgets_LIBRARIES})
			SET(MRPT_LINKER_LIBS_FOR_PKGCONFIG "${MRPT_LINKER_LIBS_FOR_PKGCONFIG} ${WXLIB}")
		ENDFOREACH(WXLIB)

	endif(wxLibsCount GREATER 0)
ENDIF(CMAKE_MRPT_HAS_WXWIDGETS)

# SuiteSparse: (for Cholmod & CSparse)
# ----------------------------
FIND_PACKAGE(SuiteSparse)

SET(CMAKE_MRPT_HAS_CHOLDMOD 0)
SET(CMAKE_MRPT_HAS_CHOLDMOD_SYSTEM 0)
IF(CHOLMOD_FOUND)
	IF($ENV{VERBOSE})
		MESSAGE(STATUS "Choldmod: includedir: ${CHOLMOD_INCLUDE_DIR}")
		MESSAGE(STATUS "          libs: ${CHOLMOD_LIBRARIES}")
	ENDIF($ENV{VERBOSE})

	INCLUDE_DIRECTORIES(${CHOLMOD_INCLUDE_DIR})
	APPEND_MRPT_LIBS(${CHOLMOD_LIBRARIES})

	SET(CMAKE_MRPT_HAS_CHOLDMOD 1)
	SET(CMAKE_MRPT_HAS_CHOLDMOD_SYSTEM 1)
ENDIF(CHOLMOD_FOUND)

# SuiteSparse: CSparse
# ----------------------------
IF(CSPARSE_FOUND)
	IF($ENV{VERBOSE})
		MESSAGE(STATUS "CSparse: includedir: ${CSPARSE_INCLUDE_DIR}")
		MESSAGE(STATUS "         libs: ${CSPARSE_LIBRARY}")
	ENDIF($ENV{VERBOSE})

	INCLUDE_DIRECTORIES(${CSPARSE_INCLUDE_DIR})
	APPEND_MRPT_LIBS(${CSPARSE_LIBRARY})

	SET(CMAKE_MRPT_HAS_CSPARSE 1)
	SET(CMAKE_MRPT_HAS_CSPARSE_SYSTEM 1)
ELSE(CSPARSE_FOUND)
	# Default: use embedded version
	SET(CMAKE_MRPT_HAS_CSPARSE 1)
	SET(CMAKE_MRPT_HAS_CSPARSE_SYSTEM 0)
ENDIF(CSPARSE_FOUND)


# Save the basic list of libraries (without MRPT-libs):
# -------------------------------------------------------------------------------------
SET(MRPTLIB_LINKER_LIBS ${MRPT_LINKER_LIBS} ${MRPT_FFMPEG_LIBS_TO_LINK})
IF($ENV{VERBOSE})
	MESSAGE(STATUS "MRPTLIB_LINKER_LIBS: ${MRPTLIB_LINKER_LIBS}")
ENDIF($ENV{VERBOSE})

# If we are building .so libs, don't link every program to all the libs:
IF(BUILD_SHARED_LIBS)
	SET(MRPT_LINKER_LIBS "")
ENDIF(BUILD_SHARED_LIBS)


IF(NOT MSVC AND NOT BORLAND)
	FOREACH(_LIB ${ALL_MRPT_LIBS})
		LIST(APPEND MRPT_LINKER_LIBS_FOR_PKGCONFIG "-l${_LIB}")
	ENDFOREACH(_LIB)
ENDIF(NOT MSVC AND NOT BORLAND)

# Save the list of libraries according to the current debug/release build (gcc only)
# -------------------------------------------------------------------------------------
SET(MRPT_LINKER_LIBS_RELorDEB ${MRPT_LINKER_LIBS})

IF(CMAKE_BUILD_TYPE STREQUAL "Debug")
	SET(MRPT_LINKER_LIBS_POSTFIX "${MRPT_DEBUG_POSTFIX}")
ELSE(CMAKE_BUILD_TYPE STREQUAL "Debug")
	SET(MRPT_LINKER_LIBS_POSTFIX "")
ENDIF(CMAKE_BUILD_TYPE STREQUAL "Debug")

# Detect wordsize:
# -------------------------------------------
IF(CMAKE_SIZEOF_VOID_P EQUAL 8)  # Size in bytes!
	SET(CMAKE_MRPT_WORD_SIZE 64)
ELSE(CMAKE_SIZEOF_VOID_P EQUAL 8)  # Size in bytes!
	SET(CMAKE_MRPT_WORD_SIZE 32)
ENDIF(CMAKE_SIZEOF_VOID_P EQUAL 8)


# ---------------------------------------------------------------
#   "Clasic" function & headers detection:
# ---------------------------------------------------------------
INCLUDE (CheckFunctionExists)
INCLUDE (CheckIncludeFile)
INCLUDE (CheckTypeSize)

SET(CMAKE_REQUIRED_INCLUDES "math.h")
if(CMAKE_COMPILER_IS_GNUCXX)
	SET(CMAKE_REQUIRED_LIBRARIES "m")
endif(CMAKE_COMPILER_IS_GNUCXX)

CHECK_FUNCTION_EXISTS(timegm HAVE_TIMEGM)
CHECK_FUNCTION_EXISTS(_mkgmtime HAVE_MKGMTIME)
CHECK_FUNCTION_EXISTS(alloca HAVE_ALLOCA_FUNC)
CHECK_FUNCTION_EXISTS(gettid HAVE_GETTID)
CHECK_FUNCTION_EXISTS(sincos HAVE_SINCOS)
CHECK_FUNCTION_EXISTS(lrint HAVE_LRINT)
CHECK_FUNCTION_EXISTS(erf HAVE_ERF)
CHECK_FUNCTION_EXISTS(posix_memalign HAVE_POSIX_MEMALIGN)
CHECK_FUNCTION_EXISTS(_aligned_malloc HAVE_ALIGNED_MALLOC)

#  This seems not to work and is more complex than it looks at first sight... :-(
#SET(CMAKE_REQUIRED_INCLUDES "windows.h")
#SET(CMAKE_REQUIRED_LIBRARIES kernel32)
#CHECK_FUNCTION_EXISTS(OpenThread HAVE_OPENTHREAD)

if(MSVC AND NOT MSVC6 AND NOT MSVC7)
	SET(HAVE_OPENTHREAD 1)
else(MSVC AND NOT MSVC6 AND NOT MSVC7)
	SET(HAVE_OPENTHREAD 0)
endif(MSVC AND NOT MSVC6 AND NOT MSVC7)



CHECK_INCLUDE_FILE("alloca.h" HAVE_ALLOCA_H)
CHECK_INCLUDE_FILE("linux/serial.h" HAVE_LINUX_SERIAL_H)
CHECK_INCLUDE_FILE("malloc.h" HAVE_MALLOC_H)
CHECK_INCLUDE_FILE("malloc/malloc.h" HAVE_MALLOC_MALLOC_H)

IF(HAVE_ALLOCA_FUNC OR HAVE_ALLOCA_H)
	SET(HAVE_ALLOCA 1)
ENDIF(HAVE_ALLOCA_FUNC OR HAVE_ALLOCA_H)

if(CMAKE_MRPT_HAS_GLUT_SYSTEM)
	SET(HAVE_FREEGLUT_EXT_H 0)
	FIND_FILE(FREEGLUTEXT_HFILE GL/freeglut_ext.h)
	if(FREEGLUTEXT_HFILE)
		MARK_AS_ADVANCED(FREEGLUTEXT_HFILE)
		SET(HAVE_FREEGLUT_EXT_H 1)
	endif(FREEGLUTEXT_HFILE)
else(CMAKE_MRPT_HAS_GLUT_SYSTEM)
	SET(HAVE_FREEGLUT_EXT_H 1)
endif(CMAKE_MRPT_HAS_GLUT_SYSTEM)

CHECK_INCLUDE_FILE("stdint.h" HAVE_STDINT_H)
CHECK_INCLUDE_FILE("inttypes.h" HAVE_INTTYPES_H)
CHECK_INCLUDE_FILE("winsock2.h" HAVE_WINSOCK2_H)

# If we want SSE2, check for the expected headers:
IF (CMAKE_MRPT_HAS_SSE2)
	CHECK_INCLUDE_FILE("emmintrin.h" HAVE_EMMINTRIN_H)
	CHECK_INCLUDE_FILE("mmintrin.h"  HAVE_MMINTRIN_H)

	# If the headers are not found, disable optimizations:
	IF (NOT HAVE_MMINTRIN_H OR NOT HAVE_EMMINTRIN_H)
		SET(CMAKE_MRPT_HAS_SSE2 0)
	ENDIF(NOT HAVE_MMINTRIN_H OR NOT HAVE_EMMINTRIN_H)
ENDIF(CMAKE_MRPT_HAS_SSE2)

# If we want SSE3, check for the expected headers:
IF (CMAKE_MRPT_HAS_SSE3)

	# JL: Before the CHECK_INCLUDE_FILE() we need to temporarily enable
	#  the -msse3 flag in GCC or the test program that CMake builds will
	#  always fail even if the header is present:
	IF(CMAKE_COMPILER_IS_GNUCXX)
		SET(TEMP_BACKUP_CMAKE_C_FLAGS ${CMAKE_C_FLAGS})
		SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -msse3")
	ENDIF(CMAKE_COMPILER_IS_GNUCXX)

	CHECK_INCLUDE_FILE("pmmintrin.h" HAVE_PMMINTRIN_H)

	# Restore from above:
	IF(CMAKE_COMPILER_IS_GNUCXX)
		SET(CMAKE_C_FLAGS ${TEMP_BACKUP_CMAKE_C_FLAGS})
	ENDIF(CMAKE_COMPILER_IS_GNUCXX)

	# If the headers are not found, disable optimizations:
	IF (NOT HAVE_PMMINTRIN_H)
		SET(CMAKE_MRPT_HAS_SSE3 0)
	ENDIF (NOT HAVE_PMMINTRIN_H)
ENDIF(CMAKE_MRPT_HAS_SSE3)


# Compiler type sizes:
check_type_size("long double"  HAVE_LONG_DOUBLE)


# ---------------------------------------------------------------
#   detect endian-ness
# ---------------------------------------------------------------
INCLUDE(TestBigEndian)
TEST_BIG_ENDIAN(CMAKE_MRPT_IS_BIG_ENDIAN)



# ----------------------------------------------------------------------------
# Transform the variables MRPT_XXX="ON/OFF" to CMAKE_MRPT_XXX="1/0"
# ----------------------------------------------------------------------------
MACRO(CREATE_CMAKE_MRPT_DEFINE defName)
	IF(${defName} MATCHES "ON")
		SET(CMAKE_${defName} "1")
	ELSE(${defName} MATCHES "ON")
		SET(CMAKE_${defName} "0")
	ENDIF(${defName} MATCHES "ON")
ENDMACRO(CREATE_CMAKE_MRPT_DEFINE)

CREATE_CMAKE_MRPT_DEFINE(MRPT_ALWAYS_CHECKS_DEBUG)
CREATE_CMAKE_MRPT_DEFINE(MRPT_ALWAYS_CHECKS_DEBUG_MATRICES)
CREATE_CMAKE_MRPT_DEFINE(MRPT_HAS_BUMBLEBEE)
CREATE_CMAKE_MRPT_DEFINE(MRPT_HAS_PHIDGET)
CREATE_CMAKE_MRPT_DEFINE(MRPT_HAS_SVS)
CREATE_CMAKE_MRPT_DEFINE(MRPT_HAS_ASSERT)
CREATE_CMAKE_MRPT_DEFINE(MRPT_HAS_STACKED_EXCEPTIONS)
CREATE_CMAKE_MRPT_DEFINE(MRPT_ENABLE_EMBEDDED_GLOBAL_PROFILER)
CREATE_CMAKE_MRPT_DEFINE(MRPT_HAS_ASIAN_FONTS)
CREATE_CMAKE_MRPT_DEFINE(CMAKE_MRPT_HAS_INOTIFY)

# ------------------------------------
#  Backward compatibility switches
# ------------------------------------
SET(MRPT_BACKCOMPATIB_08X OFF CACHE BOOL "API backward compatible with 0.8.X")

# ----------------------------------------------------------------------------
#   				UPDATE CONFIG FILES & SCRIPTS:
#
#  CONFIGURE_FILE(InputFile OutputFile [COPYONLY] [ESCAPE_QUOTES] [@ONLY])
# If @ONLY is specified, only variables of the form @VAR@ will be
#  replaces and ${VAR} will be ignored.
#
#  A directory will be created for each platform so the "config.h" file is
#   not overwritten if cmake generates code in the same path.
# ----------------------------------------------------------------------------
SET(MRPT_CONFIG_FILE_INCLUDE_DIR "${CMAKE_BINARY_DIR}/include/mrpt-config/" CACHE PATH "Where to create the platform-dependant config.h")
IF(UNIX)
	SET(MRPT_CONFIG_FILE_INCLUDE_DIR "${CMAKE_BINARY_DIR}/include/mrpt-config/unix/" )
ENDIF(UNIX)
IF (WIN32)
	SET(MRPT_CONFIG_FILE_INCLUDE_DIR "${CMAKE_BINARY_DIR}/include/mrpt-config/win32/")
ENDIF(WIN32)

FILE(MAKE_DIRECTORY  "${MRPT_CONFIG_FILE_INCLUDE_DIR}")
FILE(MAKE_DIRECTORY  "${MRPT_CONFIG_FILE_INCLUDE_DIR}/mrpt")

#MESSAGE(STATUS "Parsing 'config.h.in'")
CONFIGURE_FILE("${CMAKE_SOURCE_DIR}/parse-files/config.h.in" "${MRPT_CONFIG_FILE_INCLUDE_DIR}/mrpt/config.h")

# ----------------------------------------------------------------------------
#  Update the library version header file
#    FILE_TO_PARSE="SRC/include/mrpt/MRPT_version.h.in"
#    TARGET_FILE  ="MRPT_version.h"
# ----------------------------------------------------------------------------
#SET(CMAKE_MRPT_COMPLETE_NAME "MRPT ${CMAKE_MRPT_VERSION_NUMBER_MAJOR}.${CMAKE_MRPT_VERSION_NUMBER_MINOR}.${CMAKE_MRPT_VERSION_NUMBER_PATCH} ${CMAKE_MRPT_SVNVERSION}")
SET(CMAKE_MRPT_COMPLETE_NAME "MRPT ${CMAKE_MRPT_VERSION_NUMBER_MAJOR}.${CMAKE_MRPT_VERSION_NUMBER_MINOR}.${CMAKE_MRPT_VERSION_NUMBER_PATCH}") # Without the SVN number, avoids recompiling the whole thing with each commit/update.

# Build a three digits version code, eg. 0.5.1 -> 051,  1.2.0 -> 120
SET(CMAKE_MRPT_VERSION_CODE "0x${CMAKE_MRPT_VERSION_NUMBER_MAJOR}${CMAKE_MRPT_VERSION_NUMBER_MINOR}${CMAKE_MRPT_VERSION_NUMBER_PATCH}")

#MESSAGE(STATUS "Parsing 'version.h.in'")
CONFIGURE_FILE("${CMAKE_SOURCE_DIR}/parse-files/version.h.in" "${MRPT_CONFIG_FILE_INCLUDE_DIR}/mrpt/version.h")


# ---------------------------------------------------------------------------
# The C++ include & link directories:
# ---------------------------------------------------------------------------
INCLUDE_DIRECTORIES(".")
INCLUDE_DIRECTORIES("${MRPT_CONFIG_FILE_INCLUDE_DIR}")

LINK_DIRECTORIES("${CMAKE_BINARY_DIR}/lib")

IF(CMAKE_MRPT_HAS_BUMBLEBEE)
	INCLUDE_DIRECTORIES("${BUMBLEBEE_PGR_FLYCAPTURE_ROOT_DIR}/include")
	INCLUDE_DIRECTORIES("${BUMBLEBEE_TRICLOPS_ROOT_DIR}/include")

	LINK_DIRECTORIES("${BUMBLEBEE_PGR_FLYCAPTURE_ROOT_DIR}/lib")
	LINK_DIRECTORIES("${BUMBLEBEE_TRICLOPS_ROOT_DIR}/lib")
ENDIF(CMAKE_MRPT_HAS_BUMBLEBEE)

IF(CMAKE_MRPT_HAS_PHIDGET)
	INCLUDE_DIRECTORIES("${PHIDGET_ROOT_DIR}/include/")
	LINK_DIRECTORIES("${PHIDGET_ROOT_DIR}/lib")
ENDIF(CMAKE_MRPT_HAS_PHIDGET)

IF(CMAKE_MRPT_HAS_SVS)
	INCLUDE_DIRECTORIES("${SVS_ROOT_DIR}/src")

	LINK_DIRECTORIES("${SVS_ROOT_DIR}/bin")
ENDIF(CMAKE_MRPT_HAS_SVS)

# Add user supplied extra options (optimization, etc...)
# ==========================================================
SET(USER_EXTRA_CPP_FLAGS "" CACHE STRING "Put extra compiler options here if desired")

# ----------------------------------------------------------------------------
# 			Set the maximum level of warnings:
# ----------------------------------------------------------------------------
SET( MRPT_EXTRA_WARNINGS OFF CACHE BOOL "Enable extra warnings apart from -Wall")
MARK_AS_ADVANCED(MRPT_EXTRA_WARNINGS)

# Should be set to true for development
SET( MRPT_WARNINGS_ARE_ERRORS OFF CACHE BOOL "Treat warnings as errors")
MARK_AS_ADVANCED(MRPT_WARNINGS_ARE_ERRORS)

# Whole program optimization?
SET( MRPT_WHOLE_PROGRAM_OPTIMIZATION OFF CACHE BOOL "Flags for whole program optimization.")
MARK_AS_ADVANCED(MRPT_WHOLE_PROGRAM_OPTIMIZATION)

# Include Asian fonts in utils::CMRPTCanvas ?
SET( MRPT_HAS_ASIAN_FONTS ON CACHE BOOL "Enable Asian fonts in utils::CMRPTCanvas (increases library size).")


IF(MSVC)
	SET(EXTRA_CPP_FLAGS "/W3 /D _CRT_SECURE_NO_DEPRECATE /D _CRT_NONSTDC_NO_DEPRECATE")

	# Whole program optimization
	IF(MRPT_WHOLE_PROGRAM_OPTIMIZATION AND MSVC)
		SET(EXTRA_CPP_FLAGS "${EXTRA_CPP_FLAGS} /GL")
		SET(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} /LTCG ")
		#SET(CMAKE_EXE_LINKER_FLAGS_RELEASE "${CMAKE_EXE_LINKER_FLAGS_RELEASE} /LTCG ")
		SET(CMAKE_MODULE_LINKER_FLAGS "${CMAKE_MODULE_LINKER_FLAGS} /LTCG ")
		#SET(CMAKE_MODULE_LINKER_FLAGS_RELEASE "${CMAKE_MODULE_LINKER_FLAGS_RELEASE} /LTCG ")
		SET(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} /LTCG ")
	ENDIF(MRPT_WHOLE_PROGRAM_OPTIMIZATION AND MSVC)

	# Remove unreferenced functions: function level linking
	SET(EXTRA_CPP_FLAGS "${EXTRA_CPP_FLAGS} /Gy")
	SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} /W3 ")

	# SSE2?
	IF (CMAKE_MRPT_HAS_SSE2 AND NOT CMAKE_MRPT_WORD_SIZE EQUAL 64)
		SET(EXTRA_CPP_FLAGS "${EXTRA_CPP_FLAGS} /arch:SSE2")
	ENDIF(CMAKE_MRPT_HAS_SSE2 AND NOT CMAKE_MRPT_WORD_SIZE EQUAL 64)

	# SSE3?
	# TODO: Any flag needed?

	# Fast math:
	#SET(EXTRA_CPP_FLAGS "${EXTRA_CPP_FLAGS} /fp:fast")

ENDIF(MSVC)

IF(CMAKE_COMPILER_IS_GNUCXX)


	# Wall & pedantic?
	IF(APPLE)
		SET(MRPT_BUILD_GCC_PEDANTIC_DEFAULT "OFF")
	ELSE(APPLE)
		SET(MRPT_BUILD_GCC_PEDANTIC_DEFAULT "ON")
	ENDIF(APPLE)

	SET( MRPT_BUILD_GCC_PEDANTIC ${MRPT_BUILD_GCC_PEDANTIC_DEFAULT} CACHE BOOL "Enable pedantic error detection (with GCC only)")
	MARK_AS_ADVANCED(MRPT_BUILD_GCC_PEDANTIC)

	# High level of warnings.
	# The -Wno-long-long is required in 64bit systems when including sytem headers.
	# The -Wno-variadic-macros was needed for Eigen3, StdVector.h
 	SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${CMAKE_CONFIGURE_CFLAGS} -Wall -Wno-long-long -Wno-write-strings -Wno-variadic-macros")
 	SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${CMAKE_CONFIGURE_CFLAGS}")

	# Use "modern" C99 ! ;-)
 	SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -std=c99")


	IF(MRPT_BUILD_GCC_PEDANTIC)
		SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pedantic")
 		SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -pedantic")
	ENDIF(MRPT_BUILD_GCC_PEDANTIC)

	# We need pthread's
	IF (UNIX)
		SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread")
	ENDIF(UNIX)

	IF(MRPT_WARNINGS_ARE_ERRORS)
		SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Werror")
		SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Werror")
	ENDIF(MRPT_WARNINGS_ARE_ERRORS)


	# Whole program optimization
	IF(MRPT_WHOLE_PROGRAM_OPTIMIZATION)
		SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} --combine")

		SET(MRPT_EXES_CXX_FLAGS "${MRPT_EXES_CXX_FLAGS} -fwhole-program --combine")
	ENDIF(MRPT_WHOLE_PROGRAM_OPTIMIZATION)


	# "-mtune=native" generates code optimized for the detected current processor.
	set(opt_native_def OFF)
	IF (CMAKE_MRPT_GCC_VERSION GREATER 41)
		if (WIN32 OR "${CMAKE_MRPT_ARCH}" MATCHES ".*86" OR "${CMAKE_MRPT_ARCH}" MATCHES "amd64")
			set(opt_native_def ON)
		endif (WIN32 OR "${CMAKE_MRPT_ARCH}" MATCHES ".*86" OR "${CMAKE_MRPT_ARCH}" MATCHES "amd64")
	ENDIF (CMAKE_MRPT_GCC_VERSION GREATER 41)

	SET( MRPT_OPTIMIZE_NATIVE ${opt_native_def} CACHE BOOL "GCC optimizations for current processor (-mtune=native). Requires GCC 4.2+")


	IF(CMAKE_BUILD_TYPE STREQUAL "Debug")  # Not in debug!
		SET(MRPT_OPTIMIZE_NATIVE OFF)
	ENDIF(CMAKE_BUILD_TYPE STREQUAL "Debug")

	IF(CMAKE_MRPT_USE_DEB_POSTFIXS OR CMAKE_MRPT_IS_RPM_PACKAGE)
		# If we're building a Debian package, just DO NOT use -mtune=native
		SET(EXTRA_CPP_FLAGS ${CFLAGS})
	ELSE(CMAKE_MRPT_USE_DEB_POSTFIXS OR CMAKE_MRPT_IS_RPM_PACKAGE)
		IF(MRPT_OPTIMIZE_NATIVE AND NOT CMAKE_BUILD_TYPE STREQUAL "Debug")
			SET(EXTRA_CPP_FLAGS "${EXTRA_CPP_FLAGS} -O3 -mtune=native ")
		ENDIF(MRPT_OPTIMIZE_NATIVE AND NOT CMAKE_BUILD_TYPE STREQUAL "Debug")
	ENDIF(CMAKE_MRPT_USE_DEB_POSTFIXS OR CMAKE_MRPT_IS_RPM_PACKAGE)

	# "-ffast-math"
	SET( MRPT_OPTIMIZE_FFAST-MATH OFF CACHE BOOL "GCC optimization for floating math (-ffast-math).")
	IF(MRPT_OPTIMIZE_FFAST-MATH)
		SET(EXTRA_CPP_FLAGS "${EXTRA_CPP_FLAGS} -ffast-math")
	ENDIF(MRPT_OPTIMIZE_FFAST-MATH)

	# Profiling?
	IF(MRPT_ENABLE_PROFILING)
		SET(EXTRA_CPP_FLAGS "${EXTRA_CPP_FLAGS} -pg -g")
	ELSEIF(MRPT_ENABLE_PROFILING)
		# Remove unreferenced functions: function level linking
        # Remove unreferenced functions: function level linking
        if(NOT APPLE)
			SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -ffunction-sections")
        endif(NOT APPLE)
	ENDIF(MRPT_ENABLE_PROFILING)

	# SSE2?
	IF (CMAKE_MRPT_HAS_SSE2)
		SET(EXTRA_CPP_FLAGS "${EXTRA_CPP_FLAGS} -mfpmath=sse -msse2 -funroll-loops")
	ENDIF(CMAKE_MRPT_HAS_SSE2)

	# SSE3?
	IF (CMAKE_MRPT_HAS_SSE3)
		SET(EXTRA_CPP_FLAGS "${EXTRA_CPP_FLAGS} -msse3 -mssse3")
	ENDIF(CMAKE_MRPT_HAS_SSE3)


	# Parallel stdlibc++?
	IF(MRPT_ENABLE_LIBSTD_PARALLEL_MODE)
		SET(EXTRA_CPP_FLAGS "${EXTRA_CPP_FLAGS} -D_GLIBCXX_PARALLEL -fopenmp")
	ENDIF(MRPT_ENABLE_LIBSTD_PARALLEL_MODE)

	IF(CMAKE_BUILD_TYPE MATCHES "Debug")
		# This is to enable stack walker:
		SET(EXTRA_CPP_FLAGS "${EXTRA_CPP_FLAGS} -g")
		ADD_DEFINITIONS( -D_DEBUG)
		ADD_DEFINITIONS( -DDEBUG)
	ENDIF(CMAKE_BUILD_TYPE MATCHES "Debug")

ENDIF(CMAKE_COMPILER_IS_GNUCXX)

# Even more warnings: "Scott Meyers' Effective C++ book"
IF(MRPT_EXTRA_WARNINGS)
	IF(CMAKE_COMPILER_IS_GNUCXX)
		SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Weffc++")  # -Wold-style-cast  -Woverloaded-virtual
	ENDIF(CMAKE_COMPILER_IS_GNUCXX)
ENDIF(MRPT_EXTRA_WARNINGS)



# Add user supplied extra options (optimization, etc...)
# ==========================================================
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${EXTRA_CPP_FLAGS} ${USER_EXTRA_CPP_FLAGS}")
SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${EXTRA_CPP_FLAGS} ${USER_EXTRA_CPP_FLAGS}")
SET(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} ${EXTRA_CPP_FLAGS} ${USER_EXTRA_CPP_FLAGS}")
SET(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} ${EXTRA_CPP_FLAGS} ${USER_EXTRA_CPP_FLAGS}")

# Some tricks for MSVC:
if(MSVC)
	STRING(REGEX REPLACE "/EHsc" "/EHa" CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}")
	SET(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} /GS-")

	# For MSVC to avoid the C1128 error about too large object files:
	SET(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} /bigobj")
	SET(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} /bigobj")

endif(MSVC)

IF(CMAKE_COMPILER_IS_GNUCXX AND NOT APPLE)
	# --as-needed: Avoid unnecesary libraries (.so dependencies):
	#SET(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -Wl,--as-needed")
	#SET(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} -Wl,--as-needed")
ENDIF(CMAKE_COMPILER_IS_GNUCXX AND NOT APPLE)


IF ("${CMAKE_CONFIGURE_LDFLAGS}")
	SET(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${CMAKE_CONFIGURE_LDFLAGS}")
ENDIF ("${CMAKE_CONFIGURE_LDFLAGS}")

IF($ENV{VERBOSE})
	MESSAGE(STATUS "Final CMAKE_CXX_FLAGS: ${CMAKE_CXX_FLAGS}")
	MESSAGE(STATUS "Final CMAKE_EXE_LINKER_FLAGS: ${CMAKE_EXE_LINKER_FLAGS}")
	MESSAGE(STATUS "Final CMAKE_SHARED_LINKER_FLAGS: ${CMAKE_SHARED_LINKER_FLAGS}")
	MESSAGE(STATUS "Final MRPTLIB_LINKER_LIBS: ${MRPTLIB_LINKER_LIBS}")
ENDIF($ENV{VERBOSE})

# ----------------------------------------------------------------------------
#      					PROCESS SUBDIRECTORIES:
# ----------------------------------------------------------------------------
#MESSAGE(STATUS "CMAKE_CONFIGURATION_TYPES: ${CMAKE_CONFIGURATION_TYPES}")

# Save libs and executables in the same place
SET( LIBRARY_OUTPUT_PATH ${MRPT_BINARY_DIR}/lib CACHE PATH "Output directory for libraries" )
SET( EXECUTABLE_OUTPUT_PATH ${MRPT_BINARY_DIR}/bin CACHE PATH "Output directory for applications" )

#-----------------------------------
#  Build the hardware drivers?
#-----------------------------------
SET( BUILD_HWDRIVERS ON CACHE BOOL "Build hardware drivers (library apps/lib_hwdrivers)?.")
MARK_AS_ADVANCED(BUILD_HWDRIVERS)


#--------------------------------------------------------------
# If we are building the final step of the Debian package,
#  save each library files in the corresponding directories:
#--------------------------------------------------------------
IF(CMAKE_MRPT_USE_DEB_POSTFIXS)
	# Values when building a Debian package ---------------
	MESSAGE(STATUS "** mrpt cmake: Using Debian post-fix for install directories **")
	IF(CMAKE_BUILD_TYPE STREQUAL "Debug")
		# Debug packages
		SET(libmrpt_dev_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}/libmrpt-dbg/usr/")
	ELSE(CMAKE_BUILD_TYPE STREQUAL "Debug")
		# Normal packages
		SET(libmrpt_dev_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}/libmrpt-dev/usr/")
	ENDIF(CMAKE_BUILD_TYPE STREQUAL "Debug")

	SET(mrpt_apps_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}/mrpt-apps/usr/")
	SET(mrpt_doc_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}/mrpt-doc/usr/")
	SET(mrpt_pkgconfig_INSTALL_PREFIX "/usr")	# Values when building a Debian package
ELSE(CMAKE_MRPT_USE_DEB_POSTFIXS)
	# Values under normal conditions -----------------------
	SET(libmrpt_dev_INSTALL_PREFIX "")
	SET(mrpt_apps_INSTALL_PREFIX "")
	SET(mrpt_doc_INSTALL_PREFIX "")
	SET(mrpt_pkgconfig_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}") 	# Values under normal conditions
ENDIF(CMAKE_MRPT_USE_DEB_POSTFIXS)


# Remove /STACK:10000000 set by CMake. This value for stack size
# is very high, limiting the number of threads we can spawn.
# Default value used by Windows is 1MB which is good enough.
# By: Sahn Lam, @ CMake mailing list
STRING(REGEX REPLACE "/STACK:[0-9]+" "/STACK:1000000" CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS}")
STRING(REGEX REPLACE "/STACK:[0-9]+" "/STACK:1000000" CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS}")

# Auxiliary macros:
# ===============================
include(cmakemodules/FilterFileLists.cmake REQUIRED)
include(cmakemodules/MRPTPCHSupport.cmake REQUIRED)
include(cmakemodules/DeclareMRPTLib.cmake REQUIRED)
include(cmakemodules/DeclareAppDependencies.cmake REQUIRED)

# This will become a list with all libraries to be built, and their
#  dependencies stored in "mrpt-${name}_LIB_DEPS"
SET(ALL_MRPT_LIBS "" CACHE INTERNAL "")  # This emulates global vars

#-----------------------------------
# The third-party libraries
#-----------------------------------
add_subdirectory(otherlibs)

#-----------------------------------
# The MRPT C++ libraries:
#-----------------------------------
add_subdirectory(libs)

#-----------------------------------
# The applications:
#-----------------------------------
SET( BUILD_APPLICATIONS ON CACHE BOOL "If you only want the MRPT libraries, disable this.")
IF(BUILD_APPLICATIONS)
	add_subdirectory(apps)
ENDIF(BUILD_APPLICATIONS)



#-----------------------------------
# Documentation targets:
#-----------------------------------
add_subdirectory(doc)

# Add a "virtual" library, just for backward compatibility:
#  "mrpt-core", which just maps into a dependence on "mrpt-topography" + "mrpt-slam".
# ----------------------------------------------------------------------------
LIST(APPEND ALL_MRPT_LIBS mrpt-core)
get_property(_topo_DEP GLOBAL PROPERTY "mrpt-topography_LIB_DEPS")
get_property(_slam_DEP GLOBAL PROPERTY "mrpt-slam_LIB_DEPS")
set(_auxvar mrpt-topography mrpt-slam ${_topo_DEP} ${_slam_DEP})
list(REMOVE_DUPLICATES _auxvar)
set_property(GLOBAL PROPERTY "mrpt-core_LIB_DEPS" ${_auxvar})


# Create the code fragment: "DECLARE_LIBS_DEPS", for usage below while
#  generating "MRPTConfig.cmake"
SET(DECLARE_LIBS_DEPS "")
FOREACH(_LIB ${ALL_MRPT_LIBS})
	get_property(_LIB_DEP GLOBAL PROPERTY "${_LIB}_LIB_DEPS")
	SET(DECLARE_LIBS_DEPS "${DECLARE_LIBS_DEPS} set_property(GLOBAL PROPERTY \"${_LIB}_LIB_DEPS\" \"${_LIB_DEP}\")\n")
ENDFOREACH(_LIB)

# ----------------------------------------------------------------------------
#   Generate the MRPTConfig.cmake file
# ----------------------------------------------------------------------------
SET(THE_MRPT_SOURCE_DIR "${MRPT_SOURCE_DIR}")
SET(THE_MRPT_LIBS_INCL_DIR "${THE_MRPT_SOURCE_DIR}/libs")
SET(THE_CMAKE_BINARY_DIR "${CMAKE_BINARY_DIR}")
SET(THE_MRPT_CONFIG_FILE_INCLUDE_DIR "${MRPT_CONFIG_FILE_INCLUDE_DIR}")
SET(MRPT_CONFIGFILE_IS_INSTALL 0)

CONFIGURE_FILE("${MRPT_SOURCE_DIR}/MRPTConfig.cmake.in"
               "${MRPT_BINARY_DIR}/MRPTConfig.cmake" @ONLY IMMEDIATE )

# ----------------------------------------------------------------------------
#   Generate the MRPTConfig.cmake file for unix
#      installation in CMAKE_INSTALL_PREFIX
# ----------------------------------------------------------------------------
SET(MRPT_CONFIGFILE_IS_INSTALL 1)
IF(WIN32)
	SET(THE_MRPT_SOURCE_DIR "\${THIS_MRPT_CONFIG_PATH}")
	SET(THE_MRPT_LIBS_INCL_DIR "${THE_MRPT_SOURCE_DIR}/libs")
	SET(THE_CMAKE_BINARY_DIR "\${THIS_MRPT_CONFIG_PATH}")
	SET(THE_MRPT_CONFIG_FILE_INCLUDE_DIR "\${THIS_MRPT_CONFIG_PATH}/include/mrpt/mrpt-config/")
ELSE(WIN32)
	# Unix install. This .cmake file will end up in /usr/share/mrpt/MRPTConfig.cmake :
	IF (CMAKE_MRPT_USE_DEB_POSTFIXS)
		# We're building a .deb package: DESTDIR is NOT the final installation directory:
		SET(THE_MRPT_SOURCE_DIR "/usr")
		SET(THE_MRPT_LIBS_INCL_DIR "${THE_MRPT_SOURCE_DIR}/include/mrpt")
		SET(THE_CMAKE_BINARY_DIR "/usr")
		SET(THE_MRPT_CONFIG_FILE_INCLUDE_DIR "/usr/include/mrpt/mrpt-config/")
	ELSE(CMAKE_MRPT_USE_DEB_POSTFIXS)
		# Normal case: take the desired installation directory
		SET(THE_MRPT_SOURCE_DIR "${CMAKE_INSTALL_PREFIX}")
		SET(THE_MRPT_LIBS_INCL_DIR "${THE_MRPT_SOURCE_DIR}/include/mrpt")
		SET(THE_CMAKE_BINARY_DIR "${CMAKE_INSTALL_PREFIX}")
		SET(THE_MRPT_CONFIG_FILE_INCLUDE_DIR "${CMAKE_INSTALL_PREFIX}/include/mrpt/mrpt-config/")
	ENDIF(CMAKE_MRPT_USE_DEB_POSTFIXS)
ENDIF(WIN32)

CONFIGURE_FILE("${MRPT_SOURCE_DIR}/MRPTConfig.cmake.in"
               "${MRPT_BINARY_DIR}/unix-install/MRPTConfig.cmake" @ONLY IMMEDIATE )

# ----------------------------------------------------------------------------
#   Uninstall target, for "make uninstall"
# ----------------------------------------------------------------------------
CONFIGURE_FILE(
  "${CMAKE_CURRENT_SOURCE_DIR}/cmake_uninstall.cmake.in"
  "${CMAKE_CURRENT_BINARY_DIR}/cmake_uninstall.cmake"
  IMMEDIATE @ONLY)

ADD_CUSTOM_TARGET(uninstall
  "${CMAKE_COMMAND}" -P "${CMAKE_CURRENT_BINARY_DIR}/cmake_uninstall.cmake")


# ----------------------------------------------------------------------------
#  Hide some variables to the user, just show the important variables:
# ----------------------------------------------------------------------------
MARK_AS_ADVANCED(FORCE
	CMAKE_BACKWARDS_COMPATIBILITY
	wxWidgets_CONFIGURATION
	wxWidgets_LIB_DIR
	wxWidgets_USE_REL_AND_DBG
	wxWidgets_wxrc_EXECUTABLE
)

#-----------------------------------
# The examples
# *Note*: This must nw be AFTER the generation of the MRPTConfig.cmake files
#-----------------------------------
add_definitions(-DMRPT_OPENCV_SRC_DIR="${MRPT_OPENCV_SRC_DIR}")
SET(MRPT_DIR ${MRPT_BINARY_DIR})
include(cmakemodules/DefineExamples.cmake REQUIRED)

# ----------------------------------------------------------------------------
#   An auxiliary function to show messages:
# ----------------------------------------------------------------------------
MACRO(SHOW_CONFIG_LINE MSG_TEXT VALUE_BOOL)
	IF(${VALUE_BOOL})
		SET(VAL_TEXT "Yes")
	ELSE(${VALUE_BOOL})
		SET(VAL_TEXT " No")
	ENDIF(${VALUE_BOOL})
	MESSAGE(STATUS " ${MSG_TEXT} : ${VAL_TEXT} ${ARGV2}")
ENDMACRO(SHOW_CONFIG_LINE)

MACRO(SHOW_CONFIG_LINE_SYSTEM MSG_TEXT VALUE_BOOL)
	IF(${VALUE_BOOL})
		IF(${VALUE_BOOL}_SYSTEM)
			SET(VAL_TEXT "Yes (System)")
		ELSE(${VALUE_BOOL}_SYSTEM)
			SET(VAL_TEXT "Yes (Built-in)")
		ENDIF(${VALUE_BOOL}_SYSTEM)
	ELSE(${VALUE_BOOL})
		SET(VAL_TEXT " No")
	ENDIF(${VALUE_BOOL})
	MESSAGE(STATUS " ${MSG_TEXT} : ${VAL_TEXT} ${ARGV2}")
ENDMACRO(SHOW_CONFIG_LINE_SYSTEM)


# ----------------------------------------------------------------------------
#   TESTS
# ----------------------------------------------------------------------------
SET( BUILD_TESTING ON CACHE BOOL "Build MRPT tests")

# By default, use embedded version of gtest:
SET(CMAKE_MRPT_HAS_GTEST         1)
SET(CMAKE_MRPT_HAS_GTEST_SYSTEM  0)
SET(CMAKE_MRPT_HAS_GTEST_VERSION  "1.5.0")  # In "otherlibs/gtest-x.y.z"

IF( BUILD_TESTING)
	# Try using libgtest (Google testing library) from the system, if available
	IF(UNIX)
		FIND_FILE(GTEST_CONFIG_FILE gtest-config)
		IF(GTEST_CONFIG_FILE)
			MARK_AS_ADVANCED(GTEST_CONFIG_FILE)

			SET(CMAKE_MRPT_HAS_GTEST_SYSTEM 1)

			# Get the version:
			EXECUTE_PROCESS(COMMAND ${GTEST_CONFIG_FILE} --version
				RESULT_VARIABLE CMAKE_GTEST_CONFIG_RES
				OUTPUT_VARIABLE CMAKE_MRPT_HAS_GTEST_VERSION
				OUTPUT_STRIP_TRAILING_WHITESPACE
				)
			IF(${CMAKE_GTEST_CONFIG_RES})
				MESSAGE("Error invoking GTEST config file:\n ${GTEST_CONFIG_FILE} --version")
			ENDIF(${CMAKE_GTEST_CONFIG_RES})

			# Get the config params:
			EXECUTE_PROCESS(COMMAND ${GTEST_CONFIG_FILE} --libs
				RESULT_VARIABLE CMAKE_GTEST_CONFIG_RES
				OUTPUT_VARIABLE CMAKE_GTEST_LIBS
				OUTPUT_STRIP_TRAILING_WHITESPACE
				)
			IF(${CMAKE_GTEST_CONFIG_RES})
				MESSAGE("Error invoking GTEST config file:\n ${GTEST_CONFIG_FILE} --libs")
			ENDIF(${CMAKE_GTEST_CONFIG_RES})

			EXECUTE_PROCESS(COMMAND ${GTEST_CONFIG_FILE} --cxxflags
				RESULT_VARIABLE CMAKE_GTEST_CONFIG_RES
				OUTPUT_VARIABLE CMAKE_GTEST_CFLAGS
				OUTPUT_STRIP_TRAILING_WHITESPACE
				)
			IF(${CMAKE_GTEST_CONFIG_RES})
				MESSAGE("Error invoking GTEST config file:\n ${GTEST_CONFIG_FILE} --cxxflags")
			ENDIF(${CMAKE_GTEST_CONFIG_RES})

		ELSE(GTEST_CONFIG_FILE)
			SET(CMAKE_MRPT_HAS_GTEST_SYSTEM 0)
		ENDIF(GTEST_CONFIG_FILE)

	ENDIF(UNIX)

	add_subdirectory(tests)
ENDIF( BUILD_TESTING)


SET(IS_DEBIAN_DBG_PKG 0)
IF (CMAKE_MRPT_USE_DEB_POSTFIXS AND CMAKE_BUILD_TYPE STREQUAL "Debug")
	SET(IS_DEBIAN_DBG_PKG 1)
ENDIF (CMAKE_MRPT_USE_DEB_POSTFIXS AND CMAKE_BUILD_TYPE STREQUAL "Debug")

# ----------------------------------------------------------------------------
#   More installation commands:
# ----------------------------------------------------------------------------
FILE(GLOB MRPT_PKGCONFIG_PC_FILES "${MRPT_BINARY_DIR}/pkgconfig/mrpt-*.pc")

IF(EXISTS "${MRPT_BINARY_DIR}/pkgconfig/mrpt-base.pc" AND NOT IS_DEBIAN_DBG_PKG)
	INSTALL(
		FILES ${MRPT_PKGCONFIG_PC_FILES}
		DESTINATION ${libmrpt_dev_INSTALL_PREFIX}lib${LIB_SUFFIX}/pkgconfig )
ENDIF(EXISTS "${MRPT_BINARY_DIR}/pkgconfig/mrpt-base.pc" AND NOT IS_DEBIAN_DBG_PKG)

# CMake will look for MRPTConfig.cmake at: /usr/share|lib/mrpt
IF(WIN32)
	INSTALL(FILES "${MRPT_BINARY_DIR}/unix-install/MRPTConfig.cmake" DESTINATION ./ )
ELSE(WIN32)
	IF (NOT IS_DEBIAN_DBG_PKG)
		INSTALL(FILES "${MRPT_BINARY_DIR}/unix-install/MRPTConfig.cmake" DESTINATION ${libmrpt_dev_INSTALL_PREFIX}share/mrpt )
	ENDIF(NOT IS_DEBIAN_DBG_PKG)
ENDIF(WIN32)

# Docs, examples and the rest of files:
IF(WIN32)
	INSTALL(DIRECTORY "${MRPT_SOURCE_DIR}/doc" DESTINATION ./ PATTERN ".svn" EXCLUDE)
	INSTALL(DIRECTORY "${MRPT_SOURCE_DIR}/cmakemodules" DESTINATION ./ PATTERN ".svn" EXCLUDE)
	INSTALL(DIRECTORY "${MRPT_SOURCE_DIR}/otherlibs" DESTINATION ./ PATTERN ".svn" EXCLUDE)
	INSTALL(DIRECTORY "${MRPT_SOURCE_DIR}/parse-files" DESTINATION ./ PATTERN ".svn" EXCLUDE)
	INSTALL(DIRECTORY "${MRPT_SOURCE_DIR}/samples" DESTINATION ./  PATTERN ".svn" EXCLUDE)
	INSTALL(DIRECTORY "${MRPT_SOURCE_DIR}/share" DESTINATION ./  PATTERN ".svn" EXCLUDE)
	INSTALL(DIRECTORY "${MRPT_SOURCE_DIR}/scripts" DESTINATION ./  PATTERN ".svn" EXCLUDE)
	INSTALL(DIRECTORY "${MRPT_SOURCE_DIR}/swig" DESTINATION ./ PATTERN ".svn" EXCLUDE)
	INSTALL(DIRECTORY "${MRPT_SOURCE_DIR}/tests" DESTINATION ./  PATTERN ".svn" EXCLUDE)

	# Smart determination of the dependencies DLLs so they are also copied when installing:
	# ---------------------------------------------------------------------------------------
	# wxWidgets:
	IF (EXISTS "${wxWidgets_LIB_DIR}")
		FILE(GLOB_RECURSE EXTRA_DLLS "${wxWidgets_LIB_DIR}/*.dll")
		FOREACH(F ${EXTRA_DLLS})
			INSTALL(FILES "${F}" DESTINATION bin)
		ENDFOREACH(F)
	ENDIF (EXISTS "${wxWidgets_LIB_DIR}")

	# OpenCV:
	IF (EXISTS "${OpenCV_DIR}/bin/Release")
		FILE(GLOB_RECURSE EXTRA_DLLS "${OpenCV_DIR}/bin/*.dll") # This includes debug & release DLLs
		FOREACH(F ${EXTRA_DLLS})
			INSTALL(FILES "${F}" DESTINATION bin)
		ENDFOREACH(F)
	ENDIF (EXISTS "${OpenCV_DIR}/bin/Release")

        # Intel TBB dlls
        if(CMAKE_MRPT_HAS_TBB)
            string(REGEX REPLACE "/lib" "/bin" TBB_DLL_DIR "${TBB_LIB_DIR}")
            install(PROGRAMS "${TBB_DLL_DIR}/tbb.dll" DESTINATION bin COMPONENT main)
            install(PROGRAMS "${TBB_DLL_DIR}/tbb_debug.dll" DESTINATION bin COMPONENT main)
        endif(CMAKE_MRPT_HAS_TBB)

	# ffmpeg:
	IF (EXISTS "${FFMPEG_WIN32_ROOT_DIR}/bin")
		FILE(GLOB_RECURSE EXTRA_DLLS "${FFMPEG_WIN32_ROOT_DIR}/bin/*.dll")
		FOREACH(F ${EXTRA_DLLS})
			INSTALL(FILES "${F}" DESTINATION bin)
		ENDFOREACH(F)
	ENDIF (EXISTS "${FFMPEG_WIN32_ROOT_DIR}/bin")

	# libfreenect for windows: Copy the libusb.dll file:
	IF (NOT UNIX AND CMAKE_MRPT_HAS_FREENECT)
		IF (NOT "${LIBUSB_1_DLL_FOR_INSTALL}" STREQUAL "")
			INSTALL(FILES "${LIBUSB_1_DLL_FOR_INSTALL}/libusb0.dll" DESTINATION bin)
		ENDIF (NOT "${LIBUSB_1_DLL_FOR_INSTALL}" STREQUAL "")
	ENDIF (NOT UNIX AND CMAKE_MRPT_HAS_FREENECT)

	# Extra optional DLLs to be installed in the "bin" folder:
	file(TO_CMAKE_PATH "$ENV{MRPT_EXTRA_DLLS_TO_INSTALL}" MRPT_EXTRA_DLLS_TO_INSTALL)
	IF (NOT "${MRPT_EXTRA_DLLS_TO_INSTALL}" STREQUAL "")
		if (EXISTS "${MRPT_EXTRA_DLLS_TO_INSTALL}")
			FILE(STRINGS "${MRPT_EXTRA_DLLS_TO_INSTALL}" MRPT_EXTRA_DLLS_TO_INSTALL_FILES)
			FOREACH(XFIL ${MRPT_EXTRA_DLLS_TO_INSTALL_FILES})
				file(TO_CMAKE_PATH "${XFIL}" XFIL2)
				INSTALL(FILES "${XFIL2}" DESTINATION bin)
			ENDFOREACH(XFIL)
		endif (EXISTS "${MRPT_EXTRA_DLLS_TO_INSTALL}")
	ENDIF(NOT "${MRPT_EXTRA_DLLS_TO_INSTALL}" STREQUAL "")

	# My own debug DLLs:
	FILE(GLOB_RECURSE EXTRA_DLLS "${MRPT_BINARY_DIR}/bin/Debug/*.dll")
	FOREACH(F ${EXTRA_DLLS})
		INSTALL(FILES "${F}" DESTINATION bin)
	ENDFOREACH(F)
	FILE(GLOB_RECURSE EXTRA_LIBS "${MRPT_BINARY_DIR}/lib/Debug/*.lib")
	FOREACH(F ${EXTRA_LIBS})
		INSTALL(FILES "${F}" DESTINATION lib)
	ENDFOREACH(F)

ELSE(WIN32)
	IF (NOT IS_DEBIAN_DBG_PKG)
		INSTALL(DIRECTORY "${MRPT_SOURCE_DIR}/doc/html" DESTINATION ${mrpt_doc_INSTALL_PREFIX}share/doc/mrpt-doc/  PATTERN ".svn" EXCLUDE)
		INSTALL(DIRECTORY "${MRPT_SOURCE_DIR}/samples" DESTINATION ${mrpt_doc_INSTALL_PREFIX}share/doc/mrpt-doc/  PATTERN ".svn" EXCLUDE)
		INSTALL(FILES "${MRPT_SOURCE_DIR}/doc/mrpt_example1.tar.gz" DESTINATION ${mrpt_doc_INSTALL_PREFIX}share/doc/mrpt-doc/ )
		IF(EXISTS "${MRPT_SOURCE_DIR}/doc/mrpt-book.ps.gz")
			INSTALL(FILES "${MRPT_SOURCE_DIR}/doc/mrpt-book.ps.gz" DESTINATION ${mrpt_doc_INSTALL_PREFIX}share/doc/mrpt-doc/ )
		ENDIF(EXISTS "${MRPT_SOURCE_DIR}/doc/mrpt-book.ps.gz")

		# applications config files
		INSTALL(DIRECTORY "${MRPT_SOURCE_DIR}/share/applications" DESTINATION ${mrpt_apps_INSTALL_PREFIX}share PATTERN ".svn" EXCLUDE)
		#INSTALL(DIRECTORY "${MRPT_SOURCE_DIR}/share/menu" DESTINATION ${mrpt_apps_INSTALL_PREFIX}share  PATTERN ".svn" EXCLUDE)
		INSTALL(DIRECTORY "${MRPT_SOURCE_DIR}/share/mrpt" DESTINATION ${mrpt_apps_INSTALL_PREFIX}share  PATTERN ".svn" EXCLUDE)
		INSTALL(DIRECTORY "${MRPT_SOURCE_DIR}/share/pixmaps" DESTINATION ${mrpt_apps_INSTALL_PREFIX}share  PATTERN ".svn" EXCLUDE)

	 	# Mime types go to the mrpt-core package
		INSTALL(DIRECTORY "${MRPT_SOURCE_DIR}/share/mime" DESTINATION ${mrpt_apps_INSTALL_PREFIX}share  PATTERN ".svn" EXCLUDE)
	ENDIF(NOT IS_DEBIAN_DBG_PKG)
ENDIF(WIN32)

# The headers of all the MRPT libs:
# (in win32 the /libs/* tree is install entirely, not only the headers):
IF (UNIX AND NOT IS_DEBIAN_DBG_PKG)
	FOREACH(_LIB ${ALL_MRPT_LIBS})
		STRING(REGEX REPLACE "mrpt-(.*)" "\\1" _LIB ${_LIB})
		SET(SRC_DIR "${MRPT_SOURCE_DIR}/libs/${_LIB}/include/")
		IF (EXISTS "${SRC_DIR}")  # This is mainly to avoid a crash with "mrpt-core", which is a "virtual" MRPT module.
			INSTALL(DIRECTORY "${SRC_DIR}" DESTINATION ${libmrpt_dev_INSTALL_PREFIX}include/mrpt/${_LIB}/include/  PATTERN ".svn" EXCLUDE)
		ENDIF (EXISTS "${SRC_DIR}")
	ENDFOREACH(_LIB)
ENDIF(UNIX AND NOT IS_DEBIAN_DBG_PKG)


# Config-dependent headers:
IF (NOT IS_DEBIAN_DBG_PKG)
	INSTALL(FILES "${MRPT_CONFIG_FILE_INCLUDE_DIR}/mrpt/config.h" DESTINATION "${libmrpt_dev_INSTALL_PREFIX}include/mrpt/mrpt-config/mrpt/" )
	INSTALL(FILES "${MRPT_CONFIG_FILE_INCLUDE_DIR}/mrpt/version.h" DESTINATION "${libmrpt_dev_INSTALL_PREFIX}include/mrpt/mrpt-config/mrpt/" )
ENDIF(NOT IS_DEBIAN_DBG_PKG)

# If using embedded version, install embedded version as part of mrpt-base's headers:
IF (EIGEN_USE_EMBEDDED_VERSION AND NOT IS_DEBIAN_DBG_PKG)
	IF(WIN32)
		# Eigen headers must end up in /Program Files/MRPT-X.Y.Z/libs/base/...
		SET(MRPT_INSTALL_EIGEN_PREFIX "libs/base/include/")
	ELSE(WIN32)
		# Eigen headers must end up in /usr/...
		SET(MRPT_INSTALL_EIGEN_PREFIX "${libmrpt_dev_INSTALL_PREFIX}include/mrpt/base/include/")
	ENDIF(WIN32)

	INSTALL(
		DIRECTORY "${MRPT_SOURCE_DIR}/otherlibs/eigen3/Eigen"
		DESTINATION "${MRPT_INSTALL_EIGEN_PREFIX}" PATTERN ".svn" EXCLUDE)
	INSTALL(
		DIRECTORY "${MRPT_SOURCE_DIR}/otherlibs/eigen3/unsupported"
		DESTINATION "${MRPT_INSTALL_EIGEN_PREFIX}" PATTERN ".svn" EXCLUDE)
ENDIF (EIGEN_USE_EMBEDDED_VERSION AND NOT IS_DEBIAN_DBG_PKG)

# ----------------------------------------------------------------------------
#   Sumary:
# ----------------------------------------------------------------------------
MESSAGE(STATUS "")
IF(CMAKE_COMPILER_IS_GNUCXX)
	SET(MRPT_GCC_VERSION_STR "(GCC version: ${CMAKE_MRPT_GCC_VERSION})")
ENDIF(CMAKE_COMPILER_IS_GNUCXX)

IF($ENV{VERBOSE})
	message(STATUS "Compiler:                  ${CMAKE_CXX_COMPILER} ${MRPT_GCC_VERSION_STR} ")
	message(STATUS "  C++ flags (Release):       ${CMAKE_CXX_FLAGS} ${CMAKE_CXX_FLAGS_RELEASE}")
	message(STATUS "  C++ flags (Debug):         ${CMAKE_CXX_FLAGS} ${CMAKE_CXX_FLAGS_DEBUG}")
	message(STATUS "  Executable link flags (Release):    ${CMAKE_EXE_LINKER_FLAGS} ${CMAKE_EXE_LINKER_FLAGS_RELEASE}")
	message(STATUS "  Executable link flags (Debug):      ${CMAKE_EXE_LINKER_FLAGS} ${CMAKE_EXE_LINKER_FLAGS_DEBUG}")
	message(STATUS "  Lib link flags (Release):    ${CMAKE_SHARED_LINKER_FLAGS} ${CMAKE_SHARED_LINKER_FLAGS_RELEASE}")
	message(STATUS "  Lib link flags (Debug):      ${CMAKE_SHARED_LINKER_FLAGS} ${CMAKE_SHARED_LINKER_FLAGS_DEBUG}")
	MESSAGE(STATUS "")
ENDIF($ENV{VERBOSE})

MESSAGE(STATUS "List of MRPT libs/modules to be built (and dependencies):")
MESSAGE(STATUS "-----------------------------------------------------------------")
FOREACH(_LIB ${ALL_MRPT_LIBS})
	get_property(_LIB_DEP GLOBAL PROPERTY "${_LIB}_LIB_DEPS")
	MESSAGE(STATUS "  ${_LIB} : ${_LIB_DEP}")
ENDFOREACH(_LIB)
MESSAGE(STATUS "")

MESSAGE(STATUS "Resulting configuration for ${CMAKE_MRPT_COMPLETE_NAME}")
MESSAGE(STATUS "-----------------------------------------------------------------")
SHOW_CONFIG_LINE("Will MRPT be built as a shared library? " CMAKE_MRPT_BUILD_SHARED_LIB_ONOFF)
SHOW_CONFIG_LINE("Additional checks even in Release       " CMAKE_MRPT_ALWAYS_CHECKS_DEBUG)
SHOW_CONFIG_LINE("Additional matrix checks                " CMAKE_MRPT_ALWAYS_CHECKS_DEBUG_MATRICES)
MESSAGE(STATUS  " Word size (32/64 bits)                   : ${CMAKE_MRPT_WORD_SIZE}")
SHOW_CONFIG_LINE("Is system big endian?                   " CMAKE_MRPT_IS_BIG_ENDIAN)
SHOW_CONFIG_LINE("Use SSE2?                               " CMAKE_MRPT_HAS_SSE2)
SHOW_CONFIG_LINE("Use SSE3/SSSE3?                         " CMAKE_MRPT_HAS_SSE3)

SHOW_CONFIG_LINE_SYSTEM("Has eigen3                              " CMAKE_MRPT_HAS_EIGEN "Version: ${PKG_EIGEN3_VERSION}")
SHOW_CONFIG_LINE_SYSTEM("Has SuiteSparse/CHOLMOD                 " CMAKE_MRPT_HAS_CHOLDMOD)
SHOW_CONFIG_LINE_SYSTEM("Has SuiteSparse/CSparse                 " CMAKE_MRPT_HAS_CSPARSE)

IF(NOT UNIX)	# In fact, it's not "support for Bumblebee" but for it thru PGR vendor libraries.
	SHOW_CONFIG_LINE("Has PGR Digiclops/Triclops              " CMAKE_MRPT_HAS_BUMBLEBEE)
ENDIF(NOT UNIX)
SHOW_CONFIG_LINE("Has Phidget                             " CMAKE_MRPT_HAS_PHIDGET)
SHOW_CONFIG_LINE("Has Videre    stereo camera             " CMAKE_MRPT_HAS_SVS)
SHOW_CONFIG_LINE("Has wxWidgets incl. OpenGL classes      " CMAKE_MRPT_HAS_WXWIDGETS)
SHOW_CONFIG_LINE("Has OpenGL & GLUT:                      " CMAKE_MRPT_HAS_OPENGL_GLUT)
SHOW_CONFIG_LINE("Has xSENS MTi library (inertial sensor) " CMAKE_MRPT_HAS_xSENS)
SHOW_CONFIG_LINE("Has libdc1394-2 (FireWire capture)      " CMAKE_MRPT_HAS_LIBDC1394_2)
SHOW_CONFIG_LINE("Has SwissRanger 3/4000 3D camera        " CMAKE_MRPT_HAS_SWISSRANGE )
IF (WIN32)
SHOW_CONFIG_LINE("Has Kinect CL NUI                       " CMAKE_MRPT_HAS_CL_NUI)
ENDIF(WIN32)
SHOW_CONFIG_LINE_SYSTEM("Has OpenKinect libfreenect              " CMAKE_MRPT_HAS_FREENECT)


SHOW_CONFIG_LINE_SYSTEM("Has OpenCV (Image manipulation)         " CMAKE_MRPT_HAS_OPENCV "Version: ${MRPT_OPENCV_VERSION}")
SHOW_CONFIG_LINE("Has FabMap (optional for HMT-SLAM)      " CMAKE_MRPT_HAS_FABMAP )
SHOW_CONFIG_LINE_SYSTEM("Has ffmpeg libs (Video streaming)       " CMAKE_MRPT_HAS_FFMPEG "avcodec ${LIBAVCODEC_VERSION}, avutil ${LIBAVUTIL_VERSION}, avformat ${LIBAVFORMAT_VERSION}")
SHOW_CONFIG_LINE_SYSTEM("Has lib3ds (3DStudio scenes)            " CMAKE_MRPT_HAS_LIB3DS)
SHOW_CONFIG_LINE_SYSTEM("Has zlib (compression)                  " CMAKE_MRPT_HAS_ZLIB)
SHOW_CONFIG_LINE_SYSTEM("Has libjpeg (jpeg)                      " CMAKE_MRPT_HAS_JPEG)

IF(UNIX)
	SHOW_CONFIG_LINE_SYSTEM("Has libftdi (USB)                       " CMAKE_MRPT_HAS_FTDI)
ENDIF(UNIX)

SHOW_CONFIG_LINE_SYSTEM("Has gtest (Google unit testing library) " CMAKE_MRPT_HAS_GTEST "Version: ${CMAKE_MRPT_HAS_GTEST_VERSION}")


SHOW_CONFIG_LINE("Has PCL (Pointscloud library)           " CMAKE_MRPT_HAS_PCL  "Version: ${PCL_VERSION}")
SHOW_CONFIG_LINE("Has Intel threading lib (TBB)           " CMAKE_MRPT_HAS_TBB)


MESSAGE(STATUS "")

MESSAGE(STATUS "  Install prefix:     ${CMAKE_INSTALL_PREFIX}")
MESSAGE(STATUS "  C++ config header : ${MRPT_CONFIG_FILE_INCLUDE_DIR}")
MESSAGE(STATUS "-----------------------------------------------------------------")

MESSAGE(STATUS "")

# Final warnings:
IF (NOT CMAKE_MRPT_HAS_OPENCV AND NOT DISABLE_OPENCV)
	MESSAGE(STATUS "")
	MESSAGE(STATUS "***********************************************************************")
	MESSAGE(STATUS "* WARNING: It's STRONGLY recommended to build MRPT with OpenCV support.")
	MESSAGE(STATUS "*  To do so, set OpenCV_DIR to its CMake build dir. If you want to go ")
	MESSAGE(STATUS "*  on without OpenCV, proceed to build instead. ")
	MESSAGE(STATUS "***********************************************************************")
	MESSAGE(STATUS "")
ENDIF(NOT CMAKE_MRPT_HAS_OPENCV AND NOT DISABLE_OPENCV)


