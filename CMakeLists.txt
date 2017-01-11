# ----------------------------------------------------------------------------
# Root CMake file for the MRPT libraries and applications
#
#  Run with "cmake ." at the root directory to build the makefiles for
#   the MRPT C++ library, the samples, and the applications. Some scripts
#   for generating the documentation, etc. are also updated.
#
#  For compiling instructions for all compilers and platforms, see
#   http://www.mrpt.org/Building_and_Installing_Instructions
#
#  2007-2017, Jose Luis Blanco <jlblanco@ual.es>
#
#  NOTE: CMake can be obtained at http://www.cmake.org/
# ----------------------------------------------------------------------------
PROJECT(MRPT)

CMAKE_MINIMUM_REQUIRED(VERSION 2.6.4)

# -------------------------
#        Setup CMake
# -------------------------
#include(cmakemodules/script_init_cmake.cmake REQUIRED)		# CMake tweaks
# Allow empty endif() and such with CMake 2.4.
SET(CMAKE_ALLOW_LOOSE_LOOP_CONSTRUCTS 1)

if(COMMAND cmake_policy)
	# We don't want to mix relative and absolute paths in linker lib lists.
	cmake_policy(SET CMP0003 NEW)
	if(POLICY CMP0053)
		cmake_policy(SET CMP0053 OLD) # Simplify variable reference and escape sequence evaluation.
	endif()
	if(POLICY CMP0026)
		cmake_policy(SET CMP0026 OLD) # Allow using the LOCATION target property.
	endif()
	if(POLICY CMP0037)
		cmake_policy(SET CMP0037 OLD)  # Allow defining target "test"
	endif()
	if(POLICY CMP0043)
		cmake_policy(SET CMP0043 OLD) #  Ignore COMPILE_DEFINITIONS_<Config> properties.
	endif()

endif()
# -------------------------

# Detect wordsize:
IF(CMAKE_SIZEOF_VOID_P EQUAL 8)  # Size in bytes!
	SET(CMAKE_MRPT_WORD_SIZE 64)
ELSE()
	SET(CMAKE_MRPT_WORD_SIZE 32)
ENDIF()

include(cmakemodules/script_version_number.cmake REQUIRED)	#  Loads MRPT version number

LIST(APPEND CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmakemodules") # Directory for CMake scripts

# The root directory for all MRPT libraries/modules:
SET(MRPT_LIBS_ROOT "${CMAKE_SOURCE_DIR}/libs" CACHE INTERNAL "")  # This emulates global vars

# Windows default: lib. Debian pkgs: set from outside rules script
IF(NOT DEFINED CMAKE_INSTALL_LIBDIR)
	SET(CMAKE_INSTALL_LIBDIR "lib")
ENDIF()

# ---------------------------------------------------------------------------
# Create the minimum list of libraries required by an application that
#  uses the MRPT C++ library. This must be passed to "TARGET_LINK_LIBRARIES"
# NOTE: For the compilers MSVC* & Borland most of the libraries are
#        automatically included through "#pragma"'s ;-)
# ---------------------------------------------------------------------------
SET(MRPT_LINKER_LIBS "")

# ----- Useful macros ------
include(cmakemodules/UtilsMacros.cmake REQUIRED)
include(cmakemodules/DebugMacros.cmake REQUIRED)
include(cmakemodules/FilterFileLists.cmake REQUIRED)
include(cmakemodules/MRPTPCHSupport.cmake REQUIRED)
include(cmakemodules/DeclareMRPTLib.cmake REQUIRED)
include(cmakemodules/DeclareMEXLib.cmake REQUIRED)
include(cmakemodules/DeclareAppDependencies.cmake REQUIRED)
# --------------------------

include(cmakemodules/script_detect_unix_arch.cmake REQUIRED) # Detect machine architecture, on UNIX
include(cmakemodules/script_detect_gcc.cmake REQUIRED)       # Detect GNU GCC version
include(cmakemodules/script_detect_clang.cmake REQUIRED)     # Detect clang compiler version


# ------------------------------------------
# We want libraries to be named "libXXX"
#  and "libXXX-dbg" in all compilers:
# ------------------------------------------
set(CMAKE_DEBUG_POSTFIX  "-dbg")
IF(MSVC)
	set(MRPT_LIB_PREFIX "lib")  # Libs are: "libXXX"
ENDIF()

# In case of Makefiles if the user does not setup CMAKE_BUILD_TYPE, assume it's Release:
if (${CMAKE_GENERATOR} MATCHES ".*Makefiles")
    if("${CMAKE_BUILD_TYPE}" STREQUAL "")
        set(CMAKE_BUILD_TYPE Release)
    endif()
endif()

include(cmakemodules/script_select_app_to_build.cmake REQUIRED)		# Build (or not) some apps:

# ----------------------------------------------------------------------------
# 		CHECK FOR SYSTEM LIBRARIES, OPTIONS, ETC..
# ----------------------------------------------------------------------------

# Build static or dynamic libs?
# ===================================================
# Default: dynamic libraries:
SET(BUILD_SHARED_LIBS ON CACHE BOOL "Build shared libraries (.dll/.so) instead of static ones (.lib/.a)")
IF(BUILD_SHARED_LIBS)
	SET(CMAKE_MRPT_BUILD_SHARED_LIB "#define MRPT_BUILT_AS_DLL")
	SET(CMAKE_MRPT_BUILD_SHARED_LIB_ONOFF 1)
ELSE()
	SET(CMAKE_MRPT_BUILD_SHARED_LIB "/* #define MRPT_BUILT_AS_DLL */")
	SET(CMAKE_MRPT_BUILD_SHARED_LIB_ONOFF 0)
ENDIF()

# Only for Unix: use pkg-config to find libraries
# ===================================================
INCLUDE(FindPkgConfig OPTIONAL)
# "QUIET" was introduced in 2.8.2 for PKG_CHECK_MODULES
if (${CMAKE_MAJOR_VERSION}.${CMAKE_MINOR_VERSION}.${CMAKE_PATCH_VERSION} VERSION_GREATER 2.8.1)
   set(_QUIET QUIET)
endif()


# Group projects in "folders"
# ===================================================
IF (CMAKE_VERSION VERSION_GREATER 2.8.0)
	SET(ENABLE_SOLUTION_FOLDERS ON CACHE BOOL "Group projects under virtual folders of the compiler IDE (e.g. VisualStudio)")
ENDIF (CMAKE_VERSION VERSION_GREATER 2.8.0)

if (ENABLE_SOLUTION_FOLDERS)
  set_property(GLOBAL PROPERTY USE_FOLDERS ON)
  set_property(GLOBAL PROPERTY PREDEFINED_TARGETS_FOLDER "CMakeTargets")
endif ()


# Build the ARIA library?
SET(DEFAULT_BUILD_ARIA ON) # Default: Yes
if (${CMAKE_MRPT_KERNEL} MATCHES "GNU")   # Unless in HURD
	SET(DEFAULT_BUILD_ARIA OFF)
endif ()
# Decl option:
SET(BUILD_ARIA ${DEFAULT_BUILD_ARIA} CACHE BOOL "Build an embedded version of ActivMedia ARIA library (interface to mobile robots)")
IF(BUILD_ARIA)
	SET(CMAKE_MRPT_HAS_ARIA 1)
ELSE()
	SET(CMAKE_MRPT_HAS_ARIA 0)
ENDIF()

# Build the RoboPeak Lidar library?
SET(DEFAULT_BUILD_ROBOPEAK_LIDAR ON) # Default: Yes
if (${CMAKE_MRPT_KERNEL} MATCHES "GNU" OR APPLE)   # Unless in HURD
	SET(DEFAULT_BUILD_ROBOPEAK_LIDAR OFF)
endif ()
# Decl option:
SET(BUILD_ROBOPEAK_LIDAR ${DEFAULT_BUILD_ROBOPEAK_LIDAR} CACHE BOOL "Build an embedded version of RoboPeak LIDAR SDK (interface to low-cost lidar)")
IF(BUILD_ROBOPEAK_LIDAR)
	SET(CMAKE_MRPT_HAS_ROBOPEAK_LIDAR 1)
ELSE()
	SET(CMAKE_MRPT_HAS_ROBOPEAK_LIDAR 0)
ENDIF()

#-----------------------------------
#  Build the hardware drivers?
#-----------------------------------
SET(BUILD_HWDRIVERS ON CACHE BOOL "Build hardware drivers (library apps/lib_hwdrivers)?.")
MARK_AS_ADVANCED(BUILD_HWDRIVERS)

#-----------------------------------
#  Build with MRPT-MEX compatibility?
#-----------------------------------
SET(BUILD_MATLAB OFF CACHE BOOL "Build with compatibility options for MEX wrapper?.")

# GCC only:
# ===================================================
IF(CMAKE_COMPILER_IS_GNUCXX)
	# Enable libstdc++ parallel mode?
	SET(MRPT_ENABLE_LIBSTD_PARALLEL_MODE OFF CACHE BOOL "Enable parallel mode in libstdc++ (requires GCC 4.2.2+)")
ENDIF()

# Enable precompiled headers:
# ===================================================
IF(MSVC)
	SET(DEFAULT_VAL_PRECOMP_HDR ON)
ELSE()
	SET(DEFAULT_VAL_PRECOMP_HDR OFF)
ENDIF()

SET(MRPT_ENABLE_PRECOMPILED_HDRS ${DEFAULT_VAL_PRECOMP_HDR} CACHE BOOL "Enable precompiled headers (thru 'mrpt/LIB.h')")
MARK_AS_ADVANCED(MRPT_ENABLE_PRECOMPILED_HDRS)

# Eigen library plugins:
# ===================================================
include(cmakemodules/script_eigen.cmake REQUIRED)

# MRPT_TRY_START/END blocks
# ===================================================
SET(MRPT_HAS_STACKED_EXCEPTIONS ON CACHE BOOL "Enable MRPT_TRY_START/END blocks (disable it for speed up).")

# Optional MRPT embedded profiler:
# ===================================================
SET(MRPT_ENABLE_EMBEDDED_GLOBAL_PROFILER OFF CACHE BOOL "Enable MRPT embedded profiler.")

# ASSERT_ blocks
# ===================================================
SET(MRPT_HAS_ASSERT ON CACHE BOOL "Enable ASSERT_ statements (disable it for speed up).")

#   "Clasic" function & headers detection:
include(cmakemodules/script_detect_functions_headers.cmake REQUIRED)

# MSVC only:
# ===================================================
IF(MSVC)
	# Enable Parallel compilation?
	SET(COMPILE_IN_PARALLEL ON CACHE BOOL "Enable parallel compilation in Visual Studio")
ENDIF()

# ----------------------------------------------------------------------------
#   Uninstall target, for "make uninstall"
# Must be invoked *before* other embedded projects so MRPT's target "uninstall" exists first
# ----------------------------------------------------------------------------
CONFIGURE_FILE(
  "${CMAKE_CURRENT_SOURCE_DIR}/parse-files/cmake_uninstall.cmake.in"
  "${CMAKE_CURRENT_BINARY_DIR}/cmake_uninstall.cmake"
  IMMEDIATE @ONLY)

ADD_CUSTOM_TARGET(uninstall
  "${CMAKE_COMMAND}" -P "${CMAKE_CURRENT_BINARY_DIR}/cmake_uninstall.cmake")

if(ENABLE_SOLUTION_FOLDERS)
	set_target_properties(uninstall PROPERTIES FOLDER "CMakeTargets")
endif()

# ----------------------------------------------------------------------------
# Other sub-scripts:
# ----------------------------------------------------------------------------
include(cmakemodules/script_gridmap_options.cmake REQUIRED)			# Enable Intel Threading Building Blocks (TBB) lib?

include(cmakemodules/script_tbb.cmake REQUIRED)         # Enable Intel Threading Building Blocks (TBB) lib?
include(cmakemodules/script_phidget.cmake REQUIRED)     # Support for phidget Interface Kit with proximity sensor device :
include(cmakemodules/script_videre_svs.cmake REQUIRED)  # Support for Videre Design stereo camera:
include(cmakemodules/script_ftdi.cmake REQUIRED)        # Check for the FTDI headers (Linux only, in win32 we use built-in header & dynamic DLL load):
include(cmakemodules/script_inotify.cmake REQUIRED)     # Check for the sys/inotify headers (Linux only, in win32 we use the equivalent API for file system monitoring):
include(cmakemodules/script_gl_glut.cmake REQUIRED)     # Check for the GL,GLUT libraries
include(cmakemodules/script_opencv.cmake REQUIRED)      # Check for the OpenCV libraries (via pkg-config, CMake, with different options)
include(cmakemodules/script_openni2.cmake REQUIRED)     # Check for the OpenNI2 library
include(cmakemodules/script_ffmpeg.cmake REQUIRED)      # Check for ffmpeg C libraries: libavcodec, libavutil, libavformat, libswscale
include(cmakemodules/script_wxwidgets.cmake REQUIRED)   # Check for wxWidgets + GL
include(cmakemodules/script_libdc1394.cmake REQUIRED)   # Check for libdc1394-2
include(cmakemodules/script_assimp.cmake REQUIRED)      # Check for system assimp lib (3D models)
include(cmakemodules/script_lib3ds.cmake REQUIRED)      # Check for system lib3ds
include(cmakemodules/script_liblas.cmake REQUIRED)      # Check for the LAS LiDAR format library
include(cmakemodules/script_libtclap.cmake REQUIRED)    # Check for system libtclap
include(cmakemodules/script_zlib.cmake REQUIRED)        # Check for zlib
include(cmakemodules/script_jpeg.cmake REQUIRED)        # Check for jpeg
include(cmakemodules/script_swissrange.cmake REQUIRED)  # Support for SWISSRANGE 3D camera:
include(cmakemodules/script_SIMD.cmake REQUIRED)        # SSE2/SSE3/... optimization options
include(cmakemodules/script_gtest.cmake REQUIRED)       # Unit testing lib
include(cmakemodules/script_clang.cmake REQUIRED)       # clang library (For UTFConvert)
include(cmakemodules/script_national_instruments.cmake REQUIRED)  # NI C library
include(cmakemodules/script_sift_hess.cmake REQUIRED)  # SIFT Hess implementation
include(cmakemodules/script_xsens.cmake REQUIRED)      # XSens Motion trackers / IMU drivers
include(cmakemodules/script_flycapture2.cmake REQUIRED)# Check for PointGreyResearch (PGR) FlyCapture2 library
include(cmakemodules/script_triclops.cmake REQUIRED)	# Check for PointGreyResearch (PGR) Triclops library
include(cmakemodules/script_duo3d.cmake REQUIRED)		# Support for DUO3D Camera
include(cmakemodules/script_matlab.cmake REQUIRED)	# Support for Matlab MEX functions generation
include(cmakemodules/script_isense.cmake REQUIRED)		# Support for INTERSENSE Sensors
include(cmakemodules/script_nite2.cmake REQUIRED)		# Check for NITE2 library
include(cmakemodules/script_yamlcpp.cmake REQUIRED)		# Check for YAMLCPP library
include(cmakemodules/script_pcap.cmake REQUIRED)        # Check for the libpcap library
#include(cmakemodules/script_ros1.cmake REQUIRED)        # Check for ROS
include(cmakemodules/script_vtk.cmake REQUIRED)        # Check for VTK

# ---------------------------------------------------------------------------
#			OPTIONS
#The options for the user when using "cmakesetup" or "ccmake":
# ---------------------------------------------------------------------------
OPTION(MRPT_ALWAYS_CHECKS_DEBUG "Additional checks even in Release" "OFF")
MARK_AS_ADVANCED(FORCE MRPT_ALWAYS_CHECKS_DEBUG)
OPTION(MRPT_ALWAYS_CHECKS_DEBUG_MATRICES "Additional checks even in Release (Only in matrix classes)" "OFF")
MARK_AS_ADVANCED(FORCE MRPT_ALWAYS_CHECKS_DEBUG_MATRICES)

# ---------------------------------------------------------------------------
# Create the minimum list of libraries required by an application that
#  uses the MRPT C++ library. This must be passed to "TARGET_LINK_LIBRARIES"
# NOTE: For the compilers MSVC* & Borland most of the libraries are
#        automatically included through "#pragma"'s ;-)
# ---------------------------------------------------------------------------
IF(NOT MSVC)
	# In Linux, link against librt
	# Jerome Monceaux : 2011/03/08
	# exclude rt from apple osx linking
	IF(UNIX AND NOT APPLE)
		APPEND_MRPT_LIBS(rt)
	ENDIF()

	# In Windows, link against ole32 & uuid & oleaut32, etc...
	IF(MINGW)
		APPEND_MRPT_LIBS(ole32 uuid oleaut32 gdi32 winmm setupapi ws2_32)
	ENDIF()
ENDIF()


SET(CMAKE_MRPT_HAS_GZ_STREAMS ${CMAKE_MRPT_HAS_ZLIB})   # We have gz-streams is we have zlib:

include(cmakemodules/script_kinect.cmake REQUIRED)  # Kinect support in a set of different ways
include(cmakemodules/script_pcl.cmake REQUIRED)     # PCL library
include(cmakemodules/script_suitesparse.cmake REQUIRED)     # SuiteSparse libs

# Save the basic list of libraries (without MRPT-libs):
# -------------------------------------------------------------------------------------
SET(MRPTLIB_LINKER_LIBS ${MRPT_LINKER_LIBS} ${MRPT_FFMPEG_LIBS_TO_LINK})
IF($ENV{VERBOSE})
	MESSAGE(STATUS "MRPTLIB_LINKER_LIBS: ${MRPTLIB_LINKER_LIBS}")
ENDIF()

# If we are building .so libs, don't link every program to all the libs:
IF(BUILD_SHARED_LIBS)
	SET(MRPT_LINKER_LIBS "")
ENDIF()

# Save the list of libraries according to the current debug/release build (gcc only)
# -------------------------------------------------------------------------------------
SET(MRPT_LINKER_LIBS_RELorDEB ${MRPT_LINKER_LIBS})

IF(CMAKE_BUILD_TYPE STREQUAL "Debug")
	SET(MRPT_LINKER_LIBS_POSTFIX "${MRPT_DEBUG_POSTFIX}")
ELSE()
	SET(MRPT_LINKER_LIBS_POSTFIX "")
ENDIF()

# Include Asian fonts in utils::CMRPTCanvas ?
SET( MRPT_HAS_ASIAN_FONTS ON CACHE BOOL "Enable Asian fonts in utils::CMRPTCanvas (increases library size).")

include(cmakemodules/script_declare_defines.cmake REQUIRED)  # Transform the variables MRPT_XXX="ON/OFF" to CMAKE_MRPT_XXX="1/0"

include(cmakemodules/script_create_config_h.cmake REQUIRED)   # Build config.h
include(cmakemodules/script_create_version_h.cmake REQUIRED)  # Build version.h

# ---------------------------------------------------------------------------
# The C++ include & link directories:
# ---------------------------------------------------------------------------
INCLUDE_DIRECTORIES(".")
INCLUDE_DIRECTORIES("${MRPT_CONFIG_FILE_INCLUDE_DIR}")

# Add user supplied extra options (optimization, etc...)
SET(USER_EXTRA_CPP_FLAGS "" CACHE STRING "Put extra compiler options here if desired")

# ----------------------------------------------------------------------------
# 			Set the maximum level of warnings:
# ----------------------------------------------------------------------------
SET( MRPT_EXTRA_WARNINGS ON CACHE BOOL "Enable extra warnings apart from -Wall")
MARK_AS_ADVANCED(MRPT_EXTRA_WARNINGS)

# Should be set to true for development
SET( MRPT_WARNINGS_ARE_ERRORS OFF CACHE BOOL "Treat warnings as errors")
MARK_AS_ADVANCED(MRPT_WARNINGS_ARE_ERRORS)

# Whole program optimization?
SET( MRPT_WHOLE_PROGRAM_OPTIMIZATION OFF CACHE BOOL "Flags for whole program optimization.")
MARK_AS_ADVANCED(MRPT_WHOLE_PROGRAM_OPTIMIZATION)

# Even more warnings for different compilers
IF(MRPT_EXTRA_WARNINGS)
   IF (${CMAKE_CXX_COMPILER_ID} STREQUAL "GNU" OR ${CMAKE_CXX_COMPILER_ID} STREQUAL "Clang")
      SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wreturn-type -Wextra -Wtype-limits -Wcast-align -Wparentheses -Wno-unused-parameter") # -Wold-style-cast  -Woverloaded-virtual
   ENDIF()
ENDIF()

# Enable profiling?
SET(MRPT_ENABLE_PROFILING OFF CACHE BOOL "Enable profiling (add -g -pg in GCC/CLANG, /PROFILE in Visual C++)")

IF(MSVC)
	SET(EXTRA_CPP_FLAGS "/W3 /D _CRT_SECURE_NO_DEPRECATE /D _CRT_NONSTDC_NO_DEPRECATE")

	LINK_DIRECTORIES("${CMAKE_BINARY_DIR}/lib")  # Required to find libraries

	IF (NOT BUILD_SHARED_LIBS)
		# static libs in Win: don't optimize out the initializers for class auto registration:
		set(CMAKE_SHARED_LINKER_FLAGS_RELEASE "${CMAKE_SHARED_LINKER_FLAGS_RELEASE} /OPT:NOREF")
		set(CMAKE_SHARED_LINKER_FLAGS_MINSIZEREL "${CMAKE_SHARED_LINKER_FLAGS_MINSIZEREL} /OPT:NOREF")
		set(CMAKE_SHARED_LINKER_FLAGS_RELWITHDEBINFO "${CMAKE_SHARED_LINKER_FLAGS_RELWITHDEBINFO} /OPT:NOREF")
	ENDIF()

	SET( BUILD_WITH_DEBUG_INFO ON CACHE BOOL "Include debug info in binaries")
	MARK_AS_ADVANCED(BUILD_WITH_DEBUG_INFO)
	if(BUILD_WITH_DEBUG_INFO)
		SET(CMAKE_EXE_LINKER_FLAGS_DEBUG "${CMAKE_EXE_LINKER_FLAGS_DEBUG} /DEBUG ")
		SET(CMAKE_MODULE_LINKER_FLAGS_DEBUG "${CMAKE_MODULE_LINKER_FLAGS_DEBUG} /DEBUG ")
	endif()

	# "In earlier versions of Visual C++, the compiler used several discrete
	#   heaps, and each had a finite limit. Currently, the compiler dynamically
	#   grows the heaps as necessary up to a total heap size limit, and requires
	#   a fixed-size buffer only to construct precompiled headers. Consequently,
	#   the /Zm compiler option is rarely necessary."
 	# http://msdn.microsoft.com/en-us/library/bdscwf1c.aspx
	#
	if(MSVC_VERSION LESS 1800) # <=VS 2012
		SET(EXTRA_CPP_FLAGS "${EXTRA_CPP_FLAGS} /Zm1000")
	endif()

	# Whole program optimization
	IF(MRPT_WHOLE_PROGRAM_OPTIMIZATION)
		SET(EXTRA_CPP_FLAGS "${EXTRA_CPP_FLAGS} /GL")
		SET(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} /LTCG ")
		SET(CMAKE_MODULE_LINKER_FLAGS "${CMAKE_MODULE_LINKER_FLAGS} /LTCG ")
		SET(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} /LTCG ")
	ENDIF()

	# Remove unreferenced functions: function level linking
	SET(EXTRA_CPP_FLAGS "${EXTRA_CPP_FLAGS} /Gy")
	SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} /W3 ")

	# SSE2 for MSVC?
	IF (CMAKE_MRPT_HAS_SSE2 AND NOT CMAKE_MRPT_WORD_SIZE EQUAL 64)
		SET(EXTRA_CPP_FLAGS "${EXTRA_CPP_FLAGS} /arch:SSE2")
	ENDIF()

	# SSE3: any flag needed?

	# Fast math:
	#SET(EXTRA_CPP_FLAGS "${EXTRA_CPP_FLAGS} /fp:fast")

	# Profiling?
	IF(MRPT_ENABLE_PROFILING)
		SET(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} /PROFILE ")
		SET(CMAKE_MODULE_LINKER_FLAGS "${CMAKE_MODULE_LINKER_FLAGS} /PROFILE ")
	ENDIF()
ENDIF()

# GNU GCC options ================================
IF(CMAKE_COMPILER_IS_GNUCXX)
	# Wall & pedantic?
	SET(MRPT_BUILD_GCC_PEDANTIC_DEFAULT "OFF")
	SET( MRPT_BUILD_GCC_PEDANTIC ${MRPT_BUILD_GCC_PEDANTIC_DEFAULT} CACHE BOOL "Enable pedantic error detection (with GCC only)")
	MARK_AS_ADVANCED(MRPT_BUILD_GCC_PEDANTIC)

	# High level of warnings.
	# The -Wno-long-long is required in 64bit systems when including sytem headers.
	# The -Wno-variadic-macros was needed for Eigen3, StdVector.h
 	SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${CMAKE_CONFIGURE_CFLAGS} -Wall -Wno-long-long -Wno-variadic-macros")
  IF(NOT APPLE)
    # This causes the option "-Wnorite-strings" to be set on gcc-4.9 on OS X
    SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-write-strings")
  ENDIF()
 	SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${CMAKE_CONFIGURE_CFLAGS}")

	# Use "modern" C99 ! ;-)
 	SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -std=c99")


	IF(MRPT_BUILD_GCC_PEDANTIC)
		SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pedantic")
		# No need to be pendantic in old C files, most of them from 3rd parties anyway...
 		#SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -pedantic")
	ENDIF()

	# Enable C++11 features in GCC 4.7+
	IF (NOT CMAKE_MRPT_GCC_VERSION LESS 47)
		SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
	ELSEIF(NOT CMAKE_MRPT_GCC_VERSION LESS 46)
		SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
	ENDIF ()

	# Parallel stdlibc++?
	IF(MRPT_ENABLE_LIBSTD_PARALLEL_MODE)
		SET(EXTRA_CPP_FLAGS "${EXTRA_CPP_FLAGS} -D_GLIBCXX_PARALLEL -fopenmp")
	ENDIF()
ENDIF()

# CLang options ================================
if (${CMAKE_CXX_COMPILER_ID} STREQUAL "Clang")
	SET( MRPT_BUILD_CLANG_PEDANTIC OFF CACHE BOOL "Enable pedantic error detection (clang)")
	MARK_AS_ADVANCED(MRPT_BUILD_CLANG_PEDANTIC)

	# High level of warnings.
	# no-unused-private-field: clang seems to complain in templates without reason.
 	SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${CMAKE_CONFIGURE_CFLAGS} -Wall -Wno-unused-private-field")
 	SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${CMAKE_CONFIGURE_CFLAGS}")

	# Use "modern" C99 ! ;-)
 	SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -std=c99")

	# Use modern C++11
 	SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

	IF(NOT APPLE) # any reason for not using it in all platforms?
	# Use the libstdc++ lib vs. libc++, to avoid some build errors in MacOS
		SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -stdlib=libstdc++")
		SET(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -stdlib=libstdc++")
		SET(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} -stdlib=libstdc++")
	ENDIF()

	# This is to fix the compilation of C++ templates
	SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fdelayed-template-parsing")

endif()

# Shared options between GCC and CLANG:
# ======================================
if (${CMAKE_CXX_COMPILER_ID} STREQUAL "Clang" OR CMAKE_COMPILER_IS_GNUCXX)
	IF(MRPT_WARNINGS_ARE_ERRORS)
		SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Werror")
		SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Werror")
	ENDIF()

	IF(CMAKE_BUILD_TYPE MATCHES "Debug")
		SET(EXTRA_CPP_FLAGS "${EXTRA_CPP_FLAGS} -g")
		ADD_DEFINITIONS( -D_DEBUG)
		ADD_DEFINITIONS( -DDEBUG)
	ENDIF()

	# Profiling?
	IF(MRPT_ENABLE_PROFILING)
		SET(EXTRA_CPP_FLAGS "${EXTRA_CPP_FLAGS} -pg -g")
	ELSEIF()
		# Remove unreferenced functions: function level linking
		# Remove unreferenced functions: function level linking
		if(NOT APPLE)
			SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -ffunction-sections")
		endif()
	ENDIF()

	# Whole program optimization
	IF(MRPT_WHOLE_PROGRAM_OPTIMIZATION)
		SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} --combine")
		SET(MRPT_EXES_CXX_FLAGS "${MRPT_EXES_CXX_FLAGS} -fwhole-program --combine")
	ENDIF()


	# "-mtune=native" generates code optimized for the detected current processor.
	if (WIN32 OR "${CMAKE_MRPT_ARCH}" MATCHES ".*86" OR "${CMAKE_MRPT_ARCH}" MATCHES "amd64")
		set(opt_native_def ON)
	endif ()

	SET( MRPT_OPTIMIZE_NATIVE ${opt_native_def} CACHE BOOL "GCC/clang optimizations for current processor (-mtune=native). Requires GCC 4.2+")

	IF(CMAKE_BUILD_TYPE STREQUAL "Debug")  # Not in debug!
		SET(MRPT_OPTIMIZE_NATIVE OFF)
	ENDIF()

	IF(CMAKE_MRPT_USE_DEB_POSTFIXS OR CMAKE_MRPT_IS_RPM_PACKAGE)
		# If we're building a Debian package, just DO NOT use -mtune=native
		SET(EXTRA_CPP_FLAGS ${CFLAGS})
	ELSE()
		IF(MRPT_OPTIMIZE_NATIVE AND NOT CMAKE_BUILD_TYPE STREQUAL "Debug")
			SET(EXTRA_CPP_FLAGS "${EXTRA_CPP_FLAGS} -O3 -mtune=native ")
		ENDIF()
	ENDIF()

	# "-ffast-math"
	SET( MRPT_OPTIMIZE_FFAST-MATH OFF CACHE BOOL "GCC/clang optimization for floating math (-ffast-math).")
	IF(MRPT_OPTIMIZE_FFAST-MATH)
		SET(EXTRA_CPP_FLAGS "${EXTRA_CPP_FLAGS} -ffast-math")
	ENDIF()

	# SSE2?
	IF (CMAKE_MRPT_HAS_SSE2)
		SET(EXTRA_CPP_FLAGS "${EXTRA_CPP_FLAGS} -msse2 -funroll-loops")
		IF (CMAKE_COMPILER_IS_GNUCXX)
			SET(EXTRA_CPP_FLAGS "${EXTRA_CPP_FLAGS} -mfpmath=sse")
		ENDIF()
	ENDIF()

	# SSE3?
	IF (CMAKE_MRPT_HAS_SSE3)
		SET(EXTRA_CPP_FLAGS "${EXTRA_CPP_FLAGS} -msse3 -mssse3")
	ENDIF()

	# SSE4*?
	IF (CMAKE_MRPT_HAS_SSE4_1)
		SET(EXTRA_CPP_FLAGS "${EXTRA_CPP_FLAGS} -msse4.1")
	ENDIF()
	IF (CMAKE_MRPT_HAS_SSE4_2)
		SET(EXTRA_CPP_FLAGS "${EXTRA_CPP_FLAGS} -msse4.2")
	ENDIF()
	IF (CMAKE_MRPT_HAS_SSE4_A)
		SET(EXTRA_CPP_FLAGS "${EXTRA_CPP_FLAGS} -msse4a")
	ENDIF()

endif ()

# Add user supplied extra options (optimization, etc...)
# ==========================================================
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${EXTRA_CPP_FLAGS} ${USER_EXTRA_CPP_FLAGS}")
SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${EXTRA_CPP_FLAGS} ${USER_EXTRA_CPP_FLAGS}")


# Some tricks for MSVC:
if(MSVC)
	STRING(REGEX REPLACE "/EHsc" "/EHa" CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}")
	SET(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} /GS-")

	if (COMPILE_IN_PARALLEL)
		SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /MP")  # Multi-CPU parallel compilation (Suggested by  robert.schattschneide)
	endif ()

	# For MSVC to avoid the C1128 error about too large object files:
	SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /bigobj")

endif()

IF(CMAKE_COMPILER_IS_GNUCXX AND NOT APPLE)
	# --as-needed: Avoid unnecesary libraries (.so dependencies):
	#SET(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -Wl,--as-needed")
	#SET(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} -Wl,--as-needed")
ENDIF()

IF($ENV{VERBOSE})
	MESSAGE(STATUS "Final CMAKE_CXX_FLAGS: ${CMAKE_CXX_FLAGS}")
	MESSAGE(STATUS "Final CMAKE_EXE_LINKER_FLAGS: ${CMAKE_EXE_LINKER_FLAGS}")
	MESSAGE(STATUS "Final CMAKE_SHARED_LINKER_FLAGS: ${CMAKE_SHARED_LINKER_FLAGS}")
	MESSAGE(STATUS "Final MRPTLIB_LINKER_LIBS: ${MRPTLIB_LINKER_LIBS}")
ENDIF()

# Save libs and executables in the same place
SET( LIBRARY_OUTPUT_PATH ${MRPT_BINARY_DIR}/lib CACHE PATH "Output directory for libraries" )
SET( EXECUTABLE_OUTPUT_PATH ${MRPT_BINARY_DIR}/bin CACHE PATH "Output directory for applications" )
SET( MEX_LIBRARY_OUTPUT_PATH ${MRPT_BINARY_DIR}/mex/+mrpt CACHE PATH "Output directory for mex functions" )
SET( MEX_EXECUTABLE_OUTPUT_PATH ${MRPT_BINARY_DIR}/mex/test CACHE PATH "Output directory for executable mexs" )

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
	ELSE()
		# Normal packages
		SET(libmrpt_dev_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}/libmrpt-dev/usr/")
	ENDIF()

	SET(mrpt_apps_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}/mrpt-apps/usr/")
	SET(mrpt_doc_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}/mrpt-doc/usr/")
	SET(mrpt_common_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}/mrpt-common/usr/")
	SET(mrpt_pkgconfig_INSTALL_PREFIX "/usr")	# Values when building a Debian package
ELSE()
	# Values under normal conditions -----------------------
	SET(libmrpt_dev_INSTALL_PREFIX "")
	SET(mrpt_apps_INSTALL_PREFIX "")
	SET(mrpt_doc_INSTALL_PREFIX "")
	SET(mrpt_common_INSTALL_PREFIX "")
	SET(mrpt_pkgconfig_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}") 	# Values under normal conditions
ENDIF()

# Remove /STACK:10000000 set by CMake. This value for stack size
# is very high, limiting the number of threads we can spawn.
# Default value used by Windows is 1MB which is good enough.
# By: Sahn Lam, @ CMake mailing list
STRING(REGEX REPLACE "/STACK:[0-9]+" "/STACK:1000000" CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS}")
STRING(REGEX REPLACE "/STACK:[0-9]+" "/STACK:1000000" CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS}")


# This will become a list with all libraries to be built, and their
#  dependencies stored in "mrpt-${name}_LIB_DEPS"
SET(ALL_MRPT_LIBS "" CACHE INTERNAL "")  # This emulates global vars

# ----------------------------------------------------------------------------
#      					PROCESS SUBDIRECTORIES:
# ----------------------------------------------------------------------------
add_subdirectory(otherlibs)   # The third-party libraries
add_subdirectory(libs)        # The MRPT C++ libraries:

SET( BUILD_APPLICATIONS ON CACHE BOOL "If you only want the MRPT libraries, disable this.")
IF(BUILD_APPLICATIONS)
	add_subdirectory(apps)    # The applications:
ENDIF()

IF(BUILD_MATLAB)
        add_subdirectory(mex/apps)# The MEX applications
ENDIF()

# Generate .h to locate MRPT sources:
CONFIGURE_FILE("${CMAKE_SOURCE_DIR}/parse-files/mrpt_paths_config.h.in" "${MRPT_CONFIG_FILE_INCLUDE_DIR}/mrpt/mrpt_paths_config.h")

# Documentation targets (must be AFTER "apps" because it uses the aux program "mrpt-perfdata2html")
add_subdirectory(doc)

#   UNIT TESTS:
# ----------------------------------------------------------------------------
SET( BUILD_TESTING ON CACHE BOOL "Build MRPT tests")
IF( BUILD_TESTING)
	# Try using libgtest (Google testing library) from the system, if available
	IF(${CMAKE_CXX_COMPILER_ID} STREQUAL "Clang" AND APPLE)
		add_definitions(-DGTEST_USE_OWN_TR1_TUPLE=1)
	ENDIF()
	add_subdirectory(${CMAKE_MRPT_GTEST_SRC_DIR})  # Build gtest from sources
	add_subdirectory(tests)  # Build my tests
ENDIF()

include(cmakemodules/script_create_MRPTConfig.cmake REQUIRED)  # Generate the MRPTConfig.cmake & configure files

#-----------------------------------
# The python bindings
# *Note*: This must be AFTER the generation of the MRPTConfig.cmake files
include(cmakemodules/script_python_bindings.cmake REQUIRED)     # Support for python

# Prepare CPack params for building binary packages (has to be after the apps/)
include(cmakemodules/script_setup_cpack.cmake REQUIRED)

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
# *Note*: This must be AFTER the generation of the MRPTConfig.cmake files
#-----------------------------------
add_definitions(-DMRPT_OPENCV_SRC_DIR="${MRPT_OPENCV_SRC_DIR}")
SET(MRPT_DIR ${MRPT_BINARY_DIR})
include(cmakemodules/DefineExamples.cmake REQUIRED)

SET(IS_DEBIAN_DBG_PKG 0)
IF (CMAKE_MRPT_USE_DEB_POSTFIXS AND CMAKE_BUILD_TYPE STREQUAL "Debug")
	SET(IS_DEBIAN_DBG_PKG 1)
ENDIF ()

# Build list of files to install, packages, etc.
include(cmakemodules/script_install_commands.cmake REQUIRED)

# Summary
include(cmakemodules/script_show_final_summary.cmake REQUIRED)
