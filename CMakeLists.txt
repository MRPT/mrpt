# ----------------------------------------------------------------------------
# Root CMake file for the MRPT libraries and applications
#
#  Run with "cmake ." at the root directory to build the makefiles for
#   the MRPT C++ library, the samples, and the applications. Some scripts
#   for generating the documentation, etc. are also updated.
#
#  For compiling instructions for all compilers and platforms, see
#   https://docs.mrpt.org/reference/latest/compiling.html
#
#  2007-2021, Jose Luis Blanco <jlblanco@ual.es>
# ----------------------------------------------------------------------------

if (POLICY CMP0048)
  # cmake warns if loaded from a min-3.x-required parent dir:
  cmake_policy(SET CMP0048 NEW)
endif()

# Tell CMake we'll use both C & C++ for use in its tests/flags.
project(MRPT LANGUAGES C CXX)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
	set(default_build_type "Release")
	message(STATUS "Setting build type to '${default_build_type}' as none was specified.")
	set(CMAKE_BUILD_TYPE "${default_build_type}" CACHE
		STRING "Choose the type of build." FORCE)
	# Set the possible values of build type for cmake-gui
	set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS
	"Debug" "Release" "MinSizeRel" "RelWithDebInfo" "Coverage")
endif()

# -------------------------
#        Setup CMake
# -------------------------
# Minimum version required for:
# - target_compile_features() => 3.8.0
cmake_minimum_required(VERSION 3.8)

# CheckIncludeFile: Yes, we will set "CMAKE_REQUIRED_LIBRARIES".
if (POLICY CMP0075)
	cmake_policy(SET CMP0075 NEW)
endif()

if ((NOT IS_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/3rdparty/nanogui/include")
    OR (NOT IS_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/3rdparty/rplidar_sdk/sdk/sdk")
   )
  message(FATAL_ERROR "git submodules missing! "
    "You probably did not clone the project with --recursive. It is possible to recover "
    "by calling:\n  git submodule update --init --recursive\n")
endif()

# Display timing information for each compiler instance on screen
option(CMAKE_TIMING_VERBOSE "Enable the display of timing information for each compiler instance." OFF)
mark_as_advanced(CMAKE_TIMING_VERBOSE)

# Enable verbose timing display?
if(CMAKE_TIMING_VERBOSE AND UNIX)
  set_property(GLOBAL PROPERTY RULE_MESSAGES OFF)
  set_property(GLOBAL PROPERTY RULE_LAUNCH_COMPILE "${CMAKE_CURRENT_SOURCE_DIR}/cmakemodules/custom_output.sh")
endif(CMAKE_TIMING_VERBOSE AND UNIX)

# Detect wordsize:
if(CMAKE_SIZEOF_VOID_P EQUAL 8)  # Size in bytes!
	set(CMAKE_MRPT_WORD_SIZE 64)
else()
	set(CMAKE_MRPT_WORD_SIZE 32)
endif()

include(cmakemodules/script_version_number.cmake REQUIRED)	#  Loads MRPT version number

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmakemodules") # Directory for CMake scripts

# -------------------------
#            CodeCov
# -------------------------
enable_testing()
find_package(codecov)
# -------------------------

option(MRPT_ALLOW_LGPLV3 "Allows MRPT to use LGPLV3 code (mrpt-vision-lgpl)" "OFF")

option(MRPT_EXCEPTIONS_WITH_CALL_STACK "Report callstack upon exceptions" "ON")
set(MRPT_EXCEPTIONS_CALL_STACK_MAX_DEPTH "50" CACHE STRING "Maximum number of stack levels to show upon exceptions.")
mark_as_advanced(MRPT_EXCEPTIONS_CALL_STACK_MAX_DEPTH)

# The root directory for all MRPT libraries/modules:
set(MRPT_LIBS_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/libs" CACHE INTERNAL "")  # This emulates global vars

# ----- Useful macros ------
include(cmakemodules/UtilsMacros.cmake REQUIRED)
include(cmakemodules/DebugMacros.cmake REQUIRED)
include(cmakemodules/FilterFileLists.cmake REQUIRED)
include(cmakemodules/DeclareMRPTLib.cmake REQUIRED)
include(cmakemodules/DeclareMEXLib.cmake REQUIRED)
include(cmakemodules/DeclareAppDependencies.cmake REQUIRED)
include(cmakemodules/script_detect_unix_arch.cmake REQUIRED) # Detect machine architecture, on UNIX
# --------------------------

# Avoid the need for DLL export/import macros in Windows:
if (WIN32)
	set(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS  ON)
endif()

# ------------------------------------------
# We want libraries to be named "libXXX"
#  in all compilers, and "-dbg" for debug
# ------------------------------------------
if(MSVC)
	set(MRPT_LIB_PREFIX "lib")  # Libs are: "libXXX"
endif()
set(CMAKE_DEBUG_POSTFIX  "-dbg")

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
# Default: dynamic libraries (except if building to JavaScript code)
if ("${CMAKE_SYSTEM_NAME}" STREQUAL "Emscripten")
	set(_def_value OFF)
else()
	set(_def_value ON)
endif()
set(BUILD_SHARED_LIBS ${_def_value} CACHE BOOL "Build shared libraries (.dll/.so) instead of static ones (.lib/.a)")
unset(_def_value)

if(BUILD_SHARED_LIBS)
	set(CMAKE_MRPT_BUILD_SHARED_LIB "#define MRPT_BUILT_AS_DLL")
	set(CMAKE_MRPT_BUILD_SHARED_LIB_ONOFF 1)
else()
	set(CMAKE_MRPT_BUILD_SHARED_LIB "/* #define MRPT_BUILT_AS_DLL */")
	set(CMAKE_MRPT_BUILD_SHARED_LIB_ONOFF 0)
endif()

# Only for emscripten: build executables as .html demo pages:
# =============================================================
if ("${CMAKE_SYSTEM_NAME}" STREQUAL "Emscripten")
	set(CMAKE_EXECUTABLE_SUFFIX ".html")
endif()

# Only for Unix: use pkg-config to find libraries
# ===================================================
include(FindPkgConfig OPTIONAL)

# Group projects in "folders"
set_property(GLOBAL PROPERTY USE_FOLDERS ON)
set_property(GLOBAL PROPERTY PREDEFINED_TARGETS_FOLDER "CMakeTargets")

# Build the RoboPeak Lidar library?
set(DEFAULT_BUILD_ROBOPEAK_LIDAR ON) # Default: Yes
if ((${CMAKE_MRPT_KERNEL} MATCHES "GNU") # It doesnt build in hurd
    OR APPLE # and macos
    OR ("$ENV{DEB_TARGET_ARCH_CPU}" STREQUAL "ppc64")   # fails: termios2
    OR ("$ENV{DEB_TARGET_ARCH_CPU}" STREQUAL "ppc64el") # fails: termios2
    OR ("$ENV{DEB_TARGET_ARCH_CPU}" STREQUAL "powerpc") # fails: termios2
    OR MINGW
    )
	set(DEFAULT_BUILD_ROBOPEAK_LIDAR OFF)
endif()
# Decl option:
set(MRPT_WITH_ROBOPEAK_LIDAR ${DEFAULT_BUILD_ROBOPEAK_LIDAR} CACHE BOOL "Build an embedded version of RoboPeak LIDAR SDK (interface to low-cost lidar)")
if(MRPT_WITH_ROBOPEAK_LIDAR)
	set(CMAKE_MRPT_HAS_ROBOPEAK_LIDAR 1)
else()
	set(CMAKE_MRPT_HAS_ROBOPEAK_LIDAR 0)
endif()

set(MRPT_WITH_OUSTER_LIDAR ON CACHE BOOL "Build support for OUSTER Lidar sensors")

# Create config vars:
set(CMAKE_MRPT_HAS_OUSTER 0)
if(MRPT_WITH_OUSTER_LIDAR)
	set(CMAKE_MRPT_HAS_OUSTER 1)
endif()


#-----------------------------------
#  Build with MRPT-MEX compatibility?
#-----------------------------------
set(MRPT_WITH_MATLAB_WRAPPER OFF CACHE BOOL "Build with compatibility options for MEX wrapper?.")

# GCC only:
# ===================================================
if(CMAKE_COMPILER_IS_GNUCXX)
	# Enable libstdc++ parallel mode?
	set(MRPT_ENABLE_LIBSTD_PARALLEL_MODE OFF CACHE BOOL "Enable parallel mode in libstdc++ (requires GCC 4.2.2+)")
endif()

# Enable precompiled headers:
# ===================================================
if (MSVC)
	set(default_use_precomp ON)
else()
	# JLBC 28th Dec 2017: It seems cotire does not work well with imported -I
	# directories via target_include_directories(... PUBLIC/INTERFACE ...)
	set(default_use_precomp OFF)
endif()
set(MRPT_ENABLE_PRECOMPILED_HDRS ${default_use_precomp} CACHE BOOL "Enable precompiled headers")
mark_as_advanced(MRPT_ENABLE_PRECOMPILED_HDRS)

# MRPT_TRY_START/END blocks
# ===================================================
set(MRPT_HAS_STACKED_EXCEPTIONS ON CACHE BOOL "Enable MRPT_TRY_START/END blocks (disable it for speed up).")

# ASSERT_ blocks
set(MRPT_HAS_ASSERT ON CACHE BOOL "Enable ASSERT_ statements (disable it for speed up).")

# "Classic" function & headers detection:
include(cmakemodules/script_detect_functions_headers.cmake REQUIRED)

# MSVC only:
# ===================================================
if(MSVC)
	# Enable Parallel compilation?
	set(COMPILE_IN_PARALLEL ON CACHE BOOL "Enable parallel compilation in Visual Studio")
endif()

# ----------------------------------------------------------------------------
#   Uninstall target, for "make uninstall"
# Must be invoked *before* other embedded projects so MRPT's target "uninstall" exists first
# ----------------------------------------------------------------------------
configure_file(
  "${CMAKE_CURRENT_SOURCE_DIR}/parse-files/cmake_uninstall.cmake.in"
  "${CMAKE_CURRENT_BINARY_DIR}/cmake_uninstall.cmake"
  IMMEDIATE @ONLY)

add_custom_target(uninstall
  "${CMAKE_COMMAND}" -P "${CMAKE_CURRENT_BINARY_DIR}/cmake_uninstall.cmake")

set_target_properties(uninstall PROPERTIES FOLDER "CMakeTargets")

# See docs for CMake FindThreads()
set(THREADS_PREFER_PTHREAD_FLAG ON)
find_package(Threads) # Defines imported target Threads::Threads

# ccache:
if(NOT MSVC AND NOT XCODE_VERSION)
    option(MRPT_BUILD_WITH_CCACHE "Use ccache compiler cache" ON)
    find_program(CCACHE_FOUND ccache)
	mark_as_advanced(CCACHE_FOUND)
    if(CCACHE_FOUND)
        if(MRPT_BUILD_WITH_CCACHE)
            set_property(GLOBAL PROPERTY RULE_LAUNCH_COMPILE ccache)
            set_property(GLOBAL PROPERTY RULE_LAUNCH_LINK ccache)
        else()
            set_property(GLOBAL PROPERTY RULE_LAUNCH_COMPILE "")
            set_property(GLOBAL PROPERTY RULE_LAUNCH_LINK "")
        endif()
    endif()
endif()

# ----------------------------------------------------------------------------
# Other sub-scripts:
# ----------------------------------------------------------------------------
include(cmakemodules/script_assimp.cmake REQUIRED)      # Check for system assimp lib (3D models)
include(cmakemodules/script_bfd.cmake REQUIRED)      # BFD library for debug symbols for stack backtraces
include(cmakemodules/script_clang_tidy.cmake REQUIRED)  # Clang tidy
include(cmakemodules/script_duo3d.cmake REQUIRED)       # Support for DUO3D Camera
include(cmakemodules/script_eigen.cmake REQUIRED)       # Eigen3
include(cmakemodules/script_ffmpeg.cmake REQUIRED)      # Check for ffmpeg C libraries: libavcodec, libavutil, libavformat, libswscale
include(cmakemodules/script_flycapture2.cmake REQUIRED) # Check for PointGreyResearch (PGR) FlyCapture2 library
include(cmakemodules/script_ftdi.cmake REQUIRED)        # Check for the FTDI headers (Linux only, in win32 we use built-in header & dynamic DLL load):
include(cmakemodules/script_gcc_clang_id.cmake REQUIRED) # Helper variables
include(cmakemodules/script_gl_glut.cmake REQUIRED)     # Check for the GL,GLUT libraries
include(cmakemodules/script_gridmap_options.cmake REQUIRED) # Gridmap options
include(cmakemodules/script_gtest.cmake REQUIRED)       # Unit testing lib
include(cmakemodules/script_inotify.cmake REQUIRED)     # Check for the sys/inotify headers (Linux only, in win32 we use the equivalent API for file system monitoring):
include(cmakemodules/script_isense.cmake REQUIRED)      # Support for INTERSENSE Sensors
include(cmakemodules/script_iwyu.cmake REQUIRED)        # Include-what-you-use
include(cmakemodules/script_jpeg.cmake REQUIRED)        # Check for jpeg
include(cmakemodules/script_jsoncpp.cmake REQUIRED)     # Check for jsoncpp
include(cmakemodules/script_kinect.cmake REQUIRED)      # Kinect support in a set of different ways
include(cmakemodules/script_libdc1394.cmake REQUIRED)   # Check for libdc1394-2
include(cmakemodules/script_libfyaml.cmake REQUIRED)    # Defines embedded version of libfyaml
include(cmakemodules/script_liblas.cmake REQUIRED)      # Check for the LAS LiDAR format library
include(cmakemodules/script_libtclap.cmake REQUIRED)    # Check for system libtclap
include(cmakemodules/script_matlab.cmake REQUIRED)      # Support for Matlab MEX functions generation
include(cmakemodules/script_mynteye.cmake REQUIRED)     # Support for MYNT EYE cameras
include(cmakemodules/script_nanogui.cmake REQUIRED)     # Check for nanogui
include(cmakemodules/script_nanoflann.cmake REQUIRED)   # Check for nanoflann
include(cmakemodules/script_national_instruments.cmake REQUIRED)  # NI C library
include(cmakemodules/script_nite2.cmake REQUIRED)       # Check for NITE2 library
include(cmakemodules/script_octomap.cmake REQUIRED)     # Check for the octomap library
include(cmakemodules/script_opencv.cmake REQUIRED)      # Check for the OpenCV libraries (via pkg-config, CMake, with different options)
include(cmakemodules/script_openni2.cmake REQUIRED)     # Check for the OpenNI2 library
include(cmakemodules/script_pcap.cmake REQUIRED)        # Check for the libpcap library
include(cmakemodules/script_phidget.cmake REQUIRED)     # Support for phidget Interface Kit with proximity sensor device :
include(cmakemodules/script_qt.cmake REQUIRED)          # Check for wxWidgets + GL
include(cmakemodules/script_ros.cmake REQUIRED)        # Check for ROS1 / ROS2
include(cmakemodules/script_SIMD.cmake REQUIRED)        # SSE2/SSE3/... optimization options
include(cmakemodules/script_simpleini.cmake REQUIRED)   # SimpleINI lib
include(cmakemodules/script_suitesparse.cmake REQUIRED) # SuiteSparse libs
include(cmakemodules/script_swissrange.cmake REQUIRED)  # Support for SWISSRANGE 3D camera:
include(cmakemodules/script_tinyxml2.cmake REQUIRED)    # tinyxml2 lib
include(cmakemodules/script_triclops.cmake REQUIRED)    # Check for PointGreyResearch (PGR) Triclops library
include(cmakemodules/script_videre_svs.cmake REQUIRED)  # Support for Videre Design stereo camera:
include(cmakemodules/script_wxwidgets.cmake REQUIRED)   # Check for wxWidgets + GL
include(cmakemodules/script_xsens.cmake REQUIRED)       # XSens Motion trackers / IMU drivers
include(cmakemodules/script_zlib.cmake REQUIRED)        # Check for zlib

include(cmakemodules/process_emscripten_embedded_files.cmake REQUIRED)

# ---------------------------------------------------------------------------
#			OPTIONS
#The options for the user when using "cmakesetup" or "ccmake":
# ---------------------------------------------------------------------------
option(MRPT_ALWAYS_CHECKS_DEBUG "Additional checks even in Release" "OFF")
mark_as_advanced(FORCE MRPT_ALWAYS_CHECKS_DEBUG)
option(MRPT_ALWAYS_CHECKS_DEBUG_MATRICES "Additional checks even in Release (Only in matrix classes)" "OFF")
mark_as_advanced(FORCE MRPT_ALWAYS_CHECKS_DEBUG_MATRICES)

# Include Asian fonts in CMRPTCanvas ?
set( MRPT_HAS_ASIAN_FONTS ON CACHE BOOL "Enable Asian fonts in CMRPTCanvas (increases library size).")

include(cmakemodules/script_declare_defines.cmake REQUIRED)  # Transform the variables MRPT_XXX="ON/OFF" to CMAKE_MRPT_XXX="1/0"

# ---------------------------------------------------------------------------
# The C++ include & link directories:
# ---------------------------------------------------------------------------
# Add user supplied extra options (optimization, etc...)
set(USER_EXTRA_CPP_FLAGS "" CACHE STRING "Put extra compiler options here if desired")

# Should be set to true for development
set( MRPT_WARNINGS_ARE_ERRORS OFF CACHE BOOL "Treat warnings as errors")
mark_as_advanced(MRPT_WARNINGS_ARE_ERRORS)

# Whole program optimization?
set( MRPT_WHOLE_PROGRAM_OPTIMIZATION OFF CACHE BOOL "Flags for whole program optimization.")
mark_as_advanced(MRPT_WHOLE_PROGRAM_OPTIMIZATION)

# Enable profiling?
set(MRPT_ENABLE_PROFILING OFF CACHE BOOL "Enable profiling (add -g -pg in GCC/CLANG, /PROFILE in Visual C++)")

if(MSVC)
	add_compile_options(/W3)
	add_definitions(-D_CRT_SECURE_NO_DEPRECATE)
	add_definitions(-D_CRT_NONSTDC_NO_DEPRECATE)
	add_definitions(-D_SILENCE_ALL_CXX17_DEPRECATION_WARNINGS)

	link_directories("${CMAKE_BINARY_DIR}/lib")  # Required to find libraries

	if (NOT BUILD_SHARED_LIBS)
		# static libs in Win: don't optimize out the initializers for class auto registration:
		set(CMAKE_SHARED_LINKER_FLAGS_RELEASE "${CMAKE_SHARED_LINKER_FLAGS_RELEASE} /OPT:NOREF")
		set(CMAKE_SHARED_LINKER_FLAGS_MINSIZEREL "${CMAKE_SHARED_LINKER_FLAGS_MINSIZEREL} /OPT:NOREF")
		set(CMAKE_SHARED_LINKER_FLAGS_RELWITHDEBINFO "${CMAKE_SHARED_LINKER_FLAGS_RELWITHDEBINFO} /OPT:NOREF")
	endif()

	set( BUILD_WITH_DEBUG_INFO ON CACHE BOOL "Include debug info in binaries")
	mark_as_advanced(BUILD_WITH_DEBUG_INFO)
	if(BUILD_WITH_DEBUG_INFO)
		set(CMAKE_EXE_LINKER_FLAGS_DEBUG "${CMAKE_EXE_LINKER_FLAGS_DEBUG} /DEBUG ")
		set(CMAKE_MODULE_LINKER_FLAGS_DEBUG "${CMAKE_MODULE_LINKER_FLAGS_DEBUG} /DEBUG ")
	endif()

	# Whole program optimization
	if(MRPT_WHOLE_PROGRAM_OPTIMIZATION)
		add_compile_options(/GL)
		set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} /LTCG ")
		set(CMAKE_MODULE_LINKER_FLAGS "${CMAKE_MODULE_LINKER_FLAGS} /LTCG ")
		set(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} /LTCG ")
	endif()

	# Remove unreferenced functions: function level linking
	add_compile_options(/Gy)

	# Profiling?
	if(MRPT_ENABLE_PROFILING)
		set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} /PROFILE ")
		set(CMAKE_MODULE_LINKER_FLAGS "${CMAKE_MODULE_LINKER_FLAGS} /PROFILE ")
	endif()
endif()

# GNU GCC options ================================
if(CMAKE_COMPILER_IS_GNUCXX)
	# Since MRPT 2.0 (March 2018) we need gcc-7.{1,2} due to compiler bugs in older versions:
	if (NOT "${CMAKE_CXX_COMPILER_VERSION}" VERSION_GREATER "7.0")
		message(FATAL_ERROR "MRPT requires gcc-7 or above. Please, see instructions for upgrading in: https://github.com/MRPT/mrpt/blob/master/README.md#32-build-from-sources")
	endif()

	# Wall & pedantic?
	set(MRPT_BUILD_GCC_PEDANTIC_DEFAULT "OFF")
	set(MRPT_BUILD_GCC_PEDANTIC ${MRPT_BUILD_GCC_PEDANTIC_DEFAULT} CACHE BOOL "Enable pedantic error detection (with GCC only)")
	mark_as_advanced(MRPT_BUILD_GCC_PEDANTIC)

	# High level of warnings.
	# The -Wno-long-long is required in 64bit systems when including sytem headers.
	# The -Wno-variadic-macros was needed for Eigen3, StdVector.h
	add_compile_options(-Wall  -Wno-long-long -Wno-variadic-macros -Wshadow)
	add_compile_options(-Wreturn-local-addr   -Werror=return-local-addr)
	add_compile_options(-Wno-psabi)

	# Workaround: Eigen <3.4 produces *tons* of warnings in GCC >=6. See http://eigen.tuxfamily.org/bz/show_bug.cgi?id=1221
	if (NOT ${CMAKE_CXX_COMPILER_VERSION} LESS "6.0" AND "${MRPT_EIGEN_VERSION}" VERSION_LESS "3.4")
		add_compile_options(-Wno-ignored-attributes -Wno-int-in-bool-context)
	endif()

	if(NOT APPLE)
		# This causes the option "-Wnowrite-strings" to be set on gcc-4.9 on OS X
		add_compile_options(-Wno-write-strings)
	endif()

	# Use "modern" C99 ! ;-)
	string(APPEND CMAKE_C_FLAGS " -std=c99")

	if(MRPT_BUILD_GCC_PEDANTIC)
		# Only for C++ sources:
		string(APPEND CMAKE_CXX_FLAGS " -pedantic")

		# No need to be pendantic in old C files, most of them from 3rd parties anyway...
 		#set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -pedantic")
	endif()

	# Parallel stdlibc++?
	if(MRPT_ENABLE_LIBSTD_PARALLEL_MODE)
		add_compile_options(-fopenmp)
		add_definitions(-D_GLIBCXX_PARALLEL)
	endif()

	# BUILD_TYPE: Coverage
	set(CMAKE_CXX_FLAGS_COVERAGE "${CMAKE_CXX_FLAGS_DEBUG} --coverage -fno-inline -fno-inline-small-functions -fno-default-inline")
	set(CMAKE_EXE_LINKER_FLAGS_COVERAGE "${CMAKE_EXE_LINKER_FLAGS_DEBUG} --coverage")
	set(CMAKE_SHARED_LINKER_FLAGS_COVERAGE "${CMAKE_SHARED_LINKER_FLAGS_DEBUG} --coverage")
endif()

# CLang options ================================
if (MRPT_COMPILER_IS_CLANG)
	add_definitions(-D__STRICT_ANSI__) # fixes errors like "support for type '__float128' is not yet implemented"

	# High level of warnings.
	# no-unused-private-field: clang seems to complain in templates without reason.
	add_compile_options(-Wall -Wabi -Wno-unused-private-field -Wshadow-all)
	add_compile_options(-Wreturn-stack-address   -Werror=return-stack-address)

	# Use "modern" C99 ! ;-)
	set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -std=c99")

	if(NOT APPLE AND NOT ("${CMAKE_SYSTEM_NAME}" STREQUAL "Emscripten"))
		# From: https://stackoverflow.com/a/16788372/1631514
		# I would use the native library for each OS i.e. libstdc++ on GNU/Linux and libc++ on Mac OS X.
		# libc++ is not 100% complete on GNU/Linux, and there's no real advantage to using it when libstdc++
		# is more complete. Also, if you want to link to any other libraries written in C++ they will almost certainly have been built with libstdc++ so you'll need to link with that too to use them.
		# Use the libstdc++ lib vs. libc++, to avoid some build errors in MacOS
		set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -stdlib=libstdc++")
		set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -stdlib=libstdc++")
		set(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} -stdlib=libstdc++")
	endif()
endif()

# Shared options between GCC and CLANG:
# ======================================
if (MRPT_COMPILER_IS_GCC_OR_CLANG)
	# Even more warnings:
	add_compile_options(-Wreturn-type         -Werror=return-type)
	add_compile_options(-Wformat              -Werror=format-security)

#	add_compile_options(-Wfloat-conversion    -Werror=float-conversion)

	add_compile_options(-Wextra -Wtype-limits -Wcast-align -Wparentheses)
	add_compile_options(-Wno-unused-parameter)

	if(MRPT_WARNINGS_ARE_ERRORS)
		add_compile_options(-Werror)
	endif()

	if(CMAKE_BUILD_TYPE MATCHES "Debug")
		add_compile_options(-g)
		add_definitions( -D_DEBUG)
		add_definitions( -DDEBUG)
	endif()

	# Profiling?
	if(MRPT_ENABLE_PROFILING)
		add_compile_options(-pg -g)
	elseif()
		# Remove unreferenced functions: function level linking
		# Remove unreferenced functions: function level linking
		if(NOT APPLE)
			add_compile_options(-ffunction-sections)
		endif()
	endif()

	# Whole program optimization
	if(MRPT_WHOLE_PROGRAM_OPTIMIZATION)
		add_compile_options(--combine)
		set(MRPT_EXES_CXX_FLAGS "${MRPT_EXES_CXX_FLAGS} -fwhole-program --combine")
	endif()

    if (NOT CMAKE_CROSSCOMPILING)
    	# "-march=native" generates code optimized for the detected current processor.
    	set(MRPT_OPTIMIZE_NATIVE OFF CACHE BOOL "GCC/clang optimizations for current processor (-march=native)")
    	# "-mtune=native" generates code optimized for the detected current processor.
    	set(MRPT_TUNE_NATIVE ON CACHE BOOL "GCC/clang tune for current processor (-mtune=native)")

		# If enabled, and NOT in an arch that doesn't support the flag:
    	if(MRPT_TUNE_NATIVE AND
			(NOT "${CMAKE_MRPT_ARCH}" STREQUAL "riscv64") AND
			(NOT "${CMAKE_MRPT_ARCH}" STREQUAL "parisc64") AND # hppa
			(NOT "${CMAKE_MRPT_ARCH}" STREQUAL "parisc") # hppa
		)
    		add_compile_options(-mtune=native)
    	endif()

    	if(MRPT_OPTIMIZE_NATIVE)
    		if ("${CMAKE_MRPT_ARCH}" MATCHES "ppc64.*")
    			# Special case: PowerPC does not have match=native:
    			add_compile_options(-mcpu=native -mtune=native)
    		else()
    			# amd64, arm64, etc.
    			add_compile_options(-march=native)
    		endif()
    	endif()
    endif()

	# Was: add_compile_options(-mfpmath=sse)
	# No need to add this flag: it's enabled by default in 64bit builds.

	# SSE2? add flag to entire codebase:
	if (CMAKE_MRPT_HAS_SSE2)
		add_compile_options(-msse2)
	endif()

	# SSE3 and above: only add flags to files with the proper suffix:

endif ()

# Add user supplied extra options (optimization, etc...)
# ==========================================================
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${USER_EXTRA_CPP_FLAGS}")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${USER_EXTRA_CPP_FLAGS}")

# Some tricks for MSVC:
if(MSVC)
	string(REGEX REPLACE "/EHsc" "/EHa" CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}")
	set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} /GS-")

	if (COMPILE_IN_PARALLEL)
		set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /MP")  # Multi-CPU parallel compilation (Suggested by  robert.schattschneide)
	endif ()

	# For MSVC to avoid the C1128 error about too large object files:
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /bigobj")

endif()

if($ENV{VERBOSE})
	message(STATUS "Final CMAKE_CXX_FLAGS: ${CMAKE_CXX_FLAGS}")
	message(STATUS "Final CMAKE_EXE_LINKER_FLAGS: ${CMAKE_EXE_LINKER_FLAGS}")
	message(STATUS "Final CMAKE_SHARED_LINKER_FLAGS: ${CMAKE_SHARED_LINKER_FLAGS}")
endif()

# Save libs and executables in the same place
set( LIBRARY_OUTPUT_PATH ${MRPT_BINARY_DIR}/lib CACHE PATH "Output directory for libraries" )
set( EXECUTABLE_OUTPUT_PATH ${MRPT_BINARY_DIR}/bin CACHE PATH "Output directory for applications" )
set( MEX_LIBRARY_OUTPUT_PATH ${MRPT_BINARY_DIR}/mex/+mrpt CACHE PATH "Output directory for mex functions" )
set( MEX_EXECUTABLE_OUTPUT_PATH ${MRPT_BINARY_DIR}/mex/test CACHE PATH "Output directory for executable mexs" )


# This will become a list with all libraries to be built, and their
#  dependencies stored in "mrpt-${name}_LIB_DEPS"
set(ALL_MRPT_LIBS "" CACHE INTERNAL "")  # This emulates global vars

if (NOT MSVC AND MRPT_ENABLE_PRECOMPILED_HDRS)
	include(cmakemodules/cotire.cmake REQUIRED) # COmpiler TIme REducer helper for PCH
	set_directory_properties(PROPERTIES COTIRE_ADD_UNITY_BUILD FALSE) # Disable unity targets
endif()

# Detect Eigen Alignment to make MRPT compatible.
# EIGEN_MAX_ALIGN_BYTES & EIGEN_MAX_STATIC_ALIGN_BYTES with selected compiler flags
# ------------------------------------------------
# DISABLED! Just go for the safer side: align=32 bytes
#	get_directory_property(mrpt_root_dir_COMPILE_OPTIONS COMPILE_OPTIONS)
#try_run(RUN_EIGEN_RETVAL BUILD_EIGEN_SUCCESS
#	${CMAKE_BINARY_DIR}
#	SOURCES "${CMAKE_CURRENT_SOURCE_DIR}/parse-files/eigen_detect_alignment.cpp"
#	COMPILE_DEFINITIONS ""
#	CMAKE_FLAGS
#	-DINCLUDE_DIRECTORIES:STRING=${MRPT_EIGEN_INCLUDE_DIR}
#	-DCMAKE_CXX_FLAGS:STRING="${CMAKE_CXX_FLAGS} ${mrpt_root_dir_COMPILE_OPTIONS}"
#	OUTPUT_VARIABLE EIGEN_TEST_OUT)
#if (BUILD_EIGEN_SUCCESS AND (RUN_EIGEN_RETVAL EQUAL 0))
#	string(REGEX MATCH "EIGEN_MAX_ALIGN_BYTES[ ]+[0-9]+" EIGEN_MAX_ALIGN_BYTES "${EIGEN_TEST_OUT}")
#	string(REGEX MATCH "[0-9]+" EIGEN_MAX_ALIGN_BYTES "${EIGEN_MAX_ALIGN_BYTES}")
#	string(REGEX MATCH "EIGEN_MAX_STATIC_ALIGN_BYTES[ ]+[0-9]+" EIGEN_MAX_STATIC_ALIGN_BYTES "${EIGEN_TEST_OUT}")
#	string(REGEX MATCH "[0-9]+" EIGEN_MAX_STATIC_ALIGN_BYTES "${EIGEN_MAX_STATIC_ALIGN_BYTES}")
#else()
#message(STATUS "Test Eigen build failed, falling back to default alignment (BUILD_EIGEN_SUCCESS=${BUILD_EIGEN_SUCCESS} RUN_EIGEN_RETVAL=${RUN_EIGEN_RETVAL} BUILD_EIGEN_OUT=${EIGEN_TEST_OUT})")
if (MRPT_ARCH_INTEL_COMPATIBLE)
    set(EIGEN_MAX_ALIGN_BYTES 32)
    set(EIGEN_MAX_STATIC_ALIGN_BYTES 32)
else()
    # for aarch64, etc.
    set(EIGEN_MAX_ALIGN_BYTES 16)
    set(EIGEN_MAX_STATIC_ALIGN_BYTES 16)
endif()
#endif()

include(cmakemodules/script_create_config_h.cmake REQUIRED)   # Build config.h
include(cmakemodules/script_create_version_h.cmake REQUIRED)  # Build version.h

add_custom_target(all_mrpt_libs ALL) # all_mrpt_libs: target to build all mrpt-* modules

# ----------------------------------------------------------------------------
#      					PROCESS SUBDIRECTORIES:
# ----------------------------------------------------------------------------
add_subdirectory(3rdparty)   # The third-party libraries
add_subdirectory(libs)        # The MRPT C++ libraries

set(MRPT_BUILD_APPLICATIONS ON CACHE BOOL "If you only want the MRPT libraries, disable this.")
if(MRPT_BUILD_APPLICATIONS)
	add_subdirectory(apps)    # The applications:
endif()

if(MRPT_WITH_MATLAB_WRAPPER)
    add_subdirectory(mex/apps)# The MEX applications
endif()

# Generate .h to locate MRPT sources:
configure_file(
	"${CMAKE_CURRENT_SOURCE_DIR}/parse-files/mrpt_paths_config.h.in"
	"${MRPT_CONFIG_FILE_INCLUDE_DIR}/mrpt/mrpt_paths_config.h"
	)

# Documentation targets (must be AFTER "apps" because it uses the aux program "mrpt-perfdata2html")
add_subdirectory(doc)

#   UNIT TESTS:
# ----------------------------------------------------------------------------
if (DEFINED BUILD_TESTING)
	set(_def_value ${BUILD_TESTING})
else()
	if ("$ENV{ROS_DISTRO}" STREQUAL "")
		# Regular case:
		set(_def_value ON)
	else()
		# We are in a ROS environment: disable tests by default to reduce build time in ROS build farms.
		set(_def_value OFF)
	endif()
endif()
set(MRPT_BUILD_TESTING ${_def_value} CACHE BOOL "Build MRPT tests")
if(MRPT_BUILD_TESTING)
	add_subdirectory(tests)  # Build my tests
endif()
unset(_def_value )

#-----------------------------------
# The python bindings
# *Note*: This must be AFTER the generation of mrpt-xxx-config.cmake files
include(cmakemodules/script_python_bindings.cmake REQUIRED)     # Support for python

# Prepare CPack params for building binary packages (has to be after the apps/)
include(cmakemodules/script_setup_cpack.cmake REQUIRED)

# ----------------------------------------------------------------------------
#  Hide some variables to the user, just show the important variables:
# ----------------------------------------------------------------------------
mark_as_advanced(FORCE
	CMAKE_BACKWARDS_COMPATIBILITY
	wxWidgets_CONFIGURATION
	wxWidgets_LIB_DIR
	wxWidgets_USE_REL_AND_DBG
	wxWidgets_wxrc_EXECUTABLE
)

# Write MRPT-version file (the global one, each mrpt-xxxx also has its own):
configure_file(
	"${CMAKE_CURRENT_SOURCE_DIR}/parse-files/mrpt-config.cmake"
	"${CMAKE_BINARY_DIR}/mrpt-config.cmake"
	COPYONLY)
write_basic_package_version_file(
	"${CMAKE_BINARY_DIR}/mrpt-config-version.cmake"
	VERSION ${CMAKE_MRPT_FULL_VERSION}
	COMPATIBILITY AnyNewerVersion
)

# Build list of files to install, packages, etc.
include(cmakemodules/script_install_commands.cmake REQUIRED)


# Summary
include(cmakemodules/script_show_final_summary.cmake REQUIRED)

#-----------------------------------
# The examples
# *Note*: This must be AFTER the generation of the mrpt-xxx-config.cmake files
#-----------------------------------
add_definitions(-DMRPT_OPENCV_SRC_DIR="${MRPT_OPENCV_SRC_DIR}")
add_subdirectory(samples)

# evaluate coverage
coverage_evaluate()
