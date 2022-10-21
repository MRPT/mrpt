.. _compiling:

#########
Compiling
#########

This page describes how to build MRPT from sources. If you want instead to
install the binaries and getting quickly into developing your MRPT-based
applications, go to the `download page <download-mrpt.html>`_ and grab the
binaries for your system or install via ``apt`` in Ubuntu.

.. contents:: :local:


1. Dependencies
-----------------

Minimum compiler requisites:

- gcc-7 or newer. Only for Ubuntu 16.04LTS Xenial, you will have to
  follow `these instructions <https://gist.github.com/jlblancoc/99521194aba975286c80f93e47966dc5>`_ for
  installing gcc-7 in this version of Ubuntu.

- clang-4 or newer.
- Windows: Visual Studio 2017 version 15.3 or newer.
- cmake >= 3.3 required (>=3.4 for Windows).
- Eigen >= 3.3 required.
- Other libraries: See :ref:`doxid-dependencies`
- On SIMD optimizations, read: :ref:`doxid-simd`

.. dropdown:: Debian/Ubuntu
    :open:

    **Minimum recommended** requisites:

    .. code-block:: bash
    
       # All Ubuntu versions:
       sudo apt install build-essential pkg-config cmake \
         libopencv-dev libeigen3-dev zlib1g-dev \
         libsuitesparse-dev libjpeg-dev
         
       # plus, only for Ubuntu < 22.10:
       sudo apt install libwxgtk3.0-gtk3-dev

       # plus, only for Ubuntu >= 22.10:
       sudo apt install libwxgtk3.2-dev

    **Recommended additional** packages to enable most MRPT features:

    .. code-block:: bash

       # Build OpenGL graphics, Qt and nanogui GUIs:
       sudo apt install freeglut3-dev libassimp-dev libglfw3-dev \
            libglu1-mesa-dev libqt5opengl5-dev qtbase5-dev \
            libxrandr-dev libxxf86vm-dev

       # Support most common sensors:
       sudo apt install libftdi-dev libusb-1.0-0-dev libudev-dev libfreenect-dev \
            libdc1394-22-dev libavformat-dev libswscale-dev libpcap-dev \
            liboctomap-dev libopenni2-dev

       # Support showing debug information in call stacks upon exceptions:
       sudo apt install binutils-dev libiberty-dev

       # Support using system SimpleINI library (only Ubuntu >=20.04 focal)
       sudo apt install libicu-dev libsimpleini-dev

    If your Ubuntu distribution is old and does not have any of the packages
    above, do not worry and ignore it, MRPT CMake scripts will handle it.

    **ROS1 support:** Install additional dependencies for ros1bridge using
    official Ubuntu repositories. If you already have a ROS distribution installed,
    doing ``source /opt/ros/xxx/setup.bash`` is enough, no further packages
    must be installed. Do not install these packages if you do not need
    the `mrpt::ros1bridge <group_mrpt_ros1bridge_grp.html>`_ module.

    .. code-block:: bash

       sudo apt install libcv-bridge-dev libgeometry-msgs-dev libnav-msgs-dev \
            librosbag-storage-dev libroscpp-dev libsensor-msgs-dev \
    		libstd-srvs-dev libstereo-msgs-dev libtf2-dev \
    		libtf2-msgs-dev libbz2-dev

    **ROS2 support:** Invoke your ROS2 distribution ``setup.bash`` as usual before
    running MRPT cmake configure to enable building of
    the `mrpt::ros2bridge <group_mrpt_ros2bridge_grp.html>`_ module.

.. dropdown:: Windows

    **CMake (Mandatory)**

    Install the CMake build system from `here <https://cmake.org/download/>`_.

    **wxWidgets (Optional, but recommended)**

    In addition to the following notes, read the
    `wxWidgets wiki <https://wiki.wxwidgets.org/Microsoft_Visual_C%2B%2B_Guide>`_.

    From the `latest wxWidgets release <https://github.com/wxWidgets/wxWidgets/releases/latest>`_, download
    either:

    - precompiled binaries: the ``wxWidgets-3.x.x_Headers.7z`` file, and one
      of ``wxMSW-3.x.x-vcXXX_Dev.7z`` or ``wxMSW-3.x.x_gccXXX_Dev.7z`` files
      depending on your compiler, its version and the target architecture
      (x86 if not specified or x64). Unpack both files into the same directory so
      that include and lib directories are at the same level after unpacking.
      and (as ``.zip`` or ``.7z``) from its download page. Decompress it in a directory
      where CMake can easily find it (e.g. ``C:\wxWidgets``); or

    - the source code file ``wxWidgets-3.x.x.7z`` (more work to do, but
      compatibility is ensured with all compilers).

    .. dropdown:: Compile wxWidgets from sources

        Build for 32bit:

        Open the MSVC 32bit command-line prompt (from the start menu -> MSVC -> Visual Studio Tools), do ``cd`` to the ``WXWIDGETS/build/msw`` directory and run:

        .. code-block:: bash

           nmake -f makefile.vc BUILD=release SHARED=1 RUNTIME_LIBS=dynamic DEBUG_INFO=0 VENDOR=mrpt USE_OPENGL=1
           nmake -f makefile.vc BUILD=debug SHARED=1 RUNTIME_LIBS=dynamic DEBUG_INFO=1 VENDOR=mrpt USE_OPENGL=1

        Build for 64bit:

        Open the MSVC 64bit command-line prompt (from the start menu -> MSVC -> Visual Studio Tools), do ``cd`` to the ``WXWIDGETS/build/msw`` directory and run:

        .. code-block:: bash

           nmake -f makefile.vc BUILD=release SHARED=1 RUNTIME_LIBS=dynamic DEBUG_INFO=0 VENDOR=mrpt USE_OPENGL=1 TARGET_CPU=amd64
           nmake -f makefile.vc BUILD=debug SHARED=1 RUNTIME_LIBS=dynamic DEBUG_INFO=1 VENDOR=mrpt USE_OPENGL=1 TARGET_CPU=amd64

    **OpenCV (Optional, but strongly recommended)**

    Download the `latest OpenCV release <https://github.com/opencv/opencv/releases/latest>`_
    either as source code and compile it, or (easier) install the provided
    ``opencv-x.y.z-vcZZ.exe`` installer.

    **FFmpeg for Win32 (Optional)**

    These libraries are optional, you will need them only if you plan to read
    from video files or IP cameras using `mrpt::hwdrivers::CFFMPEG_InputStream <class_mrpt_hwdrivers_CFFMPEG_InputStream.html>`_.

    Directly download and decompress anywhere in your disk the latest Win32 builds
    `from here <https://www.ffmpeg.org/download.html#build-windows>`_.
    Then, when running CMake (cmake-gui) for MRPT, enable ``MRPT_HAS_FFMPEG_WIN32``,
    press “Configure” and then set ``FFMPEG_WIN32_ROOT_DIR`` to the directory where
    FFmpeg binaries have been uncompressed (e.g. ``c:\ffmpeg-r16537-gpl-lshared-win32``).

    The FFmpeg DLLs will be required at runtime by programs compiled with MRPT under Windows,
    so make sure the directory ``FFMPEG/bin`` is in the system PATH.

    **PCL, the Point Cloud Library (Optional)**

    Download, build and install PCL as explained `in the official web <https://pointclouds.org/>`_.

    At present, only a little functionality is provided for interaction of MRPT with PCL (check out the changelogs for details).

    **WinPCap (libpcap for Windows)**

    Used to read/write PCAP files in the Velodyne sensor classes.
    Download and install the WinPCap development packages and set the
    (advanced CMake variables) ``PCAP_INCLUDE_DIR`` and ``PCAP_LIBRARY`` to
    ``WpdPacl/Include`` and ``wpcap.lib``, respectivaly.


2. Build using cmake
----------------------

Using the console
~~~~~~~~~~~~~~~~~~~

The usual cmake stuff:

.. code-block:: bash

  cd mrpt
  mkdir build
  cd build
  cmake ..
  cmake --build .
  # To run tests:
  make test_legacy  # or "make test" to see less details

Using cmake-gui
~~~~~~~~~~~~~~~~~~~

- Open cmake-gui (Available for Windows/Linux) and set the “source dir” to the
  root directory of the MRPT source package you have downloaded.
- Set the “binary directory” to a new, empty directory where to generate the
  project files.
- Press “configure”, check for errors, tune the options as required (read below for a description of some options) and finally click “Generate”.
- Click on "open project" and build as usual.

3. CMake build options
------------------------
For all platforms/compilers:

- ``MRPT_BUILD_APPLICATIONS`` : By default ON, if unchecked the applications won’t be built. Useful if you only want to build MRPT libraries. Notice that you can also always use the MRPT_BUILD_DIR/libs/MRPT_ALL_LIB.* solution (or Makefile) to achieve the same.
- ``BUILD_xSENS``: Whether to use the CMT library for interfacing xSens inertial sensors. Default is ON.
- ``MRPT_BUILD_EXAMPLES``: Whether you want to compile all the examples in the “/samples” directory. Default is OFF.
- ``MRPT_WITH_KINECT``: By default ON. Uncheck if you don’t have the required dependencies (read above for your OS).
- ``BUILD_SHARED_LIBS``: Build static libraries if set to OFF, or dynamic libraries (.so/.dll) otherwise. Default is ON, and it’s strongly recommended to always use shared libs unless you have special need for static ones.
- ``MRPT_EIGEN_USE_EMBEDDED_VERSION``: By default O, instructs MRPT to use the Eigen headers in MRPT/otherlibs/eigen3/. Uncheck if you have Eigen installed in the system and it’s visible through pkg-config. It’s recommended to uncheck this option if you have eigen3 installed in your system (today, eigen3 it’s not yet in any distro repository, that’s why it’s ON by default).
- ``MRPT_ALWAYS_CHECKS_DEBUG``: If set to ON, additional security checks will be performed at run-time in many classes. Default is OFF.
- ``MRPT_ALWAYS_CHECKS_DEBUG_MATRICES``: If set to ON, additional security checks will be performed at run-time in several Matrix operations. Default is ON.
- ``MRPT_ENABLE_EMBEDDED_ENABLED_PROFILER``: If enabled, all code blocks within macros "MRPT_BEGIN/MRPT_END" will be profiled and the statistics dumped to the console at the end of execution of any program. Default is OFF.
- ``MRPT_HAS_ASIAN_FONTS``: Enables Asian fonts in mrpt::img::CCanvas (see this page), but increases library size by 1.5Mb. Default is ON.
- ``MRPT_HAS_SVS``: To enable integration of the Videre SVS libraries to interface their stereo cameras. You’ll need the vendor libraries installed in the system before to enable this option. After setting this option to “ON”, the new configuration fields “SVS_ROOT_DIR” will appear and will be tried to be set pointing to the directory where the library is (As of Aug/2010, this option only works in GNU/Linux).
- ``MRPT_OCCUPANCY_GRID_CELLSIZE``: Can be either 8 or 16 (bits). The size of each cell in the class mrpt::slam::COccupancyGridMap2D. Default is 8 bits. More on this here.

For Windows only:

- ``MRPT_HAS_FFMPEG_WIN32``: Enable this and (after running “Configure”) then
  set FFMPEG_WIN32_ROOT_DIR to the directory where FFmpeg binaries have been
  uncompressed (e.g. “c:\ffmpeg-r16537-gpl-lshared-win32”).
- ``MRPT_HAS_BUMBLEBEE``: To enable integration of the Bumblebee stereo camera SDK. You’ll need the vendor provided “Triclops” and “Digiclops” libraries. After setting this option to “ON”, the new configuration fields “BUMBLEBEE_DIGICLOPS_ROOT_DIR” and “BUMBLEBEE_TRICLOPS_ROOT_DIR” will appear where the correct corresponding paths must be entered.

For GNU GCC compiler only:

- ``MRPT_ENABLE_LIBSTD_PARALLEL_MODE``: Enables the GNU libstdc++ parallel mode (See http://gcc.gnu.org/onlinedocs/libstdc++/manual/parallel_mode.html. Default is OFF.
- ``MRPT_ENABLE_PROFILING``: Enables generation of information required for profiling. Default is OFF.
- ``MRPT_OPTIMIZE_NATIVE``: Enables optimization for the current architecture (-mtune=native). Default is OFF for old GCC versions, ON for 4.2+. If you have an old version of GCC (<4.2), this option cannot be set since it’s not recognized by the compiler. Instead, set USER_EXTRA_CPP_FLAGS to the optimization flags for your platform, for example: -march=pentium4.

4. Special instructions for other compilers
--------------------------------------------

.. dropdown:: MinGW in Windows

    - Install MinGW: Recommended: https://jmeubank.github.io/tdm-gcc/

    - Before compiling MRPT with MinGW, it is strongly recommended to compile
      wxWidgets and OpenCV from sources with MinGW:

      - Build wxWidgets. Open a command prompt and go to the directory ``wxWidgets/build/msw``.
        Then execute the following commands to rebuild the Release and Debug
        configurations (as shared libs), so CMake can correctly detect wxWidgets:

        .. code-block:: bash

           mingw32-make -f makefile.gcc SHARED=1 USE_OPENGL=1 BUILD=release DEBUG_INFO=0 VENDOR=mrpt
           mingw32-make -f makefile.gcc SHARED=1 USE_OPENGL=1 BUILD=debug   DEBUG_INFO=1 VENDOR=mrpt
           mingw32-make -f makefile.gcc SHARED=1 USE_OPENGL=1 BUILD=release DEBUG_INFO=0 VENDOR=mrpt
           mingw32-make -f makefile.gcc SHARED=1 USE_OPENGL=1 BUILD=debug   DEBUG_INFO=1 VENDOR=mrpt

        As usual with make, add a ``-j4`` or any higher number to exploit parallelization.
        For building wxWidgets with MinGW **for 64bit** you will need to add ``TARGET_CPU=amd64`` to the parameters above. Otherwise, even with MinGW64 you will obtain 32bit builds.

      - Build OpenCV. Use its CMake build system, select the MinGW compiler and
        follow the generic OpenCV compilation instructions.

    - Open cmake-gui and select MRPT source directory and an empty target (binary) directory.
      Press configure and in the compilers dialog pick MinGW Makefiles. If you obtain an error like:

      .. code-block::

        CMake Error: CMake was unable to find a build program corresponding to "MinGW Makefiles".
        CMAKE_MAKE_PROGRAM is not set.  You probably need to select a different build tool.

      it means MinGW is not correctly installed in the system. Review the
      installation process described above. If everything goes fine, you will
      see the new CMake variables remarked in red. Go through the normal
      configuration process for MRPT, and when you are satisfied, press Generate.

    - Open a console and in the newly created binary directory, invoke:

    .. code-block::

        mingw32-make

    either by writing the full path (e.g. ``c:\MinGW\bin\mingw32-make``) or by
    adding the “bin” directory of your MinGW installation to the system ``PATH``.
    This should start the normal build process.

.. dropdown:: clang

    - Install clang. In Debian/Ubuntu: ``sudo apt-get install clang``

    - Create an empty build directory and invoke CMake with:

    .. code-block:: bash

       mkdir build && cd build
       CC=/usr/bin/clang CXX=/usr/bin/clang++ cmake ..
       make
