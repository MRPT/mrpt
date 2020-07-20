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

Quick ``apt install`` lists
~~~~~~~~~~~~~~~~~~~~~~~~~~~~

**Minimum** recommended requisites:

.. code-block:: bash

   sudo apt install build-essential pkg-config cmake libwxgtk3.0-dev \
        libwxgtk3.0-gtk3-dev libopencv-dev libeigen3-dev libgtest-dev

**Recommended additional** packages to enable most MRPT features
(except ROS bridges):

.. code-block:: bash

   sudo apt install libftdi-dev freeglut3-dev zlib1g-dev \
        libusb-1.0-0-dev libudev-dev libfreenect-dev libdc1394-22-dev \
        libavformat-dev libswscale-dev libassimp-dev libjpeg-dev \
        libsuitesparse-dev libpcap-dev liboctomap-dev libglfw3-dev


Install additional dependencies for ros1bridge using official Ubuntu
repositories. If you already have a ROS distribution installed,
doing ``source /opt/ros/xxx/setup.bash`` is enough, no further packages
must be installed. Do not install these packages if you do not need
the `mrpt::ros1bridge <group_mrpt_ros1bridge_grp.html>`_ module.

.. code-block:: bash

   sudo apt install libcv-bridge-dev libgeometry-msgs-dev libnav-msgs-dev \
        librosbag-storage-dev libroscpp-dev libsensor-msgs-dev \
		libstd-srvs-dev libstereo-msgs-dev libtf2-dev \
		libtf2-msgs-dev libbz2-dev


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

- ``BUILD_APPLICATIONS`` : By default ON, if unchecked the applications won’t be built. Useful if you only want to build MRPT libraries. Notice that you can also always use the MRPT_BUILD_DIR/libs/MRPT_ALL_LIB.* solution (or Makefile) to achieve the same.
- ``BUILD_xSENS``: Whether to use the CMT library for interfacing xSens inertial sensors. Default is ON.
- ``BUILD_EXAMPLES``: Whether you want to compile all the examples in the “/samples” directory. Default is OFF.
- ``BUILD_KINECT``: By default ON. Uncheck if you don’t have the required dependencies (read above for your OS).
- ``BUILD_SHARED_LIBS``: Build static libraries if set to OFF, or dynamic libraries (.so/.dll) otherwise. Default is ON, and it’s strongly recommended to always use shared libs unless you have special need for static ones.
- ``EIGEN_USE_EMBEDDED_VERSION``: By default O, instructs MRPT to use the Eigen headers in MRPT/otherlibs/eigen3/. Uncheck if you have Eigen installed in the system and it’s visible through pkg-config. It’s recommended to uncheck this option if you have eigen3 installed in your system (today, eigen3 it’s not yet in any distro repository, that’s why it’s ON by default).
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
