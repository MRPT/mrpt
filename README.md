<h1 align="center">The MRPT project</h1>

gcc/clang: <a href="https://circleci.com/gh/MRPT/mrpt"><img src="https://circleci.com/gh/MRPT/mrpt.svg?style=svg"></a> MSVC: <a href="https://ci.appveyor.com/project/jlblancoc/mrpt" alt="AppVeyor:msvc"><img src="https://ci.appveyor.com/api/projects/status/yjs4lpj02f6a1ylg/branch/develop?svg=true" /></a> Last stable release: <a href="https://github.com/MRPT/mrpt/releases" alt="Releases"><img src="https://img.shields.io/github/release/MRPT/mrpt.svg" /></a> <a href="https://gitter.im/MRPT/mrpt" alt="Gitter"><img src="https://badges.gitter.im/Join%20Chat.svg" /></a>

<a href="https://repology.org/project/mrpt/versions"> <img src="https://repology.org/badge/tiny-repos/mrpt.svg" alt="Packaging status"> </a> <a href="https://codecov.io/gh/MRPT/mrpt" alt="codecov"><img src="https://codecov.io/gh/MRPT/mrpt/branch/develop/graph/badge.svg" /></a>  GH downloads: <a href="https://github.com/MRPT/mrpt/releases" alt="GitHub"><img src="https://img.shields.io/github/downloads/mrpt/mrpt/total.svg" /></a> SF downloads (datasets): <a href="https://sourceforge.net/projects/mrpt/files/" alt="SourceForge"><img src="https://img.shields.io/sourceforge/dt/mrpt.svg" /></a>

## 1. Introduction
<img align="right" src="https://mrpt.github.io/imgs/mrpt-videos-mix2.gif">

Mobile Robot Programming Toolkit (MRPT) provides C++ libraries aimed at researchers
in mobile robotics and computer vision. Libraries include [SLAM solutions](http://www.mrpt.org/List_of_SLAM_algorithms), [3D(6D) geometry](http://www.mrpt.org/tutorials/programming/maths-and-geometry/2d_3d_geometry/), [SE(2)/SE(3) Lie groups](http://ingmec.ual.es/~jlblanco/papers/jlblanco2010geometry3D_techrep.pdf),
[probability density functions (pdfs)](http://reference.mrpt.org/stable/classmrpt_1_1utils_1_1_c_probability_density_function.html) over points, landmarks, poses and maps,
Bayesian inference ([Kalman filters](http://www.mrpt.org/Kalman_Filters), [particle filters](http://www.mrpt.org/tutorials/programming/statistics-and-bayes-filtering/particle_filters/)), [image processing](http://www.mrpt.org/tutorials/programming/images-image-processing-camera-models/), [obstacle avoidance](http://www.mrpt.org/Obstacle_avoidance), [etc](http://reference.mrpt.org/devel/modules.html).
MRPT also provides GUI apps for [Stereo camera calibration](http://www.mrpt.org/list-of-mrpt-apps/application-kinect-stereo-calib/), [dataset inspection](http://www.mrpt.org/list-of-mrpt-apps/rawlogviewer/),
and [much more](http://www.mrpt.org/list-of-mrpt-apps/).

## 2. Resources
  * Download the latest unstable code with: `git clone https://github.com/MRPT/mrpt.git --depth 1`
  * Ask questions at: [this Google group](http://www.mrpt.org/forum/) or at [stackoverflow](http://stackoverflow.com/search?q=mrpt) (please, use the tag `mrpt`!)
  * [Main project website](http://www.mrpt.org/), including [sources and Windows installer downloads](http://www.mrpt.org/download-mrpt/)
  * [C++ API reference](http://docs.mrpt.org/reference/)
  * ROS packages: [`mrpt_navigation`](http://wiki.ros.org/mrpt_navigation), [`mrpt_slam`](http://wiki.ros.org/mrpt_slam)
  * [Bindings documentation](https://github.com/MRPT/mrpt/wiki) (Python, Matlab)
  * Source code for [dozens of examples](http://www.mrpt.org/tutorials/mrpt-examples/)
  * Example configuration files for  MRPT applications can be found at:
     [MRPT/share/mrpt/config_files](https://github.com/MRPT/mrpt/tree/master/share/mrpt/config_files)
  * Some sample datasets are stored in:
     [MRPT/share/mrpt/datasets](https://github.com/MRPT/mrpt/tree/master/share/mrpt/datasets).
    A more complete dataset repository is [available online](http://www.mrpt.org/robotics_datasets).
  * [How to contribute](https://github.com/MRPT/mrpt/blob/master/.github/CONTRIBUTING.md) with your code for new feaures, bug fixes, etc.
  * MRPT is used in the [MOLA modular SLAM framework](https://github.com/MOLAorg/mola/).

## 3. Install

### 3.1. Ubuntu

See [PPA](https://launchpad.net/~joseluisblancoc/+archive/ubuntu/mrpt) for mrpt 2.0 branch (for mrpt 1.5.* [read here](https://github.com/MRPT/mrpt/tree/mrpt-1.5#31-ubuntu-ppa)).

        sudo add-apt-repository ppa:joseluisblancoc/mrpt
        sudo apt-get update
        sudo apt-get install libmrpt-dev mrpt-apps

Supported distributions:
  * Ubuntu 18.04 (Bionic), 18.10 (Cosmic), 19.04 (Disco), 19.10 (Eoan)
  * Ubuntu 16.04LTS Xenial (EOL: April 2021)
  	* Using 16.04 requires installing gcc-7 due to some bugs in gcc-5:

          add-apt-repository ppa:ubuntu-toolchain-r/test
          apt-get update
          apt-get install -y g++-7


### 3.2. Build from sources

Minimum compiler requisites:
  * mrpt >=2.0 (`master` branch):
    * gcc-7 or newer.
      * Ubuntu 16.04LTS Xenial: [Instructions](https://gist.github.com/jlblancoc/99521194aba975286c80f93e47966dc5) for installing gcc-7 in this version of Ubuntu.
      * Ubuntu 18.04 or newer: default gcc version is ok.
    * clang-4 or newer.
    * Windows: Visual Studio 2017 version 15.3 or newer.
    * cmake >= 3.3 required (>=3.4 for Windows).
    * Eigen >= 3.3 required.
  * mrpt 1.5.* and **maintenance** `mrpt-1.5` branch:
    * Ubuntu 14.06LTS Xenial: default gcc/clang versions
    * Windows: Visual Studio 2012
    * cmake >= 3.1 required
    * **EOL** for `mrpt-1.5.*`: EOL of Ubuntu Xenial (April 2021).

To build in Debian/Ubuntu follow the steps below. See [full build docs](http://www.mrpt.org/Building_and_Installing_Instructions) online
for Windows instructions or to learn all the details.

  * Install minimum recommended dependencies:

```bash
sudo apt install build-essential pkg-config cmake libwxgtk3.0-dev libwxgtk3.0-gtk3-dev \
libopencv-dev libeigen3-dev libgtest-dev
```
  MRPT builds against OpenCV 2.4.x, 3.x, 4.x, but it is recommended to use 3.0 or later.


  * Install additional dependencies to enable most MRPT features (except ROS bridges):

```bash
sudo apt install libftdi-dev freeglut3-dev zlib1g-dev libusb-1.0-0-dev \
libudev-dev libfreenect-dev libdc1394-22-dev libavformat-dev libswscale-dev \
libassimp-dev libjpeg-dev   libsuitesparse-dev libpcap-dev liboctomap-dev
```

  * Install additional dependencies for `ros1bridge` using official Ubuntu repositories. 
  If you already have a ROS distribution installed, doing `source /opt/ros/xxx/setup.bash` 
  is enough, no further packages must be installed.

```bash
sudo apt install libcv-bridge-dev libgeometry-msgs-dev libnav-msgs-dev libpcl-conversions-dev librosbag-dev libroscpp-dev libsensor-msgs-dev libstd-srvs-dev libstereo-msgs-dev libtf2-dev libtf2-msgs-dev libbz2-dev
```

  * Build with `cmake` as usual:

```bash
mkdir build && cd build
cmake ..
make
```

## 4. License
MRPT is released under the [new BSD license](http://www.mrpt.org/License/).


## Stargazers over time

[![Stargazers over time](https://starchart.cc/MRPT/mrpt.svg)](https://starchart.cc/MRPT/mrpt)
      
