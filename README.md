The MRPT project
====================================================
[![Travis](https://travis-ci.org/MRPT/mrpt.png?branch=master)](https://travis-ci.org/MRPT/mrpt)
[![Shippable](http://150.214.150.101:50000/projects/5940153f75d24b06002327b7/badge?branch=master)](http://150.214.150.101:50001/github/MRPT/mrpt)
[![Coverage Badge](http://150.214.150.101:50000/projects/5940153f75d24b06002327b7/coverageBadge?branch=master)](http://150.214.150.101:50001/github/MRPT/mrpt)
[![Releases](https://img.shields.io/github/release/MRPT/mrpt.svg)](https://github.com/MRPT/mrpt/releases)
[![Gitter](https://badges.gitter.im/Join%20Chat.svg)](https://gitter.im/MRPT/mrpt)

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
  * [C++ API reference](http://reference.mrpt.org/)
  * ROS packages: [`mrpt_navigation`](http://wiki.ros.org/mrpt_navigation), [`mrpt_slam`](http://wiki.ros.org/mrpt_slam)
  * [Bindings documentation](https://github.com/MRPT/mrpt/wiki) (Python, Matlab)
  * Source code for [dozens of examples](http://www.mrpt.org/tutorials/mrpt-examples/)
  * Example configuration files for  MRPT applications can be found at:
     [MRPT/share/mrpt/config_files](https://github.com/MRPT/mrpt/tree/master/share/mrpt/config_files)
  * Some sample datasets are stored in:
     [MRPT/share/mrpt/datasets](https://github.com/MRPT/mrpt/tree/master/share/mrpt/datasets).
    A more complete dataset repository is [available online](http://www.mrpt.org/robotics_datasets).
  * [How to contribute](https://github.com/MRPT/mrpt/blob/master/.github/CONTRIBUTING.md) with your code for new feaures, bug fixes, etc.

## 3. Compiling

Minimum compiler requisites:
  * Ubuntu 16.04 default gcc/clang versions, or
  * Windows: Visual Studio 2015/2017.

To build in Debian/Ubuntu follow the steps below. See [full build docs](http://www.mrpt.org/Building_and_Installing_Instructions) online
for Windows instructions or to learn all the details.

  * Install minimum recommended dependencies:

```bash
sudo apt install build-essential pkg-config cmake libwxgtk3.0-dev \
libopencv-dev libeigen3-dev libgtest-dev
```

  * Install additional dependencies to enable all MRPT features:

```bash
sudo apt install libftdi-dev freeglut3-dev zlib1g-dev libusb-1.0-0-dev \
libudev-dev libfreenect-dev libdc1394-22-dev libavformat-dev libswscale-dev \
libassimp-dev libjpeg-dev   libsuitesparse-dev libpcap-dev liboctomap-dev
```

  * Build with `cmake` as usual:

```bash
mkdir build && cd build
cmake ..
make
```

## 4. License
MRPT is released under the [new BSD license](http://www.mrpt.org/License/).
