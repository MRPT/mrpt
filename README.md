
<h1 align="center">The MRPT project</h1>

<a href="https://github.com/MRPT/mrpt/actions/workflows/build-linux.yml"><img src="https://github.com/MRPT/mrpt/actions/workflows/build-linux.yml/badge.svg" /></a>
<a href="https://github.com/MRPT/mrpt/actions/workflows/build-macos.yml"><img src="https://github.com/MRPT/mrpt/actions/workflows/build-macos.yml/badge.svg" /></a>
<a href="https://github.com/MRPT/mrpt/actions/workflows/build-windows.yml"><img src="https://github.com/MRPT/mrpt/actions/workflows/build-windows.yml/badge.svg" /></a>
<a href="https://ci.appveyor.com/project/jlblancoc/mrpt" alt="AppVeyor:msvc"><img src="https://ci.appveyor.com/api/projects/status/yjs4lpj02f6a1ylg/branch/develop?svg=true" /></a> 
<a href="https://codecov.io/gh/MRPT/mrpt" alt="codecov"><img src="https://codecov.io/gh/MRPT/mrpt/branch/develop/graph/badge.svg" /></a>
[![CI Check clang-format](https://github.com/MRPT/mrpt/actions/workflows/check-clang-format.yml/badge.svg)](https://github.com/MRPT/mrpt/actions/workflows/check-clang-format.yml)

<a href="https://github.com/MRPT/mrpt/releases" alt="Releases"><img src="https://img.shields.io/github/release/MRPT/mrpt.svg" /></a>
<a href='https://github.com/MRPT/mrpt/releases/tag/Windows-nightly-builds'><img src='https://img.shields.io/badge/Windows-Installer-orange?logo=Windows'></a>
<a href="https://github.com/MRPT/mrpt/releases" alt="GitHub"><img src="https://img.shields.io/github/downloads/mrpt/mrpt/total.svg" /></a>
<a href="https://sourceforge.net/projects/mrpt/files/" alt="SourceForge"><img src="https://img.shields.io/sourceforge/dt/mrpt.svg" /></a>
<a href="https://gitter.im/MRPT/mrpt" alt="Gitter"><img src="https://badges.gitter.im/Join%20Chat.svg" /></a>
[![Gitpod ready-to-code](https://img.shields.io/badge/Gitpod-ready--to--code-blue?logo=gitpod)](https://gitpod.io/#https://github.com/MRPT/mrpt)

## 1. Introduction
<img align="right" src="https://mrpt.github.io/imgs/mrpt-videos-mix2.gif">

Mobile Robot Programming Toolkit (MRPT) provides C++ libraries aimed at researchers
in mobile robotics and computer vision. Libraries include [SLAM solutions](https://www.mrpt.org/List_of_SLAM_algorithms), [3D(6D) geometry](https://www.mrpt.org/tutorials/programming/maths-and-geometry/2d_3d_geometry/), [SE(2)/SE(3) Lie groups](https://ingmec.ual.es/~jlblanco/papers/jlblanco2010geometry3D_techrep.pdf),
[probability density functions (pdfs)](https://docs.mrpt.org/reference/develop/classmrpt_1_1math_1_1_c_probability_density_function.html) over points, landmarks, poses and maps,
Bayesian inference ([Kalman filters](https://www.mrpt.org/Kalman_Filters), [particle filters](https://www.mrpt.org/tutorials/programming/statistics-and-bayes-filtering/particle_filters/)), [image processing](https://www.mrpt.org/tutorials/programming/images-image-processing-camera-models/), [obstacle avoidance](https://www.mrpt.org/Obstacle_avoidance), [etc](https://reference.mrpt.org/devel/modules.html).
MRPT also provides GUI apps for [Stereo camera calibration](https://www.mrpt.org/list-of-mrpt-apps/application-kinect-stereo-calib/), [dataset inspection](https://www.mrpt.org/list-of-mrpt-apps/rawlogviewer/),
and [much more](https://www.mrpt.org/list-of-mrpt-apps/).

## 2. Resources
  * Download the latest unstable code with: `git clone https://github.com/MRPT/mrpt.git --depth 1`
  * Ask questions at: [this Google group](https://www.mrpt.org/forum/) or at [stackoverflow](https://stackoverflow.com/search?q=mrpt) (please, use the tag `mrpt`!)
  * [Main project website](https://www.mrpt.org/), including [sources and Windows installer downloads](https://www.mrpt.org/download-mrpt/)
  * [C++ API reference](https://docs.mrpt.org/reference/)
  * ROS packages: [`mrpt_navigation`](https://wiki.ros.org/mrpt_navigation), [`mrpt_slam`](https://wiki.ros.org/mrpt_slam)
  * [Bindings documentation](https://github.com/MRPT/mrpt/wiki) (Python, Matlab)
  * Source code for [dozens of examples](https://www.mrpt.org/tutorials/mrpt-examples/)
  * Example configuration files for  MRPT applications can be found at:
     [MRPT/share/mrpt/config_files](https://github.com/MRPT/mrpt/tree/master/share/mrpt/config_files)
  * Some sample datasets are stored in:
     [MRPT/share/mrpt/datasets](https://github.com/MRPT/mrpt/tree/master/share/mrpt/datasets).
    A more complete dataset repository is [available online](http://www.mrpt.org/robotics_datasets).
  * [How to contribute](https://github.com/MRPT/mrpt/blob/master/.github/CONTRIBUTING.md) with your code for new feaures, bug fixes, etc.
  * MRPT is used in the [MOLA modular SLAM framework](https://github.com/MOLAorg/mola/).

## 3. Install

<a href="https://repology.org/project/mrpt/versions"> 
  <img align="right" src="https://repology.org/badge/vertical-allrepos/mrpt.svg" alt="Packaging status">
</a>

### 3.1. Ubuntu

See [this PPA](https://launchpad.net/~joseluisblancoc/+archive/ubuntu/mrpt) for nightly builds from the `develop` branch, or [this one](https://launchpad.net/~joseluisblancoc/+archive/ubuntu/mrpt-stable) for stable releases.

        sudo add-apt-repository ppa:joseluisblancoc/mrpt   # develop branch
        #sudo add-apt-repository ppa:joseluisblancoc/mrpt-stable   # master (stable releases) branch
        sudo apt install libmrpt-dev mrpt-apps

Supported distributions:
  * Ubuntu 18.04 LTS (Bionic), Ubuntu 20.04 LTS (Focal), or newer.

### 3.2. Build from sources

See [build documentation](https://docs.mrpt.org/reference/latest/compiling.html) ([source](doc/source/compiling.rst)).

### 3.3. Windows precompiled versions

Executables (`.exe`s and `.dll`s) and development libraries (`.h`s and `.lib`s) included:

[Nightly built Windows installer](https://github.com/MRPT/mrpt/releases/tag/Windows-nightly-builds)

### 3.4. As a ROS1/ROS2 package

MRPT is also shipped as a ros1 & ros2 package named `mrpt2`, so it can be installed via: 

```bash
sudo apt install ros-$ROS_DISTRO-mrpt2
```

`mrpt2` status in ROS build farms:

| Distro | `develop` branch  | Stable release |
|---|---|---|
| ROS1 Noetic @ u20.04 | [![Build Status](https://build.ros.org/job/Ndev__mrpt2__ubuntu_focal_amd64/badge/icon)](https://build.ros.org/job/Ndev__mrpt2__ubuntu_focal_amd64/) | [![Build Status](https://build.ros.org/job/Nbin_uF64__mrpt2__ubuntu_focal_amd64__binary/badge/icon)](https://build.ros.org/job/Nbin_uF64__mrpt2__ubuntu_focal_amd64__binary/) |
| ROS2 Rolling @ u22.04 | [![Build Status](https://build.ros2.org/job/Rdev__mrpt2__ubuntu_jammy_amd64/badge/icon)](https://build.ros2.org/job/Rdev__mrpt2__ubuntu_jammy_amd64/) | [![Build Status](https://build.ros2.org/job/Rbin_uJ64__mrpt2__ubuntu_jammy_amd64__binary/badge/icon)](https://build.ros2.org/job/Rbin_uJ64__mrpt2__ubuntu_jammy_amd64__binary/) |

## 4. License
MRPT is released under the [new BSD license](http://www.mrpt.org/License/).


## Stargazers over time

[![Stargazers over time](https://starchart.cc/MRPT/mrpt.svg)](https://starchart.cc/MRPT/mrpt)
