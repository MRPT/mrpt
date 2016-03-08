The MRPT project 
====================================================
[![Build Status](https://travis-ci.org/MRPT/mrpt.png?branch=master)](https://travis-ci.org/MRPT/mrpt)
[![BSD3 License](http://img.shields.io/badge/license-BSD3-brightgreen.svg)](https://github.com/MRPT/mrpt/blob/master/doc/LICENSE.txt) 
[![Join the chat at https://gitter.im/MRPT/mrpt](https://badges.gitter.im/Join%20Chat.svg)](https://gitter.im/MRPT/mrpt)
[![GitHub release](https://img.shields.io/github/release/MRPT/mrpt.svg)](https://github.com/MRPT/mrpt/releases)

1. Introduction
------------------------------------------------------------------------------

Mobile Robot Programming Toolkit (MRPT) provides C++ libraries aimed at researchers
in mobile robotics and computer vision. Libraries include [SLAM solutions](http://www.mrpt.org/List_of_SLAM_algorithms), [3D(6D) geometry](http://www.mrpt.org/tutorials/programming/maths-and-geometry/2d_3d_geometry/), [SE(2)/SE(3) Lie groups](http://ingmec.ual.es/~jlblanco/papers/jlblanco2010geometry3D_techrep.pdf), 
[probability density functions (pdfs)](http://reference.mrpt.org/stable/classmrpt_1_1utils_1_1_c_probability_density_function.html) over points, landmarks, poses and maps, 
Bayesian inference ([Kalman filters](http://www.mrpt.org/Kalman_Filters), [particle filters](http://www.mrpt.org/tutorials/programming/statistics-and-bayes-filtering/particle_filters/)), [image processing](http://www.mrpt.org/tutorials/programming/images-image-processing-camera-models/), [obstacle avoidance](http://www.mrpt.org/Obstacle_avoidance), [etc](http://reference.mrpt.org/devel/modules.html). 
MRPT also provides GUI apps for [Stereo camera calibration](http://www.mrpt.org/list-of-mrpt-apps/application-kinect-stereo-calib/), [dataset inspection](http://www.mrpt.org/list-of-mrpt-apps/rawlogviewer/), 
and [much more](http://www.mrpt.org/list-of-mrpt-apps/).

[![ReactiveNav2D](https://raw.githubusercontent.com/MRPT/mrpt/master/doc/video-screenshots/reactivenav2d.jpg)](https://www.youtube.com/watch?v=qSMs5rnmwTM&index=9&list=PLOJ3GF0x2_eX7TXG_ZyDgEIJdNcuB193X) [![6D-EKF-SLAM](https://raw.githubusercontent.com/MRPT/mrpt/master/doc/video-screenshots/ekf6d.jpg)](https://www.youtube.com/watch?v=eOkzd_i_-ps&index=6&list=PLOJ3GF0x2_eX7TXG_ZyDgEIJdNcuB193X)
[![ReactiveNav3D](https://raw.githubusercontent.com/MRPT/mrpt/master/doc/video-screenshots/reactivenav3d.jpg)](https://www.youtube.com/watch?v=CXdZ4CjWFuU&index=2&list=PLOJ3GF0x2_eX7TXG_ZyDgEIJdNcuB193X) [![NavLogViewer](https://raw.githubusercontent.com/MRPT/mrpt/master/doc/video-screenshots/navlogviewer.jpg)](https://www.youtube.com/watch?v=6Dp-tr7yR1k&list=PLOJ3GF0x2_eXj35SYVa4-n2-E_zW6klfR&index=2)

2. Resources
------------------------------------------------------------------------------
  * Download the latest unstable code with: `git clone https://github.com/MRPT/mrpt.git --depth 1`
  * Ask questions at: [this Google group](http://www.mrpt.org/forum/) or at [stackoverflow](http://stackoverflow.com/search?q=mrpt) (please, use the tag `mrpt`!)
  * [Main project website](http://www.mrpt.org/), including [sources and Windows installer downloads](http://www.mrpt.org/download-mrpt/)
  * [C++ API reference](http://reference.mrpt.org/)
  * [Bindings documentation](https://github.com/MRPT/mrpt/wiki) (Python, Matlab)
  * Source code for [dozens of examples](http://www.mrpt.org/tutorials/mrpt-examples/)
  * Example configuration files for  MRPT applications can be found at: 
     [MRPT/share/mrpt/config_files](https://github.com/MRPT/mrpt/tree/master/share/mrpt/config_files)
  * Some sample datasets are stored in: 
     [MRPT/share/mrpt/datasets](https://github.com/MRPT/mrpt/tree/master/share/mrpt/datasets). 
    A more complete dataset repository is [available online](http://www.mrpt.org/robotics_datasets).
  * [How to contribute](https://github.com/MRPT/mrpt/blob/master/.github/CONTRIBUTING.md) with your code for new feaures, bug fixes, etc.

3. Compiling
------------------------------------------------------------------------------

### 3.1 **FIVE SECONDS INSTRUCTIONS**

1.  Invoke `cmake-gui` and select: 
      * Where source is          --> MRPT source root directory
      * Where to build binaries  --> Pick a new, empty directory.

    If your platform doesn't support cmake-gui, open a console, chdir to a new 
    empty directory and execute:
    
    	$ ccmake <PATH_TO_MRPT_SOURCES>

2. Within `cmake-gui` (or `ccmake`), set all the build options as 
   you desire, then press "Configure" and "Generate". To build 
   the examples, enable `BUILD_SAMPLES`.

3. Build with the IDE / compiler you selected from CMake (Visual Studio, GNU Make,...)


### 3.2. Further details (RECOMMENDED!)

Read the [compiling instructions](http://www.mrpt.org/Building_and_Installing_Instructions).


4. License
------------------------------------------------------------------------------

MRPT is released under the [new BSD license](http://www.mrpt.org/License/).

