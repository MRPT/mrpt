The MRPT project 
====================================================

[![Build Status](https://travis-ci.org/MRPT/mrpt.png?branch=master)](https://travis-ci.org/MRPT/mrpt)
[![GitHub release](https://img.shields.io/github/release/MRPT/mrpt.svg)](https://github.com/MRPT/mrpt/releases)
[![BSD3 License](http://img.shields.io/badge/license-BSD3-brightgreen.svg)](https://github.com/MRPT/mrpt/blob/master/doc/LICENSE.txt) 

1. Introduction
------------------------------------------------------------------------------

Mobile Robot Programming Toolkit (MRPT) provides C++ libraries aimed at researchers
in mobile robotics and computer vision. Libraries include 3D(6D) geometry, SE(2)/SE(3) Lie groups, 
probability density functions (pdfs) over points, landmarks, poses and maps, 
Bayesian inference (Kalman filters, particle filters), image processing, obstacle
avoidance, etc. 
MRPT also provides GUI apps for Stereo camera calibration, dataset inspection, 
and much more. 

2. Resources
------------------------------------------------------------------------------
  * Main website [http://www.mrpt.org/](http://www.mrpt.org/)
  * C++ API reference: [http://reference.mrpt.org/](http://reference.mrpt.org/).
  * Google group for questions: [http://www.mrpt.org/forum/](http://www.mrpt.org/forum/)
  * [Bindings documentation](https://github.com/MRPT/mrpt/wiki) (Python, Matlab)
  * Download the latest unstable code with: 
    
            git clone https://github.com/MRPT/mrpt.git

  * Example configuration files for  MRPT applications can be found at: 
     [MRPT/share/mrpt/config_files](https://github.com/MRPT/mrpt/tree/master/share/mrpt/config_files)

  * Some sample datasets are stored in: 
     [MRPT/share/mrpt/datasets](https://github.com/MRPT/mrpt/tree/master/share/mrpt/datasets). 
    A more complete dataset repository is [available online](http://www.mrpt.org/robotics_datasets).


3. Compiling
------------------------------------------------------------------------------

### 3.1 **FIVE SECONDS INSTRUCTIONS**

1.  Invoke cmake-gui and select: 
      * Where source is          --> MRPT source root directory
      * Where to build binaries  --> Pick a new, empty directory.

    If your platform doesn't support cmake-gui, open a console, chdir to a new 
    empty directory and execute:
    
    	$ ccmake <PATH_TO_MRPT_SOURCES>

2. Within cmake-gui (or ccmake), set all the build options as 
   you desire, then press "Configure" and "Generate". To build 
   the examples, enable "BUILD_SAMPLES".

3. Build with the IDE / compiler you selected from CMake (Visual Studio, GNU Make,...)


### 3.2. Further details (RECOMMENDED!)

Read the [compiling instructions](http://www.mrpt.org/Building_and_Installing_Instructions).


4. License
------------------------------------------------------------------------------

MRPT is released under the [new BSD license](http://www.mrpt.org/License/).

