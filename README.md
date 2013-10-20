The MRPT project 
====================================================

1. Introduction
------------------------------------------------------------------------------

Mobile Robot Programming Toolkit (MRPT) includes libraries and tools aimed to help researchers
in the areas of mobile robotics and computer vision in the development of 
efficient implementations with reusability of code as a priority. 
It features classes for easily managing 3D(6D) geometry, probability density 
functions (pdfs) over many predefined variables (points,landmarks,poses,maps,...),
Bayesian inference (Kalman filters, particle filters), image processing, obstacle
avoidance, etc.

2. Resources
------------------------------------------------------------------------------
  * Main website [http://www.mrpt.org/](http://www.mrpt.org/)
  * C++ API reference: [http://reference.mrpt.org/](http://reference.mrpt.org/).
  * Google group for questions: [http://www.mrpt.org/forum/](http://www.mrpt.org/forum/)


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

