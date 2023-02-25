\page nav_rrt_planning_example Example: nav_rrt_planning_example

This example demonstrates how to install a custom OpenGL
shader replacing one of MRPT default ones.

In particular, the fragment shader is modified such that
depth (raw depth, in opengl internal logarithmic scale)
with respect to the eye is visualized as grayscale levels.



![nav_rrt_planning_example screenshot](doc/source/images/nav_rrt_planning_example_screenshot.png)
C++ example source code:
\include nav_rrt_planning_example/test.cpp
