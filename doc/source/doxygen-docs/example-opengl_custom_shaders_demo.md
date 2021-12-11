\page opengl_custom_shaders_demo Example: opengl_custom_shaders_demo

This example demonstrates how to install a custom OpenGL
shader replacing one of MRPT default ones.

In particular, the fragment shader is modified such that
depth (raw depth, in opengl internal logarithmic scale)
with respect to the eye is visualized as grayscale levels.



![opengl_custom_shaders_demo screenshot](doc/source/images/opengl_custom_shaders_demo_screenshot.png)
C++ example source code:
\include opengl_custom_shaders_demo/test.cpp
