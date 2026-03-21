\defgroup mrpt_opengl_grp [mrpt-opengl]

Low-level GPU rendering: shaders, textures, framebuffers, and the rendering
pipeline that draws mrpt::viz scenes.

[TOC]

# Library mrpt-opengl

This library is part of MRPT and can be installed in Debian-based systems with:

		sudo apt install libmrpt-opengl-dev

Read also [how to import MRPT into your CMake scripts](mrpt_from_cmake.html).

## MRPT 3.0 namespace split

In MRPT 3.0 the scene-graph API (3-D objects, scenes, viewports, cameras,
stock objects) moved from `mrpt::opengl` to **mrpt::viz** (library
**mrpt-viz**). See \ref lib_mrpt_viz_grp for the scene-graph documentation
and the full list of rendering primitives.

**What remains in `mrpt::opengl`** is the low-level GPU rendering backend:

- mrpt::opengl::CFBORender — off-screen framebuffer rendering
- mrpt::opengl::Shader / mrpt::opengl::DefaultShaders — shader programs
- mrpt::opengl::Buffer — OpenGL vertex/index buffers
- mrpt::opengl::Texture — texture management
- `#include <mrpt/opengl/opengl_api.h>` — GL function loading

User code that builds 3-D scenes should depend on `mrpt_viz`, not on
`mrpt_opengl` directly. Only code that performs custom GPU rendering
(e.g., off-screen rendering with `CFBORender`) needs `mrpt_opengl`.

See the full list of classes in mrpt::opengl.

# Library contents
