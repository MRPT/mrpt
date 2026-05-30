#!/usr/bin/env python3
"""
mrpt_opengl_example.py -- Off-screen rendering with mrpt.opengl.

Demonstrates:
  - CFBORender: off-screen OpenGL framebuffer renderer
  - CFBORenderParameters: configuration struct

NOTE: Actual rendering requires an OpenGL/EGL context.
      This example only exercises object construction and parameter access.
"""

import mrpt.opengl as ogl

# ---------------------------------------------------------------------------
# CFBORenderParameters
# ---------------------------------------------------------------------------
params = ogl.CFBORenderParameters(640, 480)
params.raw_depth = False
print(f"FBO params: {params.width}x{params.height}, raw_depth={params.raw_depth}")

# ---------------------------------------------------------------------------
# CFBORender construction
# ---------------------------------------------------------------------------
fbo = ogl.CFBORender(640, 480)
print(f"CFBORender: {fbo.width()}x{fbo.height()}")
print(f"  hasCameraOverride: {fbo.hasCameraOverride()}")

# Clear any override
fbo.clearCameraOverride()
print(f"  after clearCameraOverride: {fbo.hasCameraOverride()}")

print("\nAll mrpt.opengl examples passed.")
