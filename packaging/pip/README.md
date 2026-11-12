# MRPT for Python

The [Mobile Robot Programming Toolkit (MRPT)](https://www.mrpt.org/) from
Python: SE(2)/SE(3) poses and their uncertainty, Lie groups, point clouds and
metric maps (occupancy grids, voxels...), particle filters, rawlog datasets,
3D scenes, and more.

```bash
pip install mrpt3
```

```python
from mrpt.poses import CPose3D

p = CPose3D(1.0, 2.0, 0.0, 0.5, 0.0, 0.0)  # x, y, z, yaw, pitch, roll
print(p + p)
```

Each MRPT module is a Python subpackage (`mrpt.poses`, `mrpt.maps`,
`mrpt.slam`...), with type stubs for IDE completion.

Requirements: Linux (glibc 2.28 or newer), x86_64 or aarch64, Python 3.10 to
3.14. `mrpt.opengl` and `mrpt.gui` also need the system OpenGL and X11
libraries, normally present on desktops. On minimal systems or containers,
install them with:

```bash
# Debian/Ubuntu:
sudo apt install libegl1 libgl1 libgles2 libopengl0 \
  libx11-6 libx11-xcb1 libxcursor1 libxext6 libxi6 libxinerama1 libxrandr2 libxrender1 libxxf86vm1
# Fedora/RHEL:
sudo dnf install mesa-libGL mesa-libEGL libglvnd-gles libglvnd-opengl \
  libX11 libX11-xcb libXcursor libXext libXi libXinerama libXrandr libXrender libXxf86vm
```

All other modules work without them.
See the [examples](https://github.com/MRPT/mrpt/tree/develop/mrpt_examples_py)
and the [documentation](https://docs.mrpt.org/).

## 3D windows

`mrpt.gui.CDisplayWindowGUI` shows a 3D scene in a window, with a mouse
controlled camera. `spinOnce()` processes its events and repaints it without
blocking, so it fits in any loop:

```python
import time
from mrpt.gui import CDisplayWindowGUI
from mrpt.viz import Scene, CGridPlaneXY, stock_objects

win = CDisplayWindowGUI("Demo", 800, 600)
scene = Scene()
scene.insert(CGridPlaneXY())
scene.insert(stock_objects.CornerXYZ(1.0))
win.background_scene = scene

while win.spinOnce():  # False once the window is closed
    time.sleep(0.02)
```

The wxWidgets based windows (`CDisplayWindow3D`) are not included in the wheel.

## C++ SDK

The wheel also contains the MRPT C++ headers, shared libraries and CMake config
files, so other Python packages with C++ code can build against it. With
scikit-build-core, `find_package(mrpt_poses)` works with no extra setup;
otherwise, add `python -c "import mrpt.sdk; print(mrpt.sdk.get_cmake_prefix_path())"`
to `CMAKE_PREFIX_PATH`. C++ extensions that exchange MRPT objects with these
bindings must use the pybind11 version in `mrpt.sdk.PYBIND11_VERSION`. C++ code
using the `mrpt_gui` or `mrpt_opengl` headers also needs the system OpenGL
development headers (`libgl-dev` on Debian/Ubuntu, `mesa-libGL-devel` on
Fedora).

## Other ways to install MRPT

MRPT is also available as Debian/Ubuntu packages and ROS 2 packages; see the
[installation instructions](https://docs.mrpt.org/reference/latest/download-mrpt.html).
Avoid mixing this wheel with those Python bindings in the same environment.
