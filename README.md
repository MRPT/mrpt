<div align="center">

# Mobile Robot Programming Toolkit

**Two decades of open-source robotics — since 2005**

[![Linux build](https://github.com/MRPT/mrpt/actions/workflows/build-linux.yml/badge.svg)](https://github.com/MRPT/mrpt/actions/workflows/build-linux.yml)
[![macOS build](https://github.com/MRPT/mrpt/actions/workflows/build-macos.yml/badge.svg)](https://github.com/MRPT/mrpt/actions/workflows/build-macos.yml)
[![Windows build](https://github.com/MRPT/mrpt/actions/workflows/build-windows.yml/badge.svg)](https://github.com/MRPT/mrpt/actions/workflows/build-windows.yml)
[![codecov](https://codecov.io/gh/MRPT/mrpt/branch/develop/graph/badge.svg)](https://codecov.io/gh/MRPT/mrpt)
[![clang-format](https://github.com/MRPT/mrpt/actions/workflows/check-clang-format.yml/badge.svg)](https://github.com/MRPT/mrpt/actions/workflows/check-clang-format.yml)

[![Latest release](https://img.shields.io/github/release/MRPT/mrpt.svg)](https://github.com/MRPT/mrpt/releases)
[![Windows installer](https://img.shields.io/badge/Windows-Installer-orange?logo=Windows)](https://github.com/MRPT/mrpt/releases/tag/Windows-nightly-builds)
[![DOI](https://zenodo.org/badge/13708826.svg)](https://zenodo.org/doi/10.5281/zenodo.10595286)
[![License](https://img.shields.io/badge/License-BSD_3--Clause-blue.svg)](https://www.mrpt.org/License/)

[**Documentation**](https://docs.mrpt.org/reference/latest/) · [**Tutorials**](https://docs.mrpt.org/reference/latest/tutorials.html) · [**API Reference**](https://docs.mrpt.org/reference/latest/modules.html) · [**Examples**](https://docs.mrpt.org/reference/latest/examples.html) · [**Applications**](https://docs.mrpt.org/reference/latest/applications.html)

</div>

---

## What is MRPT?

MRPT is a well-established C++ framework for mobile robotics, originally developed in 2005 at the University of Málaga
and actively maintained to this day. Over the years, contributions from the community have evolved it into a robust,
modular toolkit widely used in both academic research and industry.

**MRPT 3.0** is a major refactoring that modernises the build system (colcon-based modular packages),
cleans up the public API, and aligns with modern C++17 standards
while preserving the data structures and algorithms at the core of MRPT.

<img align="right" src="https://mrpt.github.io/imgs/mrpt-videos-mix2.gif" alt="MRPT demo" width="340"/>

### Key capabilities

| Area | What's included |
|---|---|
| **SLAM** | MonteCarlo Localization, RBPF, pose-graph optimization |
| **Geometry** | SE(2)/SE(3) Lie groups, rigid body transforms, point clouds |
| **Probabilistic** | Kalman filters, particle filters, PDFs over poses & maps |
| **Maps** | Occupancy grids, point maps, landmark maps, voxel maps |
| **Sensors** | LiDAR, stereo/RGB-D cameras, IMU, GPS, Velodyne, … |
| **Visualization** | 3-D scene graph (`mrpt::viz`), GUI windows, plot widgets |
| **Navigation** | Reactive nav, path planning, PTG-based obstacle avoidance |
| **Python** | pybind11 bindings for all major modules |

---

## Getting started

### Install (Ubuntu/Debian)

```bash
sudo apt install libmrpt-dev mrpt-apps
```

Check your distro version at [Ubuntu](https://packages.ubuntu.com/search?keywords=mrpt&searchon=sourcenames) / [Debian](https://qa.debian.org/madison.php?package=mrpt).
For a more recent build, use the nightly PPA:

```bash
sudo add-apt-repository ppa:joseluisblancoc/mrpt
sudo apt install libmrpt-dev mrpt-apps
```

### Build from source

```bash
git clone https://github.com/MRPT/mrpt.git --recursive
```

See the full [build guide](https://docs.mrpt.org/reference/latest/compiling.html) for colcon, CMake options, and dependency setup.

### Windows

[Nightly Windows installer](https://github.com/MRPT/mrpt/releases/tag/Windows-nightly-builds) — includes `.exe`, `.dll`, `.h`, and `.lib` files.

### ROS

```bash
# ROS 1
sudo apt install ros-$ROS_DISTRO-mrpt2

# ROS 2 — see https://github.com/MRPT/mrpt_ros
```

---

## Using MRPT 3.0 in your project

```cmake
cmake_minimum_required(VERSION 3.16)
project(my_robot_app)

find_package(mrpt_poses REQUIRED)
find_package(mrpt_maps  REQUIRED)

add_executable(my_app main.cpp)
target_link_libraries(my_app mrpt::mrpt_poses mrpt::mrpt_maps)
```

> **Migrating from MRPT 2.x?** See the [porting guide](https://docs.mrpt.org/reference/latest/porting_mrpt3.html)
> — package names changed from `mrpt-<name>` to `mrpt_<name>` and targets from `mrpt::<name>` to `mrpt::mrpt_<name>`.

---

## Documentation highlights

| Resource | Link |
|---|---|
| Full API reference | [docs.mrpt.org/reference/latest/modules.html](https://docs.mrpt.org/reference/latest/modules.html) |
| Tutorials | [docs.mrpt.org/reference/latest/tutorials.html](https://docs.mrpt.org/reference/latest/tutorials.html) |
| Code examples | [docs.mrpt.org/reference/latest/examples.html](https://docs.mrpt.org/reference/latest/examples.html) |
| Python examples | [docs.mrpt.org/reference/latest/python_examples.html](https://docs.mrpt.org/reference/latest/python_examples.html) |
| GUI applications | [docs.mrpt.org/reference/latest/applications.html](https://docs.mrpt.org/reference/latest/applications.html) |
| Supported sensors | [docs.mrpt.org/reference/latest/supported-sensors.html](https://docs.mrpt.org/reference/latest/supported-sensors.html) |
| ROS wrappers | [docs.mrpt.org/reference/latest/wrappers.html](https://docs.mrpt.org/reference/latest/wrappers.html) |
| Porting from 2.x | [docs.mrpt.org/reference/latest/porting_mrpt3.html](https://docs.mrpt.org/reference/latest/porting_mrpt3.html) |
| Changelog | [docs.mrpt.org/reference/latest/page_changelog.html](https://docs.mrpt.org/reference/latest/page_changelog.html) |
| Robotics datasets | [mrpt.org/robotics_datasets](https://www.mrpt.org/robotics_datasets) |

---

## Modules

MRPT 3.0 is organized as independent colcon packages. Each can be built and used standalone:

`mrpt_core` · `mrpt_math` · `mrpt_poses` · `mrpt_obs` · `mrpt_maps` · `mrpt_slam` ·
`mrpt_viz` · `mrpt_gui` · `mrpt_nav` · `mrpt_bayes` · `mrpt_graphs` · `mrpt_io` ·
`mrpt_img` · `mrpt_config` · `mrpt_serialization` · `mrpt_random` · `mrpt_system` ·
`mrpt_tfest` · `mrpt_kinematics` · `mrpt_comms` · `mrpt_hwdrivers` · …

---

## Community & support

- **Questions**: [Stack Overflow `[mrpt]`](https://stackoverflow.com/search?q=mrpt) or [mrpt-users mailing list](https://groups.google.com/forum/#!forum/mrpt-users)
- **Bugs / features**: [GitHub Issues](https://github.com/MRPT/mrpt/issues)
- **Contributing**: see [CONTRIBUTING.md](https://github.com/MRPT/mrpt/blob/master/.github/CONTRIBUTING.md)
- **MOLA SLAM**: MRPT is the core library of the [MOLA modular SLAM framework](https://github.com/MOLAorg/mola/)

---

## License

Released under the [BSD 3-Clause License](https://www.mrpt.org/License/).

---

## Contributors

<a href="https://github.com/MRPT/mrpt/graphs/contributors">
  <img src="https://contrib.rocks/image?repo=MRPT/mrpt" alt="Contributors"/>
</a>

---

## Packaging status

![Repology](https://repology.org/badge/vertical-allrepos/mrpt.svg)
