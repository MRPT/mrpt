\page python_bindings_architecture Python bindings architecture (MRPT 3.0)

# Python bindings architecture — MRPT 3.0

This document describes how the MRPT Python bindings are structured in MRPT 3.0:
which files exist, how they relate to each other, how the CMake build integrates
them, and where to find examples.

---

## Overview

MRPT 3.0 exposes a Python API through [pybind11](https://pybind11.readthedocs.io/).
Each MRPT C++ module provides an optional Python sub-package installed under the
`mrpt` namespace (e.g. `mrpt.poses`, `mrpt.maps`).

The `mrpt` root is an **implicit namespace package** (Python 3.3+, PEP 420 /
PEP 451): there is deliberately **no `mrpt/__init__.py`**.  Python automatically
merges all `mrpt/` directories it finds on `sys.path`, regardless of how many
different install prefixes they come from.  This makes the bindings work
correctly in every deployment scenario without any special build flags:

| Scenario | How it works |
|---|---|
| `colcon build` (default, no merge-install) | `source install/setup.bash` adds each package's prefix to `PYTHONPATH`; Python merges all `mrpt/` dirs automatically |
| `colcon build --merge-install` | All packages share one prefix; single `mrpt/` dir; works trivially |
| Debian / ROS package install | All `.deb` files install to the same `dist-packages/`; single `mrpt/` dir |
| Partial install (only some modules) | Only installed sub-packages appear; no import error for missing ones |
| Multiple MRPT installs (system + local ws) | Both prefixes on `PYTHONPATH`; namespace merge picks up both |

**Import style** — because there is no root `__init__.py` to auto-import
sub-modules, users must import sub-packages explicitly:

```python
# Correct — explicit sub-package import
import mrpt.poses
from mrpt.math import TPoint3D
from mrpt.slam import CICP

# Also works
import mrpt.poses as poses
p = poses.CPose3D(1, 0, 0, 0, 0, 0)
```

This is consistent with ROS 2 Python packages (`from rclpy.node import Node`,
`from sensor_msgs.msg import LaserScan`).

---

## Per-module file layout

For every module that provides Python bindings, three additions are made inside
the colcon package directory (`modules/mrpt_<name>/`):

```
modules/mrpt_<name>/
├── CMakeLists.txt                  ← calls mrpt_add_python_module()
├── python_bindings/
│   └── mrpt_<name>_py.cpp         ← pybind11 PYBIND11_MODULE(_bindings, m){…}
└── python/
    └── mrpt/
        └── <name>/
            └── __init__.py        ← re-exports from _bindings, adds helpers
```

Note: there is **no** `python/mrpt/__init__.py` in any module.  The `mrpt/`
directory itself is intentionally left without an `__init__.py` so Python
treats it as an implicit namespace package.

### `python_bindings/mrpt_<name>_py.cpp`

The C++ side.  It is a standard pybind11 extension module compiled with
`pybind11_add_module(_bindings …)`.  The module is always named `_bindings`
(underscore prefix signals it is a private implementation detail):

```cpp
PYBIND11_MODULE(_bindings, m)
{
    m.doc() = "Python bindings for mrpt::<name>";

    py::class_<mrpt::<name>::SomeClass, std::shared_ptr<mrpt::<name>::SomeClass>>(m, "SomeClass")
        .def(py::init<>())
        .def("doSomething", &mrpt::<name>::SomeClass::doSomething);

    m.def("someFunction", &mrpt::<name>::someFunction);
}
```

Key conventions used throughout the bindings:

| Convention | Rationale |
|---|---|
| `std::shared_ptr<T>` as holder for all polymorphic MRPT classes | MRPT uses `std::shared_ptr` internally; pybind11 must know the same holder type |
| Lambdas for output-reference parameters | Convert C++ `void f(T& out)` → Python tuple `(result,)` |
| `py::return_value_policy::reference_internal` for reference getters | Keeps the parent object alive while Python holds a reference to the sub-object |
| `__repr__` on every class | Makes interactive debugging pleasant |
| Context-manager (`__enter__` / `__exit__`) on resource classes | Follows Python idiom for streams, sockets, ports |
| NumPy (`pybind11/numpy.h`) helpers on array types | Zero-copy or minimal-copy exchange with NumPy arrays |

### `python/mrpt/<name>/__init__.py`

The Python side.  It imports from `_bindings` and re-exports under clean names.
It may also define pure-Python helpers or convenience functions:

```python
from . import _bindings as _b

SomeClass    = _b.SomeClass
someFunction = _b.someFunction

__all__ = ["SomeClass", "someFunction"]
```

---

## CMake integration

### The `mrpt_add_python_module()` macro

Defined in `modules/mrpt_common/cmake/mrpt_cmake_functions.cmake`:

```cmake
function(mrpt_add_python_module MODULE_NAME CPP_SOURCES)
  set(PYBIND11_FINDPYTHON ON)
  find_package(Python3 COMPONENTS Interpreter Development)
  find_package(pybind11)
  if (pybind11_FOUND)
    mrpt_ament_cmake_python_get_python_install_dir()
    pybind11_add_module(_bindings MODULE ${CPP_SOURCES})
    target_link_libraries(_bindings PRIVATE ${PROJECT_NAME})
    target_include_directories(_bindings PRIVATE include)
    install(TARGETS _bindings
            LIBRARY DESTINATION ${PYTHON_INSTALL_DIR}/mrpt/${MODULE_NAME}/)
    install(DIRECTORY python/mrpt
            DESTINATION ${PYTHON_INSTALL_DIR}
            FILES_MATCHING PATTERN "*.py")
  endif()
endfunction()
```

The macro installs:
1. `_bindings.so` → `<python_install_dir>/mrpt/<name>/_bindings.so`
2. `python/mrpt/<name>/__init__.py` → `<python_install_dir>/mrpt/<name>/__init__.py`

Nothing is ever installed to `<python_install_dir>/mrpt/__init__.py` — that
file must not exist for the namespace package mechanism to work correctly.

To add bindings to an existing module, append to its `CMakeLists.txt`:

```cmake
mrpt_add_python_module(<name>
  python_bindings/${PROJECT_NAME}_py.cpp
)
```

### Build

```bash
# Build a single module and its bindings
colcon build --packages-up-to mrpt_poses

# Build everything (no --merge-install needed)
colcon build

# Activate the install (adds all per-package prefixes to PYTHONPATH)
source install/setup.bash

# Verify
python3 -c "from mrpt.poses import CPose3D; print(CPose3D())"
```

`colcon_defaults.yaml` at the repository root enables `--symlink-install` for
faster incremental development; `--merge-install` is intentionally **not**
used so that the namespace package behaviour is exercised during development
in exactly the same way as in a downstream ROS build farm.

---

## Why no root `mrpt/__init__.py`?

An `mrpt/__init__.py` file — even an empty one — would cause Python to treat
`mrpt` as a *regular* package rather than a namespace package.  This has a
critical consequence:

> Python stops searching `sys.path` for additional `mrpt/` directories once
> it finds the first directory containing `__init__.py`.

In a split-install colcon workspace this means only the sub-packages installed
alongside the `__init__.py` (typically only those from `mrpt_core`) would be
importable; all others would silently disappear.

The previous design used `pkgutil.extend_path(__path__, __name__)` to work
around this, but that only works when `source install/setup.bash` is used.  It
breaks when only a subset of packages are sourced, and it creates a file-
ownership conflict in Debian packaging (two `.deb` files cannot both own
`/opt/ros/<distro>/lib/python3/dist-packages/mrpt/__init__.py`).

The implicit namespace package approach has none of these problems.

---

## Currently wrapped modules

| Python package | C++ module | Notable classes / functions |
|---|---|---|
| `mrpt.bayes` | `mrpt_bayes` | CParticleFilter, TParticleFilterOptions, enums |
| `mrpt.comms` | `mrpt_comms` | CClientTCPSocket, CSerialPort |
| `mrpt.config` | `mrpt_config` | CConfigFile, CConfigFileMemory |
| `mrpt.containers` | `mrpt_containers` | YAML (mrpt::containers::yaml) |
| `mrpt.core` | `mrpt_core` | Clock, WorkerThreadsPool, deg2rad, reverse_bytes |
| `mrpt.expr` | `mrpt_expr` | CRuntimeCompiledExpression |
| `mrpt.graphs` | `mrpt_graphs` | CNetworkOfPoses2D/3D |
| `mrpt.gui` | `mrpt_gui` | CDisplayWindow3D |
| `mrpt.img` | `mrpt_img` | CImage, TCamera, TStereoCamera |
| `mrpt.io` | `mrpt_io` | CStream, CFileInputStream/OutputStream, gz, CMemoryStream |
| `mrpt.kinematics` | `mrpt_kinematics` | CVehicleVelCmd, CVehicleSimul_DiffDriven/Holo |
| `mrpt.maps` | `mrpt_maps` | CSimplePointsMap, COccupancyGridMap2D |
| `mrpt.math` | `mrpt_math` | TPoint2/3D, TPose2/3D, TLine, TPlane, TBoundingBox, CPolygon, CHistogram, wrap2pi, matrices |
| `mrpt.obs` | `mrpt_obs` | CObservation2DRangeScan, IMU, Odometry, CSensoryFrame |
| `mrpt.poses` | `mrpt_poses` | CPose2D/3D, PDF types, SE_average2/3 |
| `mrpt.random` | `mrpt_random` | CRandomGenerator, drawUniformArray, drawGaussianArray |
| `mrpt.rtti` | `mrpt_rtti` | TRuntimeClassId, class registry, classFactory |
| `mrpt.serialization` | `mrpt_serialization` | objectToBytes, bytesToObject |
| `mrpt.slam` | `mrpt_slam` | CICP, CMetricMapBuilderICP |
| `mrpt.system` | `mrpt_system` | CTicTac, CTimeLogger, CRC, base64 |
| `mrpt.tfest` | `mrpt_tfest` | se2_l2, se3_l2_robust, TMatchingPair |
| `mrpt.topography` | `mrpt_topography` | TGeodeticCoords, WGS84↔ECEF↔ENU |
| `mrpt.viz` | `mrpt_viz` | Scene, all 3D renderables, stock_objects |

### Modules intentionally not wrapped

| Module | Reason |
|---|---|
| `mrpt_typemeta` | C++ compile-time metaprogramming; no runtime API surface |
| `mrpt_common` | Build-system helpers only |
| `mrpt_data` | Shared test data assets |
| `mrpt_imgui` | Dear ImGui C++ integration; no meaningful Python surface |
| `mrpt_libapps_cli/gui` | Internal application scaffolding |
| `mrpt_hwdrivers` | Hardware I/O; better served by dedicated Python packages or ROS drivers |

---

## Python examples

All examples live in `mrpt_examples_py/` (repository root).  Each script:

- imports only the MRPT Python API using explicit sub-package imports;
- demonstrates a single module or small combination;
- prints expected output and includes sanity assertions;
- doubles as API documentation.

See `mrpt_examples_py/README.md` for the full index.

```bash
# Run an example after building and sourcing the workspace
python3 mrpt_examples_py/mrpt_poses_example.py
python3 mrpt_examples_py/mrpt_slam_example.py
python3 mrpt_examples_py/mrpt_math_example.py
```

---

## Adding bindings to a new module

Step-by-step checklist:

1. **Create** `modules/mrpt_<name>/python_bindings/mrpt_<name>_py.cpp`
   with a `PYBIND11_MODULE(_bindings, m)` body.
2. **Create** `modules/mrpt_<name>/python/mrpt/<name>/__init__.py`
   re-exporting from `_bindings`.  Do **not** create `python/mrpt/__init__.py`.
3. **Append** to `modules/mrpt_<name>/CMakeLists.txt`:
   ```cmake
   mrpt_add_python_module(<name>
     python_bindings/${PROJECT_NAME}_py.cpp
   )
   ```
4. **Create** `mrpt_examples_py/mrpt_<name>_example.py` using explicit imports.
5. **Update** `mrpt_examples_py/README.md` with a new table row.
6. **Tick** the relevant checkbox in `doc/source/doxygen-docs/port_mrpt3.md`
   section 13.11.
