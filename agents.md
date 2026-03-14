# MRPT 3.0 AI Agent Instructions

Welcome, AI Agent! This file contains the architectural context and coding guidelines for the Mobile Robot Programming Toolkit (MRPT) 3.0. When generating, refactoring, or reviewing code in this repository, you must strictly adhere to the following rules.

## 1. Project Architecture
* **MRPT 3.0 is highly modular.** It is designed to be built using **colcon** (similar to ROS 2 packages).
* Each module (e.g., `mrpt_opengl`, `mrpt_math`) lives in its own directory and functions as an independent CMake project.
* **Target OS/Compilers:** Cross-platform (Linux, Windows, macOS, WebAssembly/Emscripten).

## 2. Build System (CMake) Conventions
MRPT 3.0 abstracts away standard CMake boilerplate using `mrpt_common`. Use standard CMake commands but prefer mrpt_common cmake helpers when possible for consistency.

* **Compiling a module**: Use ``colcon build --packages-up-to mrpt_XXX``, then ``. install/setup.bash`` then you can run the executables or unit tests.

* **Minimum CMake Version:** `cmake_minimum_required(VERSION 3.16)`
* **Always include the MRPT common scripts:**

```cmake
find_package(mrpt_common REQUIRED)
```

* Target Definition: Use mrpt_add_library instead of add_library. This macro automatically handles C++ standard configurations, export targets, and installation steps.

```cmake
mrpt_add_library(
  TARGET ${PROJECT_NAME}
  SOURCES ${LIB_SOURCES} ${LIB_PUBLIC_HEADERS}
  PUBLIC_LINK_LIBRARIES mrpt::another_module
  CMAKE_DEPENDENCIES another_module
)
```

Dependency Management: 

* Use find_package(mrpt_<module_name> REQUIRED) to find other MRPT modules.

Link against namespaced targets (e.g., mrpt::mrpt_poses, Eigen3::Eigen).

## 3. C++ Coding Guidelines
Standard: Use Modern C++ features where appropriate (C++17/C++20).

Namespaces: All core code must reside within the mrpt:: namespace or its sub-namespaces (e.g., mrpt::opengl::).

Unit tests: test files are in a "tests" subdirectory in each module, and their name must end in `_unittest.cpp`, then they will be catched automatically for inclusion as unit tests.

License Headers: Every new .cpp, .h, and CMakeLists.txt file must start with the standard MRPT SPDX-License-Identifier header:

```cpp
/*                    _
                     | |    Mobile Robot Programming Toolkit (MRPT)
 _ __ ___  _ __ _ __ | |_
| '_ ` _ \| '__| '_ \| __|          https://www.mrpt.org/
| | | | | | |  | |_) | |_
|_| |_| |_|_|  | .__/ \__|     https://github.com/MRPT/mrpt/
               | |
               |_|

 Copyright (c) 2005-2026, Individual contributors, see AUTHORS file
 See: https://www.mrpt.org/Authors - All rights reserved.
 SPDX-License-Identifier: BSD-3-Clause
*/
```

Formatting: according to `.clang-format` and `.clang-tidy`. In particular: prefer `if (x) {\n y;\n }` instead of `if(x) y;`. Use `[[nodiscard]]` where applicable.

## 4. Anti-Patterns to Avoid
Do not manually configure .so versioning or write manual install() blocks for standard headers/libraries. mrpt_add_library does this.

Do not hardcode compiler flags (like -std=c++17 or -fPIC). The MRPT CMake wrappers handle these natively.

Do not use raw pointers for ownership. Default to std::shared_ptr or std::unique_ptr, and use MRPT's smart pointer macros where applicable.


## 5. Pybind11 modules

In mrpt 3.x, most MRPT libraries under `ROOT/modules/mrpt_*` now have their own pybind11 module, in a modular way so users can import
only the required modules.

* `modules/mrpt_core/python/mrpt/__init__.py` is the main root python file for all modules, in charge of trying to import the rest, if they exist.
* Each module, for example `mrpt_img`, has its own `modules/mrpt_img/python/mrpt/img/__init__.py` that must be updated with new wrapped C++ classes.
* Each module, for example `mrpt_img`, has its own `modules/mrpt_img/python_bindings/mrpt_img_py.cpp` file with the specific wrapped C++ classes and python adaptors.
* Python examples, demonstrating each module wrapped features, live under `mrpt_examples_py`. They should be updated/extended or new examples created when appropriate as new classes are wrapped.

### 5.1 File structure for each pybind11 module

To add or extend Python bindings for a module `mrpt_foo`, three files are needed:

1. **`modules/mrpt_foo/python_bindings/mrpt_foo_py.cpp`** — The C++ pybind11 source. Must define `PYBIND11_MODULE(_bindings, m) { ... }`. The compiled shared library is always named `_bindings.so`.
2. **`modules/mrpt_foo/python/mrpt/foo/__init__.py`** — Python re-exports: `from . import _bindings as _b`, then `ClassName = _b.ClassName` for each wrapped class/function, and an `__all__` list.
3. **`modules/mrpt_foo/CMakeLists.txt`** — Must call `mrpt_add_python_module(foo python_bindings/${PROJECT_NAME}_py.cpp)`. If this line is commented out, uncomment it.

The root `mrpt/__init__.py` (in `mrpt_core`) auto-imports all known submodules by name. If adding a new module name, add it to the `MRPT_MODULES` list there.

### 5.2 Pybind11 coding patterns and conventions

**Includes:** Always include `<pybind11/pybind11.h>` and `<pybind11/stl.h>`. Add `<pybind11/eigen.h>` for Eigen/matrix types, `<pybind11/numpy.h>` for `py::array_t<T>`, `<pybind11/operators.h>` for operator overloading, `<pybind11/chrono.h>` for time types, `<pybind11/functional.h>` for `std::function` callbacks.

**Class binding with inheritance:**
```cpp
py::class_<Derived, Base1, Base2, std::shared_ptr<Derived>>(m, "Derived")
```
Always use `std::shared_ptr<T>` holder for classes that are commonly used via smart pointers. Include `CSerializable` in the base list for serializable classes.

**Properties:** When C++ uses `x()` getter and `x(double)` setter (common in MRPT), use:
```cpp
.def_property("x",
    [](const T& p) { return p.x(); },
    [](T& p, double val) { p.x(val); })
```
For public member variables, use `.def_readwrite("name", &T::name)` or `.def_readonly(...)`.

**Overloaded methods:** Resolve with `py::overload_cast<ArgTypes...>(&Class::method)` or `static_cast<RetType(Class::*)(ArgTypes...)>(&Class::method)`.

**Output-argument functions → return values:** Wrap in lambda:
```cpp
.def("compute", [](const T& self) {
    ResultType result;
    bool ok = self.compute(result);
    return py::make_tuple(ok, result);
})
```

**Operator overloading:** Use `py::self + py::self`, etc. Always add `__str__` and `__repr__`.

**NumPy integration:**
- For Eigen matrices: `#include <pybind11/eigen.h>` enables automatic conversion. For zero-copy, return `Eigen::Map<const RowMajorMatrix>(ptr, rows, cols)`.
- For `CImage`: Use `py::array_t<uint8_t>` with shape/strides and pass `self_obj` (the Python object) as the buffer base for zero-copy.
- Add `__array__` protocol: `.def("__array__", [](const T& self) { return self.as_numpy(); })`.

**Iterators:** `py::make_iterator(container.begin(), container.end())` with `py::keep_alive<0, 1>()`.

**Enums:** `py::enum_<T>(m, "Name").value("A", T::A).export_values();` Use `py::arithmetic()` for bitmask enums.

**GIL management:** When passing Python callables to C++ threads, acquire the GIL: `py::gil_scoped_acquire gil;` inside the C++ callback lambda.

**Return value policies:** Use `py::return_value_policy::reference_internal` for getters returning references to internal objects (e.g., `getCamera()` on a Viewport). Use `py::return_value_policy::reference` for singletons/static data.

**Python `__init__.py` conventions:**
- Re-export all bound classes: `ClassName = _b.ClassName`
- Monkey-patch `__array__` for NumPy integration where appropriate
- Add Pythonic conveniences (e.g., `<<` operator for scene building, color constants)
- Include `__all__` listing all public names

### 5.3 Current binding status

**Active (12 modules):** `core`, `math`, `poses`, `img`, `viz`, `serialization`, `system`, `containers`, `tfest`, `rtti`, `config`, `expr`.

**Not yet implemented:** `obs`, `maps`, `gui`, `opengl`, `slam`, `nav`, `io`, `graphs`, `graphslam`, `kinematics`, `topography`, `bayes`, `random`, `hwdrivers`, `comms`, `vision`, `imgui`, `typemeta`.

See `python/pybind11_plan_v3.md` for a detailed plan on extending wrappers to all modules.


## 6. Instructions for AI agents

- Do not use the compound command "cd foo && git ...", instead, use "git -C foo ...".

