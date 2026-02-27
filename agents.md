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
