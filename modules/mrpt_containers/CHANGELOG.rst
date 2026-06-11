^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package mrpt_containers
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

3.0.2 (2026-06-11)
------------------

3.0.1 (2026-06-11)
------------------
* Merge pull request `#1363 <https://github.com/MRPT/mrpt/issues/1363>`_ from MRPT/fix/dont-export-eigen3-dep
  refactor: limit visibility of eigen3 as build dep
* fix: restore lean libfyaml-core.h include; disable libfyaml tests
  Revert yaml.cpp back to including <libfyaml/libfyaml-core.h> (as in develop)
  instead of the monolithic <libfyaml.h>, which pulls in <stdatomic.h> and
  breaks the C++ build on macOS (<atomic> incompatible with <stdatomic.h>) and
  gcc. The submodule is back on the fork commit that ships libfyaml-core.h.
  Also pass -DBUILD_TESTING=OFF to the embedded libfyaml so it does not
  FetchContent the 'check' test framework (needs network; breaks isolated and
  Debian-package builds).
* submodule: revert libfyaml to fork commit with Windows/macOS fixes
  The previous bump to upstream 9a4d9b2 lost the fork's portability fixes
  (MSVC ssize_t / C++17 atomic fallback) and used cmake_minimum_required(3.0),
  breaking the Windows and macOS CI. Revert to 1ed7581, which builds on all
  platforms (matches develop).
* fix: allow libfyaml to configure with recent CMake (>=4.0)
  The bundled libfyaml uses cmake_minimum_required(VERSION 3.0), which is
  rejected by CMake >=4.0 (macOS/Windows CI runners). Pass
  CMAKE_POLICY_VERSION_MINIMUM=3.5 to its ExternalProject configure step.
* submodule: update libfyaml
* fix: KF math errors
* Contributors: Jose Luis Blanco-Claraco

3.0.0 (2026-06-06)
------------------

2.20.0 (2026-06-06)
-------------------
* Last release of the 2.x series. Starting from 3.0.0, changes are tracked
  in each module's own CHANGELOG.rst file.
* Contributors: Jose Luis Blanco-Claraco

