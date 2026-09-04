^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package mrpt_containers
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

3.1.4 (2026-09-04)
------------------
* fix(mrpt_containers): yaml double round-trip precision loss, %.16g -> %.17g (`#1396 <https://github.com/MRPT/mrpt/issues/1396>`_).
* fix(mrpt_containers): yaml_ref/yaml_cref: add missing asSequenceRange() and yaml_cref::getOrDefault() (`#1397 <https://github.com/MRPT/mrpt/issues/1397>`_, `#1398 <https://github.com/MRPT/mrpt/issues/1398>`_).
* fix(mrpt_containers): yaml: fix a TOP comment corrupting the document on serialize+reparse (`#1400 <https://github.com/MRPT/mrpt/issues/1400>`_).
* fix(mrpt_containers): yaml: keep an unquoted leading-zero digit run (e.g. "00") as a string instead of parsing it as a number (`#1401 <https://github.com/MRPT/mrpt/issues/1401>`_).
* Contributors: Jose Luis Blanco-Claraco

3.1.3 (2026-08-12)
------------------
* test(mrpt_maps): raise unit test coverage for occupancy grids and maps. Fix out-of-bounds reads in CDynamicGrid3D and invert check in computeObservationLikelihood_ConsensusOWA().
* fix(mrpt_containers): guard ts_hash_map self-assignment; extend tests.
* perf(mrpt_system): minimize CTimeLogger enter()/leave() overhead.
* fix(mrpt_system): make CTimeLogger reporting thread-safe vs. concurrent logging.
* test(mrpt_containers): raise unit-test coverage for dynamic grids, circular buffers, ts_hash_map, and YAML.
* Contributors: Jose Luis Blanco-Claraco

3.1.2 (2026-07-07)
------------------

3.1.1 (2026-07-04)
------------------

3.1.0 (2026-07-03)
------------------

3.0.4 (2026-06-17)
------------------

3.0.3 (2026-06-15)
------------------

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

