^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package mrpt_hwdrivers
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

3.4.0 (2026-10-04)
------------------
* Python: commit generated .pyi type stubs for all modules
* Fix test failures on ARM and s390x Ubuntu PPA builds
* Contributors: Jose Luis Blanco-Claraco

3.3.1 (2026-09-29)
------------------
* hwdrivers, comms: harden the network and serial drivers; add fake-device tests.
* Increase test coverage and fix the bugs found.
* Contributors: Jose Luis Blanco-Claraco

3.3.0 (2026-09-26)
------------------
* fix formatting
* fix: qualify mrpt::format() calls to avoid ambiguity with std::format
* Python bindings: datasets, maps, localization, sensors (+ fixes) (`#1429 <https://github.com/MRPT/mrpt/issues/1429>`_)
* fix(system): locate mrpt_data in 3.x layouts; stop tests passing silently
* changelogs
* Contributors: Jose Luis Blanco-Claraco

3.2.0 (2026-09-16)
------------------
* fix: clean up ROS buildfarm warnings and correct the real issues they exposed across MRPT.
* feat: expand hardware-driver coverage with mocked I/O streams, reviving dead pcap support and improving test depth.
* fix: fix GPS, laser, and sensor-factory defects uncovered by the new mocked-stream tests.
* fix: ensure the scanner and driver implementations behave correctly when streams are external, missing, or failing.
* Contributors: Jose Luis Blanco-Claraco

3.1.4 (2026-09-04)
------------------

3.1.3 (2026-08-12)
------------------

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

3.0.0 (2026-06-06)
------------------

2.20.0 (2026-06-06)
-------------------
* Last release of the 2.x series. Starting from 3.0.0, changes are tracked
  in each module's own CHANGELOG.rst file.
* Contributors: Jose Luis Blanco-Claraco

