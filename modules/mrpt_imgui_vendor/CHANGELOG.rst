^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package mrpt_imgui_vendor
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

3.5.1 (2026-10-09)
------------------

3.5.0 (2026-10-07)
------------------
* CMake config: add EXTRA_PRE_DEPS_CONFIG_CMDS hook; mrpt_imgui_vendor uses it to prefer GLVND (fixes CMP0072 warning in consumers)
* Contributors: Jose Luis Blanco-Claraco

3.4.0 (2026-10-04)
------------------
* Contributors: Jose Luis Blanco-Claraco

3.3.1 (2026-09-29)
------------------

3.3.0 (2026-09-26)
------------------
* changelogs
* Merge pull request `#1418 <https://github.com/MRPT/mrpt/issues/1418>`_ from MRPT/feat/imgui-vendor-package
* mrpt_imgui_vendor: require GLFW, and provide it on Windows via vcpkg
* mrpt_imgui_vendor: new package with a single vendored Dear ImGui copy
* Contributors: Jose Luis Blanco-Claraco


* New package: single vendored copy of Dear ImGui (docking branch), ImPlot,
