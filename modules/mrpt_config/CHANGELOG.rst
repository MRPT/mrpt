^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package mrpt_config
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

3.1.0 (2026-07-03)
------------------
* Merge pull request `#1377 <https://github.com/MRPT/mrpt/issues/1377>`_ from MRPT/feat/options-capable-in-config
  feat(mrpt_config): add OptionsCapable mixin, implement in map classes
* fix(mrpt_config): mark trySetCreationOptions() [[nodiscard]]
  Ignoring the return value would silently leave creation options unapplied.
  Address coderabbitai review comment on PR `#1377 <https://github.com/MRPT/mrpt/issues/1377>`_.
* feat(mrpt_config): add OptionsCapable mixin, implement in map classes
  Backports mola::OptionsCapable (MOLAorg/mola) into mrpt_config, next to
  CLoadableOptions: a virtual interface exposing a class' CLoadableOptions
  members generically by name, plus a safe creation-options setter. This lets
  generic tooling (de)serialize a map's insertion/likelihood/render options
  without knowing the concrete class.
  Implement it in every mrpt_maps class that holds CLoadableOptions-derived
  members: COccupancyGridMap2D, COccupancyGridMap3D, CPointsMap,
  CHeightGridMap2D, CHeightGridMap2D_MRF, CWirelessPowerGridMap2D,
  CGasConcentrationGridMap2D, CRandomFieldGridMap3D, CReflectivityGridMap2D,
  CBeaconMap, COctoMapBase and CVoxelMapOccupancyBase.
* Contributors: Jose Luis Blanco-Claraco

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

