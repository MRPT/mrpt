\page app_observations2map observations2map


Builds one or more metric maps (occupancy grids, point clouds, voxel maps,
etc.) from a simple map (mrpt::maps::CSimpleMap, a `*.simplemap` file: a
sequence of keyframes, each one a robot pose plus its observations), and saves
them to files. The set of maps and their parameters are defined in an INI
config file, as an mrpt::maps::CMultiMetricMap. Typical use: re-generate a map
with different parameters (e.g. resolution, insertion options) from the
simple map saved by a SLAM app such as \ref app_icp-slam or \ref app_rbpf-slam.

# Usage

    observations2map <config_file.ini> <observations.simplemap> <outputmap_prefix> [-s <section>]

- `config_file.ini`: INI file with the map definitions.
- `observations.simplemap`: input simple map (uncompressed, gzip or zstd
  compressed).
- `outputmap_prefix`: prefix (it may include a path) of all output files.
- `-s <section>`: name of the INI section with the map definitions (default:
  `MappingApplication`). If used, it must be the last argument.

# Example

Using sample files from `mrpt_data`:

    observations2map \
      config_files/icp-slam/icp-slam_demo_classic_gridmatch.ini \
      datasets/malaga-cs-fac-building.simplemap.gz \
      out_map

This builds one occupancy grid map and writes:

- `out_map_00_mrpt__maps__COccupancyGridMap2D.bin`: the serialized map.
- `out_map_mrpt__maps__COccupancyGridMap2D_00.png`: the map as an image.
- `out_map_mrpt__maps__COccupancyGridMap2D_00_limits.txt`: the map limits.

# Configuration file

The section (`[MappingApplication]` by default) defines how many maps of each
type to create, and further sections `[<section>_<mapType>_<NN>_<options>]`
their parameters, as read by
mrpt::maps::TSetOfMetricMapInitializers::loadFromConfigFile(). For example:

    [MappingApplication]
    occupancyGrid_count=1
    pointsMap_count=0

    [MappingApplication_occupancyGrid_00_creationOpts]
    resolution=0.05

    [MappingApplication_occupancyGrid_00_insertOpts]
    maxDistanceInsertion=30

Other keys in the section (e.g. those used by SLAM apps) are ignored, so the
same config file of the SLAM run can be used. See the
`[MappingApplication]` sections of the files in `config_files/icp-slam/` and
`config_files/rbpf-slam/` in `mrpt_data` for complete examples.

# Output files

All maps are written with the prefix `outputmap_prefix`:

- One `<prefix>_<NN>_<class_name>.bin` file per map: the compressed (zstd)
  serialization of the map object, loadable from C++ with
  mrpt::serialization::CArchive.
- The files written by each map's `saveMetricMapRepresentationToFile()`
  method, with the prefix `<prefix>_<class_name>_<NN>` (e.g. images and limits
  for occupancy grids, text files for point maps, 3D scenes for some maps).

In the file names, `NN` is the map index and `class_name` the C++ class name
with `::` replaced by `__`.

# See also

- \ref app_carmen2simplemap: creates simple maps from CARMEN logs.
- \ref app_icp-slam, \ref app_rbpf-slam: SLAM apps that save simple maps.
- mrpt::maps::CMultiMetricMap, mrpt::maps::CSimpleMap.
