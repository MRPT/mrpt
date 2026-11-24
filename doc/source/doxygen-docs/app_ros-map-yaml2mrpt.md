\page app_ros-map-yaml2mrpt ros-map-yaml2mrpt


Imports an occupancy grid map in the ROS `map_server` format (a `.yaml`
metadata file plus an image) and saves it as an MRPT grid map file
(`*.gridmap.gz`, a serialized mrpt::maps::COccupancyGridMap2D). Optionally, it
also creates a `.3Dscene` file to quickly visualize the map in
[SceneViewer3D](page_app_SceneViewer3D.html).

# Usage

    ros-map-yaml2mrpt -i <map.yaml> [-d <output_dir>] [--generate-3d] [-w] [-q]

- `-i, --input`: input map `.yaml` file (required).
- `-d, --output-directory`: directory for the output files. Default: the
  directory of the input file. It must already exist.
- `--generate-3d`: also write a `.3Dscene` file with a 3D view of the grid map.
- `-w, --overwrite`: overwrite the output files if they already exist
  (otherwise, the program aborts).
- `-q, --quiet`: only print errors.
- `--version`, `-h, --help`.

The output files are named after the input file, with the extension changed
to `.gridmap.gz` and `.3Dscene`.

# Example

    ros-map-yaml2mrpt -i my_map.yaml --generate-3d

writes `my_map.gridmap.gz` and `my_map.3Dscene` next to `my_map.yaml`.

# Input format

The `.yaml` file follows the
[map_server YAML format](http://wiki.ros.org/map_server#YAML_format):

    image: my_map.pgm
    resolution: 0.05
    origin: [-10.0, -10.0, 0.0]
    negate: 0
    occupied_thresh: 0.65
    free_thresh: 0.196
    mode: trinary

- `image`: image file, relative to the directory of the `.yaml` file.
- `resolution`: cell size, in meters.
- `origin`: map coordinates `[x, y, yaw]` of the lower-left pixel. Only `x`
  and `y` are used.
- `negate`, `occupied_thresh`, `free_thresh`: as in `map_server`.
- `mode`: optional, `trinary` (default) or `scale`. Other modes are not
  supported.

See mrpt::maps::COccupancyGridMap2D::loadFromROSMapServerYAML() for the
details of the conversion.

# See also

- \ref app_image2gridmap: converts a plain image into a grid map.
- mrpt::maps::COccupancyGridMap2D::FromROSMapServerYAML().
