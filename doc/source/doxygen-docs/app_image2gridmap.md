\page app_image2gridmap image2gridmap


Converts an image (e.g. a hand-drawn map, or a floor plan) into an MRPT
occupancy grid map file (`*.gridmap.gz`), the serialization of an
mrpt::maps::COccupancyGridMap2D. Such files can be used by localization,
navigation and simulation apps, and by \ref app_simul-gridmap.

# Usage

    image2gridmap -i <image> -r <resolution> [-o <output>] [--cx <x> --cy <y> | --px <col> --py <row>] [-w]

- `-i, --input`: input image file, in any format supported by
  mrpt::img::CImage (`png`, `jpg`, ...). Required.
- `-r, --res`: resolution, i.e. the size of one pixel (one grid cell) in
  meters. Required.
- `-o, --output`: output file. Default: the input file name with the
  extension changed to `.gridmap.gz`.
- `--cx`, `--cy`: map coordinates (in meters) of the image central pixel
  (default: 0, 0, i.e. the map origin is at the image center).
- `--px`, `--py`: alternatively, the pixel coordinates of the map origin.
  `px` counts columns from the left; `py` counts rows from the **bottom** of
  the image upwards.
- `-w, --overwrite`: overwrite the output file if it already exists (otherwise,
  the program aborts).
- `--version`, `-h, --help`.

`--cx`/`--cy` and `--px`/`--py` must be given in pairs, and the two pairs
cannot be combined.

# Example

Using a sample map image from `mrpt_data`, with 5 cm per pixel and the map
origin at the image center:

    image2gridmap -i datasets/graphslam-engine-demos/basic_map.png -r 0.05 -o basic_map.gridmap.gz

# Input / output formats

Each pixel intensity (the gray level, for color images), normalized to
[0,1], is the probability of the cell being **free**: white means free, black
means occupied, and mid gray (0.5) unknown. Values are clamped to
[0.01, 0.99].

The output is a compressed (zstd) serialized mrpt::maps::COccupancyGridMap2D,
despite the `.gz` default extension. MRPT programs detect the compression
format automatically when reading it.

# See also

- \ref app_ros-map-yaml2mrpt: imports ROS `map_server` maps (image plus YAML
  metadata).
- \ref app_simul-gridmap: simulates laser scans in a grid map.
