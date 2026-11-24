\page app_ReactiveNav3D-Demo ReactiveNav3D-Demo


Simulator to test mrpt::nav::CReactiveNavigationSystem3D, the PTG-based
reactive navigator for differential-driven robots whose shape is modeled in
3D as a stack of prisms ("height levels"), each with its own 2D polygonal
footprint. Obstacles are checked against each level separately. The
simulated world has one occupancy grid map per height level (built into the
app), sensed with simulated 2D laser scanners and depth cameras placed at
different heights. An optional short-term memory grid keeps obstacles seen
by the depth cameras once they leave their field of view.

# Usage

    ReactiveNav3D-Demo <config_file.ini>

# Example

    ReactiveNav3D-Demo modules/mrpt_data/config_files/navigation-ptgs/reactive3d_config.ini

The PTGs are precomputed at startup for each height level, which may take a
while the first time (they are cached to files afterwards).

# User interface

A 3D window shows the maps (one per level), the robot shape, the sensed
obstacles and the target. Keys:

| Key | Action |
|---|---|
| `m` | Move the target: the target follows the mouse; click to place it and start navigating. |
| `p` | Pause navigation. |
| `r` | Resume navigation. |
| `e` | Exit. |

# Configuration file

- `[CReactiveNavigationSystem3D]`: robot shape and PTGs:
  - `HEIGHT_LEVELS`, and for each level `i` (1-based): `LEVELi_HEIGHT`,
    `LEVELi_VECTORX`, `LEVELi_VECTORY` (polygon vertices, in meters).
  - `PTG_COUNT` and `PTGn_Type`, `PTGn_*` (1-based) for each PTG.
  - `min_obstacles_height`, `max_obstacles_height`.
- `[CAbstractNavigator]`, `[CWaypointsNavigator]`,
  `[CAbstractPTGBasedReactive]`, holonomic method and motion optimizer
  sections: the generic navigator parameters (see
  mrpt::nav::CAbstractPTGBasedReactive).
- Simulator:
  - `[LASER_CONFIG]`: `N_LASERS`, and for each laser `LASERi_POSE`,
    `LASERi_MAX_RANGE`, `LASERi_APERTURE`, `LASERi_STD_ERROR`,
    `LASERi_LEVEL` (height level it belongs to), `LASERi_SEGMENTS`.
  - `[KINECT_CONFIG]`: `N_KINECTS`, and for each depth camera its level,
    pose (`KINECTi_X`, `_Y`, `_Z`, `_PHI`, `_PITCH`), range, field of view,
    resolution (`_ROWS`, `_COLUMNS`) and noise.
  - `[MAP_CONFIG]`: `MAP_RESOLUTION` of the built-in maps, and the initial
    robot pose `X0`, `Y0`, `PHI0` (degrees).
  - `[STM_CONFIG]`: short-term memory obstacle grid (`Stm_active`,
    `Obs_grid_length`, `Obs_grid_resolution`, `Vision_limit`, likelihood
    increments and `Occupancy_threshold`).

# Output

Navigation logs (`*.reactivenavlog`, see
[navlog-viewer](page_app_navlog-viewer.html)) are written to
`./reactivenav.logs/`, and the PTG collision grids are cached as
`ReacNavGrid_NNN_LNN.dat.gz` files in the current directory.

# See also

- [ReactiveNavigationDemo](page_app_ReactiveNavigationDemo.html): 2D version.
- [navlog-viewer](page_app_navlog-viewer.html)
- mrpt::nav::CReactiveNavigationSystem3D
