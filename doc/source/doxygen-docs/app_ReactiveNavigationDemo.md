\page app_ReactiveNavigationDemo ReactiveNavigationDemo


Interactive simulator to test the MRPT 2D reactive navigation system
(mrpt::nav::CReactiveNavigationSystem, based on PTGs) on a simulated robot,
either differential-driven (mrpt::kinematics::CVehicleSimul_DiffDriven) or
holonomic (mrpt::kinematics::CVehicleSimul_Holo), moving in an occupancy grid
map and sensing it with a simulated 2D laser scanner. Use it to try navigator
parameters, PTGs and holonomic methods before running them on a real robot.
It uses wxWidgets.

# Usage

    ReactiveNavigationDemo

There are no command line arguments. At startup, the navigator parameters
are loaded from `config_files/navigation-ptgs/reactivenav-app-config.ini` in
the MRPT shared directory, and a default map is shown.

# User interface

The main view shows the map, the robot and its laser scan, the target and
the predicted path of the selected PTG; a second view shows the robot local
view. Typical workflow: select the robot kinematics type and navigator, press
**RUN**, then **Set target...** and click on the map.

- **Robot kinematics type**: differential (Ackermann) drive or holonomic.
  Parameters are taken from the sections with prefix `DIFF_` or `HOLO_`,
  respectively, of the reactive navigator parameters.
- **Navigator**: "Autonavigation (reactive)", or "Preprogrammed sequences"
  of velocity commands (mrpt::nav::CNavigatorManualSequence), for debugging.
- **RUN** / **STOP**: initialize (reloading all parameters) and start, or
  stop, the simulation.
- **Set target...**: click on the map to navigate to that point.
- **Waypoint list...**: left-click to add waypoints (Shift+click: the
  waypoint can't be skipped; Ctrl+click: with the heading given in "Waypt yaw
  (deg)"), right-click to finish and start navigating them
  (mrpt::nav::CWaypointsNavigator).
- **Replace robot...**: click to move the robot to a new position.
- **Draw occupied...** / **Draw empty...**: edit the map by clicking on it.
- **Show for PTG**: select the PTG whose predicted path is drawn.
- **Record .navlog files**: save the navigation logs, to be analyzed with
  [navlog-viewer](page_app_navlog-viewer.html).
- **Log trajectory**: save the robot trajectory to `traj_log.txt`.
- Tabs **Params: Reactive navigator**, **Params: Preprogrammed seq.** and
  **Params: Simulation**: editable INI texts with the parameters of the
  navigator, the manual sequences, and the simulator (`[SIMULATOR]`: sensor
  range, field of view, number of rays, noise and rate). **Generate template
  config file...** fills the navigator parameters with all the options and
  their default values.
- **File** menu: new map (empty, with a given size and resolution), load a
  map (an image, or a `.gridmap` / `.gridmap.gz` file) and save it.
- **View** menu: show or hide the sensor range and the robot path, and clear
  the path.

# Output

Navigation logs (`*.reactivenavlog`) are written to `./reactivenav.logs/`
when enabled. The PTGs may also cache their precomputed collision grids as
`ReacNavGrid_*.dat.gz` files in the current directory.

# See also

- [ptg-configurator](page_app_ptg-configurator.html), [navlog-viewer](page_app_navlog-viewer.html)
- [ReactiveNav3D-Demo](page_app_ReactiveNav3D-Demo.html): 3D robot shape version.
- [holonomic-navigator-demo](page_app_holonomic-navigator-demo.html)
- mrpt::nav::CReactiveNavigationSystem, mrpt::nav::CAbstractPTGBasedReactive
