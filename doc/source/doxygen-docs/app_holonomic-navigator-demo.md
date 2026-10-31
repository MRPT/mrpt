\page app_holonomic-navigator-demo Application: holonomic-navigator-demo

# holonomic-navigator-demo

An interactive simulator of the holonomic reactive navigation methods of
mrpt-nav (mrpt::nav::CHolonomicVFF and mrpt::nav::CHolonomicND) for a
circular robot with a 360-degree range sensor, in a 2D occupancy grid map.

    holonomic-navigator-demo [--start]

`--start` starts navigating at once, with the default settings.

## User interface

The world (map, robot, its sensor range and path, and the target) is the
background of the main window. Dockable panels:

- **Local view**: the normalized sensor readings around the robot, the
  relative target, the direction and speed chosen by the method, and the
  gaps detected by ND.
- **Configuration**: the holonomic method, and the parameters of the
  simulator and of both methods, as INI text (editable while stopped).
- **Stats**: robot and target positions, distance to the target, chosen
  speed and direction, ND situation and gaps, and the cursor position.

Toolbar: **Place robot** (R) and **Set target** (T), then left-click on the
map (Esc cancels); **START / STOP** (Space). The "File" menu loads another map,
either a binary `.gridmap(.gz)` file or an image (its resolution and center are
asked for). The "View" menu shows or hides the sensor range and the robot path.
