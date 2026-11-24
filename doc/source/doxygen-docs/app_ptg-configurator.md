\page app_ptg-configurator ptg-configurator

# ptg-configurator

GUI tool to design, tune and visualize Parameterized Trajectory Generators
(PTGs, mrpt::nav::CParameterizedTrajectoryGenerator), the families of
kinematically feasible paths used by the MRPT reactive navigators
(mrpt::nav::CReactiveNavigationSystem and others). For a selected PTG class
and set of parameters it shows the paths in the workspace, the TP-Space
(Trajectory Parameter Space) with the obstacles transformed into it
(TP-Obstacles), clearance diagrams, and the velocity profiles of each path.
Use it to choose the PTG parameters for a robot before writing a navigator
config file. It uses wxWidgets.

## Usage

    ptg-configurator [-i <file.ini>] [-s <section>] [-l <library>]

| Option | Description |
|---|---|
| `-i`, `--ini FILE` | Load this INI file into the parameters text box. |
| `-s`, `--ini-section NAME` | INI section with the PTG parameters (default: `PTG_PARAMS`). |
| `-l`, `--load LIB` | Load a plugin library with additional PTG classes (Linux only). |

## Example

    ptg-configurator -i modules/mrpt_data/config_files/navigation-ptgs/reactivenav-app-config.ini -s DIFF_CReactiveNavigationSystem

Then select `CPTG_DiffDrive_C` in "Pick a PTG", set "PTG index for cfg file"
to 0 and press "Initialize PTG".

## User interface

- **Pick a PTG**: the list of all registered PTG classes. Selecting one fills
  the parameters text box with its default parameters (unless an INI file
  was loaded with `-i`).
- **PTG index for cfg file**: parameters are read from and written to keys
  prefixed with `PTG<index>_` (e.g. `PTG0_num_paths`), in the section given
  with `-s`, as in the navigator config files.
- **Initialize PTG**: (re)builds the PTG from the parameters in the text box.
- **Robot path** view: the family of paths in the workspace, optionally with
  the robot shape along the paths ("Draw robot shape", "Dist. btw. robot
  shapes"). Place an obstacle point or a target by clicking ("Obstacle point",
  "Target"), and highlight a single trajectory with the slider.
- **TP-Space** view: TP-Obstacles and clearance diagram (optionally
  interpolated) of the current obstacles, and the target in TP-Space.
- Plots of the selected path: x, y, heading, traversed distance and
  velocity components versus time, and the velocity commands at t=0.
- **Load Plugin** button: load a library with additional PTG classes.
- **File** menu: export the selected path (and the PTG parameters) to a
  MATLAB/Octave `.m` file.

## See also

- [ReactiveNavigationDemo](page_app_ReactiveNavigationDemo.html): test PTGs in a
  simulated reactive navigation.
- [navlog-viewer](page_app_navlog-viewer.html)
- mrpt::nav::CParameterizedTrajectoryGenerator, mrpt::nav::CPTG_DiffDrive_C,
  mrpt::nav::CPTG_Holo_Blend
