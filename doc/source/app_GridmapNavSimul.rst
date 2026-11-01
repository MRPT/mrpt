.. _app_GridmapNavSimul:

====================================================
Application: GridmapNavSimul
====================================================

GridmapNavSimul
is a GUI program that takes an occupancy gridmap and let you move a robot
around simulating a laser scanner.
The robot can be teleoperated manually with the keyboard, with a joystick
or reproduce a sequence of fixed poses given in a file.

Launch it with:

.. code-block:: bash

   GridmapNavSimul

The following video tutorial summarizes its usage:

.. raw:: html

    <div style="position: relative; padding-bottom: 56.25%; height: 0; overflow: hidden; max-width: 100%; height: auto;">
        <iframe src="https://www.youtube.com/embed/ud9b3Z77Cbo" frameborder="0" allowfullscreen style="position: absolute; top: 0; left: 0; width: 100%; height: 100%;"></iframe>
    </div>

User interface
----------------

The simulated world is the background of the window; the camera follows the
robot (this can be disabled in the "View" menu). Drive the robot with the arrow
keys while the mouse is over the 3D view (Space brakes), or with a joystick.
Dockable panels:

- **Simulation**: START / STOP recording, the output rawlog and ground truth
  files, the rawlog format (actions/sensory-frames, or observations only),
  decimation, joystick, and **Re-simulate a rawlog**, which regenerates the
  laser scans of a recorded rawlog with the current sensor parameters.
- **Sensor**: number of ranges, span, and range and bearing noise of the
  simulated laser scanner.
- **Odometry error model**: bias and standard deviation of the odometry
  errors in x, y and heading, as fractions of each motion increment.

While recording, the rawlog, a ground truth text file (``time x y phi`` per
line, relative to the starting pose) and the gridmap
(``<rawlog>_grid.gridmap.gz``) are saved. The "File" menu loads another
gridmap: a binary ``.gridmap(.gz)`` file, or an image (its resolution and
center are asked for).
