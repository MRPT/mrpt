.. _app_2d-slam-demo:

====================================================
Application: 2d-slam-demo
====================================================


1. Description
-----------------
This GUI application is a demonstration of 2D range-bearing SLAM
under the paradigm of EKF-based SLAM, in a didactic way that 
allows extensive experimentation with data-association and parameter tuning.

It is an extension to a similar Matlab program developed
by `J. Neira <https://webdiis.unizar.es/~neira/>`_ and 
`J. D. Tardós <https://webdiis.unizar.es/~jdtardos/>`_ (University of Zaragoza). 

At the core of this program lies one MRPT class: 
`mrpt::slam::CRangeBearingKFSLAM2D <class_mrpt_slam_CRangeBearingKFSLAM2D.html>`_.

The paper explaining the Matching Likelihood criteron for data association (one of the two choices offered in this program)
is :cite:`blanco2012amd` 
(`BibTex <https://ingmec.ual.es/aigaion2//index.php/export/publication/216/bibtex>`_, 
`Draft PDF <https://ingmec.ual.es/~jlblanco/papers/blanco2012amd.pdf>`_ ).


2. Demo video
----------------

.. raw:: html

    <div style="position: relative; padding-bottom: 56.25%; height: 0; overflow: hidden; max-width: 100%; height: auto;">
          <iframe src="https://www.youtube.com/embed/IHGcW_DCaps" frameborder="0" allowfullscreen style="position: absolute; top: 0; left: 0; width: 100%; height: 100%;"></iframe>
    </div>

3. Command line options
-----------------------------

Invoking the program without any flag will launch it in normal, interactive, mode::


   USAGE: 
   
   2d-slam-demo  [-r] [-n] [-c <params.ini>] [--] [--version] [-h]
   
   
   Where: 
   
   -r,  --norun
       Just load the config file, don't run it.
   
   -n,  --nogui
       Don't stay in the GUI, exit after the experiment.
   
   -c <params.ini>,  --config <params.ini>
       Config file to load
   
   --,  --ignore_rest
       Ignores the rest of the labeled arguments following this flag.
   
   --version
       Displays version information and exits.
   
   -h,  --help
       Displays usage information and exits.
   
   
   2d-slam-demo


With ``--config``, a non-interactive experiment is run first (its statistics
are printed and appended to ``slam_out.txt``); with ``--nogui`` the program
then exits, without ever opening a window, so batch experiments can run on
machines without a display.


4. User interface
-------------------

All views are dockable panels; their layout is remembered across sessions.

- **Ground truth**: the landmarks, the robot and the area seen by its sensor.
- **Observation**: the last observation in the robot frame, with the 3-sigma
  ellipses of each reading.
- **Estimated map**: the robot and landmarks estimated by the EKF, with their
  3-sigma ellipses (optionally labeled with their real landmark index).
- **Individual compatibility**: observations and predictions in
  range-bearing space, with the individually compatible pairs and the final
  associations; and the **DA matrix** of their distances.
- **Errors**: robot pose errors in x, y and heading vs. step, with their
  3-sigma bounds.
- **Statistics**: data association results (true/false positives and
  negatives, JCBB iterations) and computation time per step.
- **Parameters**: every parameter of the KF, the robot motion, the sensor,
  the data association and the ground truth map (square corridor, random, or a
  text file with one "x y" landmark per row), editable while the simulation
  runs.

Toolbar and keyboard: **Reset** (R), **Step** (S), **Run/Stop** (Space) and
**Batch** (B, a full non-interactive run in the background, which can be
cancelled). The **Tools** menu loads and saves the parameters, saves the
filter state or the last data association, records the simulated data to a
rawlog file, and shows the profiler statistics.
