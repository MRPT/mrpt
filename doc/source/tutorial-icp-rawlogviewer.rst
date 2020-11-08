.. tutorial-icp-rawlogviewer:

===================================================================
Using the ScanMatching (ICP) module in RawLogViewer
===================================================================


The "ScanMatching" module within `RawLogViewer <app_RawLogViewer.html>`_
can be used to try the performance of the ICP algorithms as the parameters
are changed, which is interesting to parameter tuning or just for
didactic and teaching purposes.

Basically, this module takes two sensory frames `mrpt::obs::CSensoryFrame <class_mrpt_obs_CSensoryFrame.html>`_
or two single observations `mrpt::obs::CObservation <class_mrpt_obs_CObservation.html>`_,
builds a pointcloud from each of them, and then run `mrpt::slam::CICP <class_mrpt_slam_CICP.html>`__
on them.

Experiments with videos
~~~~~~~~~~~~~~~~~~~~~~~~~

Note that in the "animate" mode the execution of ICP is intentionally carried
out more slowly to ease the step-by-step visualization, though the normal
operation usually takes just a few milliseconds.

In this first experiment we can see how the threshold value for the maximum
distance for establishing a correspondence between the two maps strongly limits
the attainable adjustment that the ICP algorithm can perform from the initial
position: in a first trial, a threshold of 0.75m is not enough to lead the
algorithm to convergence, but it does quickly by changing the threshold to 2.5m.

.. raw:: html

    <div style="position: relative; padding-bottom: 56.25%; height: 0; overflow: hidden; max-width: 100%; height: auto;">
        <iframe src="https://www.youtube.com/embed/B_OAlxjDhEM" frameborder="0" allowfullscreen style="position: absolute; top: 0; left: 0; width: 100%; height: 100%;"></iframe>
    </div>

|

This second video demonstrates the two reference maps that our ICP
implementation can handle: a map of points, or a grid map.
In this later case, correspondences are established by matching laser
points to occupied cells in the grid.

.. raw:: html

    <div style="position: relative; padding-bottom: 56.25%; height: 0; overflow: hidden; max-width: 100%; height: auto;">
        <iframe src="https://www.youtube.com/embed/WbM8ri7Jk_w" frameborder="0" allowfullscreen style="position: absolute; top: 0; left: 0; width: 100%; height: 100%;"></iframe>
    </div>

    |


Finally, this video shows ICP for two Velodyne VLP16 scans:

.. raw:: html

    <div style="position: relative; padding-bottom: 56.25%; height: 0; overflow: hidden; max-width: 100%; height: auto;">
        <iframe src="https://www.youtube.com/embed/wh12Gvd3GY4" frameborder="0" allowfullscreen style="position: absolute; top: 0; left: 0; width: 100%; height: 100%;"></iframe>
    </div>
