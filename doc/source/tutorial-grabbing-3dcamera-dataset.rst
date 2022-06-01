.. tutorial-grabbing-3dcamera-dataset

===================================================================
Grabbing your own RGBD camera datasets
===================================================================

.. contents:: :local:

Preparation
----------------

You will need MRPT compiled with support for Kinect, OpenNI2, or the driver of your specific camera.
Either download a Windows precompiled version, install the mrpt-apps Ubuntu package or compile MRPT from sources.

Make sure the camera is properly working by running one of the tests, for example:

  - Application `kinect-3d-view <page_app_kinect-3d-slam.html>`_
  - Example: `hwdrivers_openni2_driver_demo <page_hwdrivers_openni2_driver_demo.html>`_

Under GNU/Linux you may need root privileges (executing as `sudo`) to access the camera.
To avoid that, install `51-kinect.rules` into `/etc/udev/rules.d/`.

Calibrate your camera, using:

  - `camera-calib <app_camera-calib.html>`_ for monocular cameras,
  - `kinect-stereo-calib <page_app_kinect-stereo-calib.html>`_ for kinect-like RGBD cameras.

Make sure you have enough free space in some disk partition with bandwidth enough.
Prefer native filesystems, that is, e.g. do NOT grab into an NTFS partition from a GNU/Linux OS.

Make sure you have installed or compiled the programs:

  - `rawlog-grabber <page_app_rawlog-grabber.html>`_,
  - `rawlog-edit <page_app_rawlog-edit.html>`_, and
  - `RawLogViewer <app_RawLogViewer.html>`_.

Grabbing
-------------

Prepare a configuration file for `rawlog-grabber <page_app_rawlog-grabber.html>`_. Start from this template:
`[MRPT]/share/mrpt/config_files/rawlog-grabber/kinect.ini <https://github.com/MRPT/mrpt/blob/develop/share/mrpt/config_files/rawlog-grabber/kinect.ini>`_.

Notice that with the default configuration, all data and images will be embedded into one big binary file.
This is done to reduce the OS overhead of creating dozens of small files per second,
allocate them in the partition tables, etc.

Typically you should disable the option grab_3D_points to reduce the computational load.
To regenerate 3D point clouds, use `rawlog-edit <page_app_rawlog-edit.html>`_ or the corresponding
`C++ APIs <generating_3d_point_clouds_from_rgb_d_observations.html>`_.

Modify the parameter grab_decimation according to the processing power/bandwidth available in your recording computer. Under ideal conditions, grab_decimation=1 will record all frames.

By the way: You could also simultaneously grab any other set of sensors by adding new sections to the configuration file. See the examples.

Execute rawlog-grabber to actually record your dataset, invoking it with your config file as the only argument

.. code-block:: bash

   rawlog-grabber <YOUR_CONFIG_FILE>


Press ESC to end the recording. A single big `.rawlog <rawlog_format.html>`_
file should have been generated with your dataset.

Post-processing
---------------------

It is normally preferred to strip the rawlog so RGB & depth images are stored
in a separate directory (“externalize the rawlog“). In this way, the .rawlog file
can be entirely and very quickly loaded into your programs or in RawLogViewer without
needing GBs of RAM, you can inspect the images with any other standard tool, etc.

To do so, execute this command:

.. code-block:: bash

   rawlog-edit -i INPUT.rawlog -o OUTPUT.rawlog --externalize


Note: MRPT programs expect the external directory with the RGB & depth files to
be named like: `${RAWLOG_FILENAME}_Images`. This will be normally observed when
using MRPT tools, but take it into account when modifying the file names manually.

Inspect your dataset with `RawLogViewer <app_RawLogViewer.html>`_
(see the video below) or load it from your program to start with the real work!

.. raw:: html

    <div style="position: relative; padding-bottom: 56.25%; height: 0; overflow: hidden; max-width: 100%; height: auto;">
        <iframe src="https://www.youtube.com/embed/RZhMZWRplO0" frameborder="0" allowfullscreen style="position: absolute; top: 0; left: 0; width: 100%; height: 100%;"></iframe>
    </div>
