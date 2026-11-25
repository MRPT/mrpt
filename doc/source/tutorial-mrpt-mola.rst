.. _tutorial-mrpt-mola:

#################
MRPT inside MOLA
#################

`MOLA <https://github.com/MOLAorg/mola>`_ (a Modular Optimization framework for
Localization and mApping) is a framework for LiDAR odometry, SLAM and
localization built on top of MRPT. It runs on ROS 2 or standalone, on live
sensors, ROS bags or datasets. Its documentation is at
`docs.mola-slam.org <https://docs.mola-slam.org/>`_.

What MOLA uses from MRPT
========================

- **Observations and datasets**: sensor data travel through MOLA as MRPT
  observations (mrpt::obs::CObservationPointCloud, CObservationIMU,
  CObservationGPS...), so MOLA reads MRPT :ref:`rawlogs <rawlog_format>`, ROS 2
  bags and public datasets with the same pipeline.
- **Poses and geometry**: mrpt::poses and mrpt::math for SE(3) poses,
  uncertainty and Lie group operations.
- **Maps and registration**: point cloud maps (mrpt::maps) and
  `mp2p_icp <https://github.com/MOLAorg/mp2p_icp>`_, the ICP library behind
  MOLA LiDAR odometry, which builds on MRPT maps and ``mrpt::tfest``.
- **Visualization**: mrpt::viz scenes and the MRPT GUI libraries for the MOLA
  GUI and its map viewers.

So if you already have MRPT datasets or code, MOLA can use them directly.

Try MOLA LiDAR odometry
=======================

With ROS 2 sourced:

.. code-block:: bash

   sudo apt install \
     ros-$ROS_DISTRO-mola \
     ros-$ROS_DISTRO-mola-lidar-odometry \
     ros-$ROS_DISTRO-mola-test-datasets

   # Run LiDAR odometry with GUI on a short sample (a rawlog):
   mola-lo-gui-rawlog \
     $(ros2 pkg prefix mola_test_datasets)/share/mola_test_datasets/datasets/mulran/mulran_KAIST01_extract.rawlog

A window shows the point cloud and the estimated trajectory. On your own data,
``mola-lo-gui-rosbag2 your_dataset.mcap`` does the same with a ROS 2 bag, and
``mola-lo-gui-rawlog`` accepts any rawlog recorded with MRPT, e.g. with
:doc:`rawlog-grabber <page_app_rawlog-grabber>`. For live operation as a ROS 2
node, mapping, localization on a prior map and all configuration options, see
the `MOLA documentation <https://docs.mola-slam.org/>`_.

See also: :ref:`tutorial-mrpt-ros2` and :ref:`children-projects`.
