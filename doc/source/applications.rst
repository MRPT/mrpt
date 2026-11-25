.. _applications:

##############
Applications
##############

Besides its :ref:`modules` for writing your own programs, MRPT ships ready-to-use
applications, installed with the ``mrpt_apps_cli`` and ``mrpt_apps_gui``
packages (ROS 2), the ``mrpt-apps`` package (Ubuntu PPA), or built from sources.
See :ref:`downloadmrpt`.

.. toctree::
  :hidden:
  :maxdepth: 1

  app_RawLogViewer.rst
  page_app_SceneViewer3D.rst
  page_app_navlog-viewer.rst
  app_3d-rotation-converter.rst
  app_robotic-arm-kinematics.rst
  page_app_robot-map-gui.rst
  page_app_icp-slam.rst
  page_app_rbpf-slam.rst
  page_app_kf-slam.rst
  app_2d-slam-demo.rst
  page_app_pf-localization.rst
  page_app_ro-localization.rst
  page_app_icp-slam-live.rst
  page_app_graph-slam.rst
  page_app_map-partition.rst
  page_app_graphslam-engine.rst
  page_app_ReactiveNavigationDemo.rst
  page_app_ReactiveNav3D-Demo.rst
  page_app_holonomic-navigator-demo.rst
  page_app_ptg-configurator.rst
  app_GridmapNavSimul.rst
  page_app_rawlog-edit.rst
  page_app_rawlog-grabber.rst
  page_app_observations2map.rst
  page_app_image2gridmap.rst
  page_app_ros-map-yaml2mrpt.rst
  page_app_gps2rawlog.rst
  page_app_carmen2rawlog.rst
  page_app_carmen2simplemap.rst
  page_app_mrpt-ini2yaml.rst
  page_app_mrpt-yaml2ini.rst
  page_app_simul-landmarks.rst
  page_app_simul-beacons.rst
  page_app_simul-gridmap.rst
  page_app_velodyne-view.rst
  page_app_mrpt-performance.rst

Viewers and GUI tools
=====================

.. grid:: 1 2 3 3
  :gutter: 3

  .. grid-item-card:: RawLogViewer
    :link: app_RawLogViewer
    :link-type: doc
    :img-top: images/app_RawLogViewer_imgui.webp
    :class-card: mrpt-app-card

    Inspect, edit and plot datasets (rawlogs): sensor data, odometry, timing.

  .. grid-item-card:: SceneViewer3D
    :link: page_app_SceneViewer3D
    :link-type: doc
    :img-top: images/app_SceneViewer3D_imgui.webp
    :class-card: mrpt-app-card

    View 3D scenes, 3D models and point clouds; render videos and snapshots.

  .. grid-item-card:: navlog-viewer
    :link: page_app_navlog-viewer
    :link-type: doc
    :img-top: images/app_navlog-viewer_imgui.webp
    :class-card: mrpt-app-card

    Replay and debug the logs of the reactive navigation engine.

  .. grid-item-card:: 3d-rotation-converter
    :link: app_3d-rotation-converter
    :link-type: doc
    :img-top: https://mrpt.github.io/docs-assets/apps/app_3d-rotation-converter.webp
    :class-card: mrpt-app-card

    Convert between 3D rotation parameterizations, and visualize them.

  .. grid-item-card:: robotic-arm-kinematics
    :link: app_robotic-arm-kinematics
    :link-type: doc
    :img-top: https://mrpt.github.io/docs-assets/apps/app_robotic-arm-kinematics.webp
    :class-card: mrpt-app-card

    Define robotic arms with Denavit-Hartenberg parameters and move their joints.

  .. grid-item-card:: robot-map-gui
    :link: page_app_robot-map-gui
    :link-type: doc
    :class-card: mrpt-app-card

    Open, inspect and edit simple maps (``.simplemap``) and the maps built from them.

SLAM and localization
=====================

.. grid:: 1 2 3 3
  :gutter: 3

  .. grid-item-card:: icp-slam
    :link: page_app_icp-slam
    :link-type: doc
    :img-top: https://mrpt.github.io/docs-assets/apps/app_icp-slam.webp
    :class-card: mrpt-app-card

    2D/3D map building from a dataset with ICP scan matching.

  .. grid-item-card:: rbpf-slam
    :link: page_app_rbpf-slam
    :link-type: doc
    :img-top: https://mrpt.github.io/docs-assets/apps/app_rbpf-slam.webp
    :class-card: mrpt-app-card

    Rao-Blackwellized particle filter SLAM: grid maps, landmarks, range-only.

  .. grid-item-card:: kf-slam
    :link: page_app_kf-slam
    :link-type: doc
    :img-top: https://mrpt.github.io/docs-assets/apps/app_kf-slam.webp
    :class-card: mrpt-app-card

    EKF-based SLAM with 2D or 3D landmarks.

  .. grid-item-card:: 2d-slam-demo
    :link: app_2d-slam-demo
    :link-type: doc
    :img-top: images/app_2d-slam-demo_imgui.webp
    :class-card: mrpt-app-card

    Interactive EKF-SLAM simulator, to learn and compare data association methods.

.. grid:: 1 2 3 4
  :gutter: 2

  .. grid-item-card:: pf-localization
    :link: page_app_pf-localization
    :link-type: doc
    :class-card: mrpt-app-card-text

    Particle filter localization on a known map.

  .. grid-item-card:: ro-localization
    :link: page_app_ro-localization
    :link-type: doc
    :class-card: mrpt-app-card-text

    Range-only localization with beacons.

  .. grid-item-card:: icp-slam-live
    :link: page_app_icp-slam-live
    :link-type: doc
    :class-card: mrpt-app-card-text

    ICP-based SLAM with live sensors.

  .. grid-item-card:: graph-slam
    :link: page_app_graph-slam
    :link-type: doc
    :class-card: mrpt-app-card-text

    Optimize and view pose graphs.

  .. grid-item-card:: map-partition
    :link: page_app_map-partition
    :link-type: doc
    :class-card: mrpt-app-card-text

    Split a map into sub-maps with spectral graph partitioning.

  .. grid-item-card:: graphslam-engine
    :link: page_app_graphslam-engine
    :link-type: doc
    :class-card: mrpt-app-card-text

    Graph-SLAM engine (not built in MRPT 3 yet).

Navigation
==========

.. grid:: 1 2 3 3
  :gutter: 3

  .. grid-item-card:: ReactiveNavigationDemo
    :link: page_app_ReactiveNavigationDemo
    :link-type: doc
    :img-top: https://mrpt.github.io/docs-assets/apps/app_ReactiveNavigationDemo.webp
    :class-card: mrpt-app-card

    Simulator to test and tune the reactive navigation engine.

  .. grid-item-card:: ReactiveNav3D-Demo
    :link: page_app_ReactiveNav3D-Demo
    :link-type: doc
    :img-top: https://mrpt.github.io/docs-assets/apps/app_ReactiveNav3D-Demo.webp
    :class-card: mrpt-app-card

    Reactive navigation of robots with a 3D (multi-height) shape.

  .. grid-item-card:: holonomic-navigator-demo
    :link: page_app_holonomic-navigator-demo
    :link-type: doc
    :img-top: images/app_holonomic-navigator-demo_imgui.webp
    :class-card: mrpt-app-card

    Interactive demo of the holonomic obstacle avoidance methods.

  .. grid-item-card:: ptg-configurator
    :link: page_app_ptg-configurator
    :link-type: doc
    :img-top: https://mrpt.github.io/docs-assets/apps/app_ptg-configurator.webp
    :class-card: mrpt-app-card

    Design and visualize Parameterized Trajectory Generators (PTGs).

  .. grid-item-card:: GridmapNavSimul
    :link: app_GridmapNavSimul
    :link-type: doc
    :img-top: images/app_GridmapNavSimul_imgui.webp
    :class-card: mrpt-app-card

    Drive a simulated robot in a grid map to generate synthetic datasets.

Datasets, maps and conversion tools
===================================

.. grid:: 1 2 3 4
  :gutter: 2

  .. grid-item-card:: rawlog-edit
    :link: page_app_rawlog-edit
    :link-type: doc
    :class-card: mrpt-app-card-text

    Swiss army knife for datasets: filter, cut, convert, export.

  .. grid-item-card:: rawlog-grabber
    :link: page_app_rawlog-grabber
    :link-type: doc
    :class-card: mrpt-app-card-text

    Record datasets from any set of sensors.

  .. grid-item-card:: observations2map
    :link: page_app_observations2map
    :link-type: doc
    :class-card: mrpt-app-card-text

    Build maps from a simple map (poses and observations).

  .. grid-item-card:: image2gridmap
    :link: page_app_image2gridmap
    :link-type: doc
    :class-card: mrpt-app-card-text

    Convert an image into an occupancy grid map.

  .. grid-item-card:: ros-map-yaml2mrpt
    :link: page_app_ros-map-yaml2mrpt
    :link-type: doc
    :class-card: mrpt-app-card-text

    Convert ROS map_server maps into MRPT grid maps.

  .. grid-item-card:: gps2rawlog
    :link: page_app_gps2rawlog
    :link-type: doc
    :class-card: mrpt-app-card-text

    Convert GPS/GNSS NMEA logs into a rawlog.

  .. grid-item-card:: carmen2rawlog
    :link: page_app_carmen2rawlog
    :link-type: doc
    :class-card: mrpt-app-card-text

    Convert CARMEN logs into a rawlog.

  .. grid-item-card:: carmen2simplemap
    :link: page_app_carmen2simplemap
    :link-type: doc
    :class-card: mrpt-app-card-text

    Convert CARMEN logs with corrected poses into a simple map.

  .. grid-item-card:: mrpt-ini2yaml
    :link: page_app_mrpt-ini2yaml
    :link-type: doc
    :class-card: mrpt-app-card-text

    Convert an INI configuration file into YAML.

  .. grid-item-card:: mrpt-yaml2ini
    :link: page_app_mrpt-yaml2ini
    :link-type: doc
    :class-card: mrpt-app-card-text

    Convert a YAML configuration file into INI.

Simulation, sensors and benchmarks
==================================

.. grid:: 1 2 3 4
  :gutter: 2

  .. grid-item-card:: simul-landmarks
    :link: page_app_simul-landmarks
    :link-type: doc
    :class-card: mrpt-app-card-text

    Simulate a robot observing landmarks, for SLAM benchmarks.

  .. grid-item-card:: simul-beacons
    :link: page_app_simul-beacons
    :link-type: doc
    :class-card: mrpt-app-card-text

    Simulate range-only beacon observations.

  .. grid-item-card:: simul-gridmap
    :link: page_app_simul-gridmap
    :link-type: doc
    :class-card: mrpt-app-card-text

    Simulate laser scans along a path in a grid map.

  .. grid-item-card:: velodyne-view
    :link: page_app_velodyne-view
    :link-type: doc
    :class-card: mrpt-app-card-text

    View live Velodyne LiDAR data or PCAP files, and record datasets.

  .. grid-item-card:: mrpt-performance
    :link: page_app_mrpt-performance
    :link-type: doc
    :class-card: mrpt-app-card-text

    Benchmark the performance of MRPT algorithms.

.. index::
   single: List of MRPT applications
