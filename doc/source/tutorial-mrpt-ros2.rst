.. _tutorial-mrpt-ros2:

##############
MRPT and ROS 2
##############

MRPT 3 is released into ROS 2 (Humble, Jazzy, Kilted, Lyrical and Rolling) as
plain CMake libraries: one package per module, without any ROS dependency.
A set of separate repositories provide the ROS 2 side: message conversions,
nodes for localization, navigation and sensors, and a simulator.

.. contents:: :local:

1. Install
==========

.. code-block:: bash

   # Only the MRPT modules you need, e.g.:
   sudo apt install ros-$ROS_DISTRO-mrpt-maps
   # Or all MRPT libraries and apps:
   sudo apt install ros-$ROS_DISTRO-mrpt-apps-cli ros-$ROS_DISTRO-mrpt-apps-gui

   # Conversions between MRPT classes and ROS 2 messages:
   sudo apt install ros-$ROS_DISTRO-mrpt-libros-bridge

The ROS package of each module ``mrpt_<module>`` is named
``ros-$ROS_DISTRO-mrpt-<module>``. See the
`build farm status <https://github.com/MRPT/mrpt/blob/develop/packaging/ROS_BUILD_STATUS.md>`_
of every package.

2. Use MRPT from your ROS 2 package
===================================

This complete package defines a node that subscribes to a
``sensor_msgs/PointCloud2`` topic, converts each message into an MRPT point
cloud, and prints its size and bounding box.

``package.xml``: add one ``<depend>`` per MRPT module used directly (their
dependencies come along), plus ``mrpt_libros_bridge`` for message conversions:

.. code-block:: xml

   <?xml version="1.0"?>
   <package format="3">
     <name>my_mrpt_node</name>
     <version>0.1.0</version>
     <description>Example ROS 2 node using MRPT</description>
     <maintainer email="you@example.com">You</maintainer>
     <license>BSD-3-Clause</license>

     <buildtool_depend>ament_cmake</buildtool_depend>

     <depend>rclcpp</depend>
     <depend>sensor_msgs</depend>
     <depend>mrpt_maps</depend>
     <depend>mrpt_libros_bridge</depend>

     <export>
       <build_type>ament_cmake</build_type>
     </export>
   </package>

``CMakeLists.txt``: each module is a CMake package ``mrpt_<module>`` with the
target ``mrpt::mrpt_<module>`` (see :ref:`mrpt_from_cmake`):

.. code-block:: cmake

   cmake_minimum_required(VERSION 3.16)
   project(my_mrpt_node LANGUAGES CXX)

   find_package(ament_cmake REQUIRED)
   find_package(rclcpp REQUIRED)
   find_package(sensor_msgs REQUIRED)
   find_package(mrpt_maps REQUIRED)
   find_package(mrpt_libros_bridge REQUIRED)

   add_executable(cloud_stats src/cloud_stats.cpp)
   target_compile_features(cloud_stats PRIVATE cxx_std_17)
   target_link_libraries(cloud_stats
     rclcpp::rclcpp
     ${sensor_msgs_TARGETS}
     mrpt::mrpt_maps
     mrpt_libros_bridge::mrpt_libros_bridge
   )

   install(TARGETS cloud_stats DESTINATION lib/${PROJECT_NAME})
   ament_package()

``src/cloud_stats.cpp``:

.. code-block:: cpp

   #include <mrpt/maps/CSimplePointsMap.h>
   #include <mrpt/ros2bridge/point_cloud2.h>

   #include <rclcpp/rclcpp.hpp>
   #include <sensor_msgs/msg/point_cloud2.hpp>

   class CloudStats : public rclcpp::Node
   {
    public:
     CloudStats() : Node("cloud_stats")
     {
       sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
           "points", rclcpp::SensorDataQoS(),
           [this](const sensor_msgs::msg::PointCloud2& msg) { onCloud(msg); });
     }

    private:
     void onCloud(const sensor_msgs::msg::PointCloud2& msg)
     {
       // ROS message -> MRPT point cloud:
       mrpt::maps::CSimplePointsMap pts;
       if (!mrpt::ros2bridge::fromROS(msg, pts))
       {
         RCLCPP_WARN(get_logger(), "Could not convert the point cloud");
         return;
       }
       // Any MRPT algorithm can now be used on it, e.g. its bounding box:
       const auto bbox = pts.boundingBox();
       RCLCPP_INFO(
           get_logger(), "%zu points, bounding box: %s", pts.size(), bbox.asString().c_str());
     }

     rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
   };

   int main(int argc, char** argv)
   {
     rclcpp::init(argc, argv);
     rclcpp::spin(std::make_shared<CloudStats>());
     rclcpp::shutdown();
     return 0;
   }

Build and run it as any other ROS 2 package:

.. code-block:: bash

   colcon build --packages-select my_mrpt_node
   source install/setup.bash
   ros2 run my_mrpt_node cloud_stats --ros-args -r points:=/your/lidar/topic

.. code-block:: text

   [INFO] [cloud_stats]: 3 points, bounding box: [-1.000000 -2.000000 0.000000]-[3.000000 2.000000 1.000000]

3. Message conversions
======================

`mrpt_ros_bridge <https://github.com/MRPT/mrpt_ros_bridge>`_ provides overloaded
``mrpt::ros2bridge::fromROS()`` and ``toROS()`` functions, one header per kind
of data: ``pose.h`` (poses, poses with covariance, tf2 transforms), ``time.h``,
``point_cloud2.h``, ``laser_scan.h``, ``imu.h``, ``gps.h``, ``image.h``,
``camera_info.h``, ``map.h`` (occupancy grids) and ``odometry.h``. For example:

.. code-block:: cpp

   #include <mrpt/ros2bridge/pose.h>
   #include <mrpt/ros2bridge/time.h>

   const mrpt::poses::CPose3D p = mrpt::ros2bridge::fromROS(msg.pose);       // geometry_msgs/Pose
   const mrpt::system::TTimeStamp t = mrpt::ros2bridge::fromROS(msg.header.stamp);
   out.pose = mrpt::ros2bridge::toROS_Pose(p);

The same repository has two command-line tools to convert datasets:
``rosbag2rawlog`` (ROS 2 bags into MRPT :ref:`rawlogs <rawlog_format>`, e.g. to
inspect them with :ref:`app_RawLogViewer`) and ``rawlog2rosbag``.

4. ROS 2 packages built on MRPT
===============================

.. list-table::
  :header-rows: 1
  :widths: 28 72

  * - Repository
    - Contents
  * - `mrpt_navigation <https://github.com/mrpt-ros-pkg/mrpt_navigation>`_
    - Nodes for particle filter localization (``mrpt_pf_localization``), map
      servers (``mrpt_map_server``), point cloud pipelines, TPS-A* path planning,
      reactive navigation (``mrpt_reactivenav2d``) and trajectory following, plus
      runnable demos in ``mrpt_tutorials``.
  * - `mrpt_sensors <https://github.com/mrpt-ros-pkg/mrpt_sensors>`_
    - ROS 2 nodes for the sensors supported by MRPT drivers (GNSS, IMUs, cameras...).
  * - `mrpt_path_planning <https://github.com/MRPT/mrpt_path_planning>`_
    - Kinematically-feasible path planning for arbitrary robot shapes.
  * - `MVSim <https://github.com/MRPT/mvsim>`_
    - Lightweight multi-robot simulator with LiDARs, cameras and IMUs, publishing
      ROS 2 topics.
  * - `MOLA <https://github.com/MOLAorg/mola>`_
    - LiDAR odometry, SLAM and localization. See :ref:`tutorial-mrpt-mola`.

For example, after installing ``ros-$ROS_DISTRO-mvsim`` and
``ros-$ROS_DISTRO-mrpt-navigation``:

.. code-block:: bash

   # A simulated warehouse with a robot and its sensors:
   ros2 launch mvsim demo_warehouse.launch.py
   # Particle filter localization of a simulated robot with a 2D LiDAR:
   ros2 launch mrpt_tutorials demo_localization_pf_mvsim_2d_lidar.launch.py

Porting ROS 2 nodes written for MRPT 2.x (``mrpt_ros``)? Read the last section
of :doc:`page_porting_mrpt3`.
