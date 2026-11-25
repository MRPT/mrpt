\defgroup mrpt_ros2bridge_grp [mrpt-ros2bridge]

Conversion between MRPT and ROS2 types.

[TOC]

# Library mrpt-ros2bridge

As of mrpt 3.0, this C++ library is no longer part of MRPT but is available
through its own repository: https://github.com/MRPT/mrpt_ros_bridge

## Overview

The `mrpt_ros_bridge` repository contains these ROS 2 packages, available
from the ROS 2 build farm (e.g. `sudo apt install ros-$ROS_DISTRO-mrpt-libros-bridge`):

- `mrpt_libros_bridge`: the C++ library with conversion functions between
  MRPT classes and ROS 2 messages, in the namespace `mrpt::ros2bridge`.
- `rosbag2rawlog`: converts ROS 2 bags into MRPT rawlog datasets.
- `rawlog2rosbag`: converts MRPT rawlog datasets into ROS 2 bags.

Since its version 4.0.0, `mrpt_ros_bridge` requires MRPT 3.

## Conversion functions

Each header `<mrpt/ros2bridge/XXX.h>` provides overloaded `fromROS()` and
`toROS()` functions for one kind of data. Among others:

- `pose.h`: poses, poses with covariance, quaternions and tf2 transforms
  (mrpt::poses::CPose3D, mrpt::poses::CPose3DPDFGaussian, ...).
- `time.h`: ROS time stamps from/to mrpt::system::TTimeStamp.
- `point_cloud2.h`: `sensor_msgs/PointCloud2` from/to MRPT point maps.
- `laser_scan.h`, `imu.h`, `gps.h`, `image.h`, `camera_info.h`, `map.h`,
  `odometry.h`: the corresponding sensor and map messages.

In CMake, use `find_package(mrpt_libros_bridge REQUIRED)` and link against
`mrpt_libros_bridge::mrpt_libros_bridge`.

## Example

```cpp
#include <mrpt/poses/CPose3D.h>
#include <mrpt/ros2bridge/pose.h>
#include <mrpt/ros2bridge/time.h>

#include <geometry_msgs/msg/pose_stamped.hpp>

void onPose(const geometry_msgs::msg::PoseStamped& msg)
{
  const mrpt::poses::CPose3D p = mrpt::ros2bridge::fromROS(msg.pose);
  const mrpt::system::TTimeStamp t = mrpt::ros2bridge::fromROS(msg.header.stamp);

  // ... and back to ROS:
  geometry_msgs::msg::PoseStamped out;
  out.header.stamp = mrpt::ros2bridge::toROS(t);
  out.pose = mrpt::ros2bridge::toROS_Pose(p);
}
```
