#!/usr/bin/env python3

# This example shows how to convert back and forth between MRPT poses
# and ROS 1/2 Pose messages.
#
# Requires: a ROS environment with geometry_msgs available, and the
# mrpt-ros bridge package (mrpt_ros_bridge or similar).
#
# NOTE: The old mrpt.ros_bridge Python module from the monolithic pymrpt
# package has been removed. A ROS-side bridge should be used instead
# (e.g. via mrpt_msgs or the mrpt2 ROS package conversion utilities).

from math import radians
from mrpt.poses import CPose2D, CPose3D, CPose3DQuat, CPose3DPDFGaussian

# MRPT-side pose operations work standalone:
p1 = CPose2D(1.0, 2.0, radians(90.0))
p2 = CPose3D.FromXYZYawPitchRoll(10.0, 5.0, 0.0, 0.0, 0.0, 0.0)

print('mrpt CPose2D p1 : ' + str(p1))
print('mrpt CPose3D p2 : ' + str(p2))

# For ROS integration, use geometry_msgs and convert manually, e.g.:
#
#   from geometry_msgs.msg import Pose
#   ros_pose = Pose()
#   ros_pose.position.x = p2.x()
#   ros_pose.position.y = p2.y()
#   ros_pose.position.z = p2.z()
#   q = p2.getAsQuaternion()  # returns CPose3DQuat
#   ros_pose.orientation.w = q.qr()
#   ros_pose.orientation.x = q.qx()
#   ros_pose.orientation.y = q.qy()
#   ros_pose.orientation.z = q.qz()
