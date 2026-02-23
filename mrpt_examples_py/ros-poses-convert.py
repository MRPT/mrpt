#!/usr/bin/env python3

# ---------------------------------------------------------------------
# Install python3-pymrpt, ros-$ROS_DISTRO-python-mrpt,
# ros-$ROS_DISTRO-mrpt2, or test with a local build with:
# export PYTHONPATH=$HOME/code/mrpt/build-Release/:$PYTHONPATH
# ---------------------------------------------------------------------

# This example shows how to convert back and forth between MRPT poses
# and ROS 1 or ROS 2 (both are compatible with this same code) Pose

from mrpt.pymrpt import mrpt
from mrpt import ros_bridge
from math import radians
from geometry_msgs.msg import Pose, PoseWithCovariance


# Example 1: 2D pose
# --------------------------
p1 = mrpt.poses.CPose2D(1.0, 2.0, radians(90.0))
rp1 = ros_bridge.CPose2D_to_ROS_Pose_msg(p1)

p2 = ros_bridge.ROS_Pose_msg_to_CPose3D(rp1)

print('mrpt p1             : ' + str(p1))
print('ros  p1             : ' + str(rp1))
print('mrpt p2             : ' + str(p2))

# Example 2: 3D pose
# ----------------------
r2 = Pose()
r2.position.x = 10.0
r2.position.y = 5.0
r2.position.z = 0.0
r2.orientation.w = 1.0
r2.orientation.x = 0.0
r2.orientation.y = 0.0
r2.orientation.z = 0.0

mr2 = ros_bridge.ROS_Pose_msg_to_CPose3D(r2)
mr2b = ros_bridge.ROS_Pose_msg_to_CPose3DQuat(r2)

print()
print('ros  r2             : ' + str(r2))
print('mrpt mr2            : ' + str(mr2))
print('mrpt mr2b           : ' + str(mr2b))

# Example 3: with covariance
# -------------------------------
r3 = PoseWithCovariance()
r3.pose = r2
r3.covariance = [1.0, 0.0, 0.0,  0.0, 0.0, 0.0,
                 0.0, 1.0, 0.0,  0.0, 0.0, 0.0,
                 0.0, 0.0, 1.0,  0.0, 0.0, 0.0,
                 0.0, 0.0, 0.0,  1.0, 0.0, 0.0,
                 0.0, 0.0, 0.0,  0.0, 1.0, 0.0,
                 0.0, 0.0, 0.0,  0.0, 0.0, 1.0]

mr3 = ros_bridge.ROS_PoseWithCovariance_msg_to_CPose3DPDFGaussian(r3)

mr3b = mr3
mr3b.mean.x(mr3b.mean.x()+1.0)
r3b = ros_bridge.CPose3DPDFGaussian_to_ROS_PoseWithCovariance_msg(mr3)

print()
print('ros PDF r3         : ' + str(r3))
print('mrpt PDF mr3       : ' + str(mr3))
print('mrpt PDF mr3b      : ' + str(mr3b))
print('ros PDF r3b        : ' + str(r3b))

a = mr3
b = mr3
c = a+b

print('r3b (+) rb3        : ' + str(c))
