#!/usr/bin/env python3
"""
Converts MRPT poses and pose PDFs from/to ROS geometry_msgs Pose and PoseWithCovariance.

Uses the ROS 2 (or ROS 1) geometry_msgs Python messages if available, or
minimal stand-in classes with the same fields otherwise, so it runs anywhere.
C++ code can use the equivalent functions of the mrpt_ros_bridge package.
"""

import math
from types import SimpleNamespace

import numpy as np
from mrpt.math import CMatrixDouble66
from mrpt.poses import CPose2D, CPose3D, CPose3DPDFGaussian

try:
    from geometry_msgs.msg import Pose, PoseWithCovariance
except ImportError:
    def Pose():
        return SimpleNamespace(
            position=SimpleNamespace(x=0.0, y=0.0, z=0.0),
            orientation=SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0))

    def PoseWithCovariance():
        return SimpleNamespace(pose=Pose(), covariance=[0.0] * 36)

# Covariance index of each MRPT variable (x y z yaw pitch roll) in ROS
# (x y z rot_x rot_y rot_z):
MRPT_TO_ROS_COV = [0, 1, 2, 5, 4, 3]


def pose3d_to_ros(p: CPose3D):
    yaw, pitch, roll = p.getYawPitchRoll()
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    msg = Pose()
    msg.position.x, msg.position.y, msg.position.z = p.x, p.y, p.z
    msg.orientation.w = cr * cp * cy + sr * sp * sy
    msg.orientation.x = sr * cp * cy - cr * sp * sy
    msg.orientation.y = cr * sp * cy + sr * cp * sy
    msg.orientation.z = cr * cp * sy - sr * sp * cy
    return msg


def ros_to_pose3d(msg) -> CPose3D:
    q = msg.orientation
    yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
    pitch = math.asin(max(-1.0, min(1.0, 2 * (q.w * q.y - q.z * q.x))))
    roll = math.atan2(2 * (q.w * q.x + q.y * q.z), 1 - 2 * (q.x * q.x + q.y * q.y))
    pos = msg.position
    return CPose3D.FromXYZYawPitchRoll(pos.x, pos.y, pos.z, yaw, pitch, roll)


def pose2d_to_ros(p: CPose2D):
    return pose3d_to_ros(CPose3D(p))


def ros_to_pose2d(msg) -> CPose2D:
    return CPose2D(ros_to_pose3d(msg))


def pdf3d_to_ros(pdf: CPose3DPDFGaussian):
    msg = PoseWithCovariance()
    msg.pose = pose3d_to_ros(pdf.mean)
    cov = np.array(pdf.cov)
    ros_cov = np.zeros((6, 6))
    for i in range(6):
        for j in range(6):
            ros_cov[MRPT_TO_ROS_COV[i], MRPT_TO_ROS_COV[j]] = cov[i, j]
    msg.covariance = ros_cov.flatten().tolist()
    return msg


def ros_to_pdf3d(msg) -> CPose3DPDFGaussian:
    ros_cov = np.array(msg.covariance).reshape(6, 6)
    cov = np.zeros((6, 6))
    for i in range(6):
        for j in range(6):
            cov[i, j] = ros_cov[MRPT_TO_ROS_COV[i], MRPT_TO_ROS_COV[j]]
    return CPose3DPDFGaussian(ros_to_pose3d(msg.pose), CMatrixDouble66(cov.tolist()))


def fmt(msg):
    p, q = msg.position, msg.orientation
    return 'position=({:.3f} {:.3f} {:.3f}) orientation(x y z w)=({:.4f} {:.4f} {:.4f} {:.4f})'.format(
        p.x, p.y, p.z, q.x, q.y, q.z, q.w)


# SE(2):
p1 = CPose2D(1.0, 2.0, math.radians(90.0))
ros_p1 = pose2d_to_ros(p1)
print('mrpt CPose2D        : ' + str(p1))
print('  -> ROS Pose       : ' + fmt(ros_p1))
print('  -> back to MRPT   : ' + str(ros_to_pose2d(ros_p1)))

# SE(3):
p2 = CPose3D.FromXYZYawPitchRoll(10.0, 5.0, 0.5, math.radians(30), math.radians(-10), math.radians(5))
ros_p2 = pose3d_to_ros(p2)
print('mrpt CPose3D        : ' + str(p2))
print('  -> ROS Pose       : ' + fmt(ros_p2))
print('  -> back to MRPT   : ' + str(ros_to_pose3d(ros_p2)))

# SE(3) with uncertainty:
cov = np.diag([0.1, 0.2, 0.3, 0.01, 0.02, 0.03])  # x y z yaw pitch roll
pdf = CPose3DPDFGaussian(p2, CMatrixDouble66(cov.tolist()))
ros_pdf = pdf3d_to_ros(pdf)
print('ROS covariance diag : ' + str(np.diag(np.array(ros_pdf.covariance).reshape(6, 6))))
pdf_back = ros_to_pdf3d(ros_pdf)
print('back to MRPT, diag  : ' + str(np.diag(np.array(pdf_back.cov))))
