from geometry_msgs.msg import Pose, PoseWithCovariance
from mrpt import pymrpt
import math

_mrpt = pymrpt.mrpt


def CPose3DQuat_to_ROS_Pose_msg(pq: _mrpt.poses.CPose3DQuat) -> Pose:
    """
    Converts from mrpt::poses::CPose3DQuat to ROS geometry_msgs.Pose
    """
    q = pq.asTPose()
    p = Pose()
    p.position.x = q.x
    p.position.y = q.y
    p.position.z = q.z
    p.orientation.x = q.qx
    p.orientation.y = q.qy
    p.orientation.z = q.qz
    p.orientation.w = q.qr
    return p


def ROS_Pose_msg_to_CPose3DQuat(p: Pose) -> _mrpt.poses.CPose3DQuat:
    """
    Converts to mrpt::poses::CPose3DQuat from ROS geometry_msgs.Pose
    """
    qq = _mrpt.math.TPose3DQuat(
        p.position.x, p.position.y, p.position.z,
        p.orientation.w, p.orientation.x, p.orientation.y, p.orientation.z)
    return _mrpt.poses.CPose3DQuat(qq)


def CPose3D_to_ROS_Pose_msg(p: _mrpt.poses.CPose3D) -> Pose:
    """
    Converts from mrpt::poses::CPose3D to ROS geometry_msgs.Pose
    """
    return CPose3DQuat_to_ROS_Pose_msg(_mrpt.poses.CPose3DQuat(p))


def ROS_Pose_msg_to_CPose3D(p: Pose) -> _mrpt.poses.CPose3D:
    """
    Converts to mrpt::poses::CPose3D from ROS geometry_msgs.Pose
    """
    return _mrpt.poses.CPose3D(ROS_Pose_msg_to_CPose3DQuat(p))


def CPose2D_to_ROS_Pose_msg(q: _mrpt.poses.CPose2D) -> Pose:
    """
    Converts from mrpt::poses::CPose2D to ROS geometry_msgs.Pose
    """
    p = Pose()
    p.position.x = q.x()
    p.position.y = q.y()
    p.position.z = 0.0
    p.orientation.x = 0.0
    p.orientation.y = 0.0
    p.orientation.z = 1.0 * math.sin(.5*q.phi())
    p.orientation.w = math.cos(.5*q.phi())
    return p


def ROS_Pose_msg_to_CPose2D(p: Pose) -> _mrpt.poses.CPose2D:
    """
    Converts to mrpt::poses::CPose2D from ROS geometry_msgs.Pose
    """
    return _mrpt.poses.CPose2D(ROS_Pose_msg_to_CPose3D(p))


#
# Read REP103: http://ros.org/reps/rep-0103.html#covariance-representation
#
# # Row-major representation of the 6x6 covariance matrix
# # The orientation parameters use a fixed-axis representation.
# # In order, the parameters are:
# # (x, y, z, rotation about X axis, rotation about Y axis, rotation about
# # Z axis)
# float64[36] covariance
#
# ==> MRPT uses non-fixed z-y-x and ROS uses fixed x-y-z rotations -->
# which
#     are equal except for the ordering --> only need to rearrange yaw,
#     pitch, roll

def ROS_PoseWithCovariance_msg_to_CPose3DPDFGaussian(p: PoseWithCovariance) -> _mrpt.poses.CPose3DPDFGaussian:
    """
    Converts to mrpt::poses::CPose3DPDFGaussian from ROS geometry_msgs.PoseWithCovariance
    """
    # rearrange covariance (see notes above)
    ind_map = [0, 1, 2, 5, 4, 3]  # X,Y,Z,YAW,PITCH,ROLL

    ret = _mrpt.poses.CPose3DPDFGaussian()
    ret.mean = ROS_Pose_msg_to_CPose3D(p.pose)

    for i in range(0, 6):
        for j in range(0, 6):
            ret.cov[i, j] = p.covariance[ind_map[i] * 6 + ind_map[j]]

    return ret


def CPose3DPDFGaussian_to_ROS_PoseWithCovariance_msg(p: _mrpt.poses.CPose3DPDFGaussian) -> PoseWithCovariance:
    """
    Converts from mrpt::poses::CPose3DPDFGaussian to ROS geometry_msgs.PoseWithCovariance
    """
    # rearrange covariance (see notes above)
    ind_map = [0, 1, 2, 5, 4, 3]  # X,Y,Z,YAW,PITCH,ROLL

    ret = PoseWithCovariance()
    ret.pose = CPose3D_to_ROS_Pose_msg(p.mean)

    for i in range(0, 6):
        for j in range(0, 6):
            ret.covariance[ind_map[i] * 6 + ind_map[j]] = p.cov[i, j]

    return ret
