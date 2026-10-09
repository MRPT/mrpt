"""
Python bindings for mrpt::kinematics — vehicle kinematic models and simulators
"""
from __future__ import annotations
import mrpt.math
import typing
__all__: list[str] = ['CVehicleSimulVirtualBase', 'CVehicleSimul_DiffDriven', 'CVehicleSimul_Holo', 'CVehicleVelCmd_DiffDriven', 'CVehicleVelCmd_Holo']
class CVehicleVelCmd_DiffDriven:
    """
    Kinematic model for Ackermann-like or differential-driven vehicles.
    """
    def __init__(self) -> None:
        """
        Default constructor.
        """
    def __repr__(self) -> str:
        ...
    def isStopCmd(self) -> bool:
        """
        Returns true if the command means "do not move" / "stop".
        """
    def setToStop(self) -> None:
        """
        Set to a command that means "do not move" / "stop".
        """
    @property
    def ang_vel(self) -> float:
        """
        Angular velocity (rad/s)
        """
    @ang_vel.setter
    def ang_vel(self, arg0: float) -> None:
        ...
    @property
    def lin_vel(self) -> float:
        """
        Linear velocity (m/s)
        """
    @lin_vel.setter
    def lin_vel(self, arg0: float) -> None:
        ...
class CVehicleVelCmd_Holo:
    """
    Velocity command for holonomic robots: speed, direction, rotational speed and ramp time.
    """
    @typing.overload
    def __init__(self) -> None:
        """
        Default constructor.
        """
    @typing.overload
    def __init__(self, vel: float, dir_local: float, ramp_time: float, rot_speed: float) -> None:
        """
        Builds a command from speed, direction (rad), ramp time (s) and rotational speed (rad/s).
        """
    def __repr__(self) -> str:
        ...
    def isStopCmd(self) -> bool:
        """
        Returns true if the command means "do not move" / "stop".
        """
    def setToStop(self) -> None:
        """
        Set to a command that means "do not move" / "stop".
        """
    @property
    def dir_local(self) -> float:
        """
        Direction relative to robot heading (radians)
        """
    @dir_local.setter
    def dir_local(self, arg0: float) -> None:
        ...
    @property
    def ramp_time(self) -> float:
        """
        Blending time (seconds)
        """
    @ramp_time.setter
    def ramp_time(self, arg0: float) -> None:
        ...
    @property
    def rot_speed(self) -> float:
        """
        Rotational speed for heading correction (rad/s)
        """
    @rot_speed.setter
    def rot_speed(self, arg0: float) -> None:
        ...
    @property
    def vel(self) -> float:
        """
        Linear speed (m/s)
        """
    @vel.setter
    def vel(self, arg0: float) -> None:
        ...
class CVehicleSimulVirtualBase:
    """
    Base class of the 2D robot kinematic simulators, including odometry errors.
    """
    def getCurrentGTPose(self) -> mrpt.math.TPose2D:
        """
        Get current ground-truth pose (x, y, phi)
        """
    def getCurrentOdometricPose(self) -> mrpt.math.TPose2D:
        """
        Get current odometric (noisy) pose (x, y, phi)
        """
    def setCurrentGTPose(self, pose: mrpt.math.TPose2D) -> None:
        """
        Brute-force move robot to target coordinates ("teleport")
        """
    def setCurrentOdometricPose(self, pose: mrpt.math.TPose2D) -> None:
        """
        Brute-force overwrite robot odometry.
        """
    def simulateOneTimeStep(self, dt: float) -> None:
        """
        Advance simulation by dt seconds
        """
class CVehicleSimul_DiffDriven(CVehicleSimulVirtualBase):
    """
    Simulates the kinematics of a differential-driven planar mobile robot/vehicle, including odometry errors and dynamics limitations.
    """
    def __init__(self) -> None:
        """
        Default constructor.
        """
    def __repr__(self) -> str:
        ...
    def movementCommand(self, lin_vel: float, ang_vel: float) -> None:
        """
        Set velocity command: linear (m/s) and angular (rad/s)
        """
class CVehicleSimul_Holo(CVehicleSimulVirtualBase):
    """
    Kinematic simulator of a holonomic 2D robot capable of moving in any direction, with "blended" velocity profiles.
    """
    def __init__(self) -> None:
        """
        Default constructor.
        """
    def __repr__(self) -> str:
        ...
    def sendVelRampCmd(self, vel: float, dir: float, ramp_time: float, rot_speed: float) -> None:
        """
        Send a velocity ramp command to the holonomic robot
        """
