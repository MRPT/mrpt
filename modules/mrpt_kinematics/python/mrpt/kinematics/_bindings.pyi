"""
Python bindings for mrpt::kinematics — vehicle kinematic models and simulators
"""
from __future__ import annotations
import mrpt.math
import typing
__all__: list[str] = ['CVehicleSimulVirtualBase', 'CVehicleSimul_DiffDriven', 'CVehicleSimul_Holo', 'CVehicleVelCmd_DiffDriven', 'CVehicleVelCmd_Holo']
class CVehicleVelCmd_DiffDriven:
    def __init__(self) -> None:
        ...
    def __repr__(self) -> str:
        ...
    def isStopCmd(self) -> bool:
        ...
    def setToStop(self) -> None:
        ...
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
    @typing.overload
    def __init__(self) -> None:
        ...
    @typing.overload
    def __init__(self, vel: float, dir_local: float, ramp_time: float, rot_speed: float) -> None:
        ...
    def __repr__(self) -> str:
        ...
    def isStopCmd(self) -> bool:
        ...
    def setToStop(self) -> None:
        ...
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
    def getCurrentGTPose(self) -> mrpt.math.TPose2D:
        """
        Get current ground-truth pose (x, y, phi)
        """
    def getCurrentOdometricPose(self) -> mrpt.math.TPose2D:
        """
        Get current odometric (noisy) pose (x, y, phi)
        """
    def setCurrentGTPose(self, pose: mrpt.math.TPose2D) -> None:
        ...
    def setCurrentOdometricPose(self, pose: mrpt.math.TPose2D) -> None:
        ...
    def simulateOneTimeStep(self, dt: float) -> None:
        """
        Advance simulation by dt seconds
        """
class CVehicleSimul_DiffDriven(CVehicleSimulVirtualBase):
    def __init__(self) -> None:
        ...
    def __repr__(self) -> str:
        ...
    def movementCommand(self, lin_vel: float, ang_vel: float) -> None:
        """
        Set velocity command: linear (m/s) and angular (rad/s)
        """
class CVehicleSimul_Holo(CVehicleSimulVirtualBase):
    def __init__(self) -> None:
        ...
    def __repr__(self) -> str:
        ...
    def sendVelRampCmd(self, vel: float, dir: float, ramp_time: float, rot_speed: float) -> None:
        """
        Send a velocity ramp command to the holonomic robot
        """
