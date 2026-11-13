"""
Python bindings for mrpt::kinematics: vehicle kinematic models and simulators
"""
from __future__ import annotations
import mrpt.math
import mrpt.serialization
import typing
__all__: list[str] = ['CVehicleSimulVirtualBase', 'CVehicleSimul_DiffDriven', 'CVehicleSimul_Holo', 'CVehicleVelCmd', 'CVehicleVelCmd_DiffDriven', 'CVehicleVelCmd_Holo']
class CVehicleVelCmd(mrpt.serialization.CSerializable):
    """
    Base class of the velocity commands of the kinematic models (and of real robots).
    """
    def __str__(self) -> str:
        ...
    def asString(self) -> str:
        """
        Returns a human-readable description of the command.
        """
    def getVelCmdDescription(self, index: int) -> str:
        """
        Returns a description of the index-th component of the velocity command.
        """
    def getVelCmdElement(self, index: int) -> float:
        """
        Returns the index-th component of the velocity command.
        """
    def getVelCmdLength(self) -> int:
        """
        Returns the number of components of the velocity command.
        """
    def setVelCmdElement(self, index: int, value: float) -> None:
        """
        Sets the index-th component of the velocity command.
        """
class CVehicleVelCmd_DiffDriven(CVehicleVelCmd):
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
class CVehicleVelCmd_Holo(CVehicleVelCmd):
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
    def getCurrentGTVel(self) -> mrpt.math.TTwist2D:
        """
        Returns the ground truth velocity (vx, vy, omega) in world coordinates. With odometry errors enabled, this is the noise-free commanded velocity.
        """
    def getCurrentGTVelLocal(self) -> mrpt.math.TTwist2D:
        """
        Returns the ground truth velocity (vx, vy, omega) in the robot local frame.
        """
    def getCurrentOdometricPose(self) -> mrpt.math.TPose2D:
        """
        Get current odometric (noisy) pose (x, y, phi)
        """
    def getCurrentOdometricVel(self) -> mrpt.math.TTwist2D:
        """
        Returns the odometric velocity (vx, vy, omega) in the odometry frame.
        """
    def getCurrentOdometricVelLocal(self) -> mrpt.math.TTwist2D:
        """
        Returns the odometric velocity (vx, vy, omega) in the robot local frame.
        """
    def getTime(self) -> float:
        """
        Returns the current simulation time (seconds).
        """
    def getVelCmdType(self) -> CVehicleVelCmd:
        """
        Returns an empty velocity command of the type this simulator accepts.
        """
    def resetStatus(self) -> None:
        """
        Resets the poses, velocities and pending commands, but not the simulation time.
        """
    def resetTime(self) -> None:
        """
        Resets the simulation time to zero.
        """
    def sendVelCmd(self, cmd_vel: CVehicleVelCmd) -> None:
        """
        Sends a velocity command, whose type must match the simulator: CVehicleVelCmd_DiffDriven or CVehicleVelCmd_Holo.
        """
    def setCurrentGTPose(self, pose: mrpt.math.TPose2D) -> None:
        """
        Brute-force move robot to target coordinates ("teleport")
        """
    def setCurrentOdometricPose(self, pose: mrpt.math.TPose2D) -> None:
        """
        Brute-force overwrite robot odometry.
        """
    def setOdometryErrors(self, enabled: bool, Ax_err_bias: float = 0.001, Ax_err_std: float = 0.01, Ay_err_bias: float = 0.001, Ay_err_std: float = 0.01, Aphi_err_bias: float = 1.7453292519943296e-05, Aphi_err_std: float = 0.00017453292519943296) -> None:
        """
        Enables or disables odometry errors: relative factors applied to the displacement in the vehicle frame (x forward, y left, phi heading). Bias values are relative errors; std values are 1-sigma relative errors accumulated over 1 second of motion.
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
    def getV(self) -> float:
        """
        Returns the current linear velocity (m/s).
        """
    def getW(self) -> float:
        """
        Returns the current angular velocity (rad/s).
        """
    def movementCommand(self, lin_vel: float, ang_vel: float) -> None:
        """
        Set velocity command: linear (m/s) and angular (rad/s)
        """
    def setDelayModelParams(self, TAU_delay_sec: float = 1.8, CMD_delay_sec: float = 0.0) -> None:
        """
        Sets the dynamics model: velocities follow commands with a first order response of time constant TAU_delay_sec, after a pure delay of CMD_delay_sec (0, 0: immediate).
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
