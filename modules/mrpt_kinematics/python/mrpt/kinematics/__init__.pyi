"""
mrpt.kinematics — Vehicle kinematic models and simulators.

Provides:
  - CVehicleVelCmd_DiffDriven : Velocity command for differential-drive robots
  - CVehicleVelCmd_Holo       : Velocity command for holonomic robots
  - CVehicleSimulVirtualBase  : Abstract base for vehicle simulators
  - CVehicleSimul_DiffDriven  : Differential-drive robot simulator
  - CVehicleSimul_Holo        : Holonomic robot simulator

Example — simulate a diff-drive robot::

    import mrpt.kinematics as kin
    import mrpt.math

    robot = kin.CVehicleSimul_DiffDriven()
    robot.movementCommand(lin_vel=0.5, ang_vel=0.1)
    for _ in range(100):
        robot.simulateOneTimeStep(dt=0.01)
    pose = robot.getCurrentGTPose()
    print(f"Pose: ({pose.x:.3f}, {pose.y:.3f}, {pose.phi:.3f})")
"""
from __future__ import annotations
import mrpt as mrpt
from mrpt.kinematics._bindings import CVehicleSimulVirtualBase as CVehicleSimulVirtualBase
from mrpt.kinematics._bindings import CVehicleSimul_DiffDriven as CVehicleSimul_DiffDriven
from mrpt.kinematics._bindings import CVehicleSimul_Holo as CVehicleSimul_Holo
from mrpt.kinematics._bindings import CVehicleVelCmd_DiffDriven as CVehicleVelCmd_DiffDriven
from mrpt.kinematics._bindings import CVehicleVelCmd_Holo as CVehicleVelCmd_Holo
from . import _bindings
__all__: list = ['CVehicleVelCmd_DiffDriven', 'CVehicleVelCmd_Holo', 'CVehicleSimulVirtualBase', 'CVehicleSimul_DiffDriven', 'CVehicleSimul_Holo']
