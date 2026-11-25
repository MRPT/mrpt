\defgroup mrpt_kinematics_grp [mrpt-kinematics]

Kinematics of robot arms and mobile robots.

[TOC]

# Library mrpt-kinematics

This library is part of MRPT and can be installed in Debian-based systems with:

		sudo apt install libmrpt-kinematics-dev

Read also [how to import MRPT into your CMake scripts](mrpt_from_cmake.html).

Classes for defining and managing the kinematics of robotic arms.

See mrpt::kinematics

## Overview

This module covers two different topics:

- **Robotic arms**: open-loop kinematic chains described with
  Denavit-Hartenberg parameters, with forward kinematics and a 3D
  representation for visualization.
- **Planar mobile robots**: velocity commands for differential-drive and
  holonomic robots, and simple kinematic simulators of both, used for example
  by the reactive navigators in \ref mrpt_nav_grp.

## Key classes

- mrpt::kinematics::CKinematicChain: a chain of revolute or prismatic links;
  `recomputeAllPoses()` computes the pose of every link, and
  `getAs3DObject()` builds an mrpt::viz representation.
- mrpt::kinematics::TKinematicLink: one link, with its Denavit-Hartenberg
  parameters `[theta, d, a, alpha]`.
- mrpt::kinematics::CVehicleVelCmd_DiffDriven,
  mrpt::kinematics::CVehicleVelCmd_Holo: velocity commands for each kind of
  mobile base.
- mrpt::kinematics::CVehicleSimul_DiffDriven,
  mrpt::kinematics::CVehicleSimul_Holo: simulators that integrate velocity
  commands over time, with optional odometry errors.

## Example: forward kinematics of an arm

```cpp
#include <mrpt/core/bits_math.h>
#include <mrpt/kinematics/CKinematicChain.h>

mrpt::kinematics::CKinematicChain arm;
// Denavit-Hartenberg parameters: theta, d, a, alpha, is_prismatic
arm.addLink(0.0, 0.3, 0.0, mrpt::DEG2RAD(90.0), false);
arm.addLink(0.0, 0.0, 0.5, 0.0, false);
arm.addLink(0.0, 0.0, 0.4, 0.0, false);

// Joint values (radians for revolute joints, meters for prismatic ones):
arm.setConfiguration(std::vector<double>{0.1, -0.4, 0.8});

// poses[0] is the base, poses.back() the end effector.
std::vector<mrpt::poses::CPose3D> poses;
arm.recomputeAllPoses(poses);
```

## Examples

- The GUI application [robotic-arm-kinematics](app_robotic-arm-kinematics.html)
  lets you edit a kinematic chain interactively.
- Python: [mrpt_kinematics_example.py](pymrpt_example_mrpt_kinematics_example.html).

# Library contents
