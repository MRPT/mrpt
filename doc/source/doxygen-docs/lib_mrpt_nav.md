\defgroup mrpt_nav_grp [mrpt-nav]

Autonomous navigation, path planning

[TOC]

# Library mrpt-nav

This library is part of MRPT and can be installed in Debian-based systems with:

		sudo apt install libmrpt-nav-dev

Read also [how to import MRPT into your CMake scripts](mrpt_from_cmake.html).

---

## Overview

`mrpt-nav` provides reactive and planned navigation for mobile robots.
The architecture is layered:

```
CAbstractNavigator               ← base state machine (IDLE / NAVIGATING / SUSPENDED / NAV_ERROR)
  └── CWaypointsNavigator        ← waypoint sequencing on top of single-goal navigation
        └── CAbstractPTGBasedReactive   ← TP-Space reactive core
              ├── CReactiveNavigationSystem    ← 2-D robots
              └── CReactiveNavigationSystem3D  ← multi-level 3-D robots
```

---

## Reactive navigation

### Navigator state machine

mrpt::nav::CAbstractNavigator drives a state machine with four states
(mrpt::nav::CAbstractNavigator::TState):

| State | Meaning |
|-------|---------|
| `TState::IDLE` | No active navigation goal |
| `TState::NAVIGATING` | Actively navigating toward target |
| `TState::SUSPENDED` | Navigation paused (call `resume()` to continue) |
| `TState::NAV_ERROR` | An unrecoverable error occurred; query `getErrorReason()` for details |

Error codes (mrpt::nav::CAbstractNavigator::TErrorCode) are stored in the
`TErrorReason` struct returned by `getErrorReason()`:

| Code | Meaning |
|------|---------|
| `TErrorCode::NONE` | No error |
| `TErrorCode::EMERGENCY_STOP` | Robot stopped due to safety violation |
| `TErrorCode::CANNOT_REACH_TARGET` | Timeout: robot not approaching target |
| `TErrorCode::OTHER` | Unclassified exception |

### Waypoint navigation

mrpt::nav::CWaypointsNavigator extends the base navigator with a sequence of
waypoints (`TWaypointSequence`). Each waypoint can optionally be skipped if the
robot can reach a later one more directly.

Thread-safe access to the waypoint list:

```cpp
// Preferred RAII form:
auto guard = nav.getWaypointsAccessGuard();
guard.waypoints().waypoints[2].allow_skip = false;
// mutex released automatically when guard goes out of scope
```

### TP-Space reactive core

mrpt::nav::CAbstractPTGBasedReactive implements the TP-Space obstacle
transformation method:

1. For each PTG (Parameterized Trajectory Generator), obstacles in workspace
   are projected into TP-Space via `CParameterizedTrajectoryGenerator::updateTPObstacle()`.
2. The selected holonomic method (`CHolonomicVFF`, `CHolonomicND`, or
   `CHolonomicFullEval`) picks a direction in the normalized TP-Space.
3. The chosen TP-Space motion is mapped back to a velocity command via
   `CParameterizedTrajectoryGenerator::directionToMotionCommand()`.

---

## Holonomic navigation methods

All holonomic methods derive from mrpt::nav::CAbstractHolonomicReactiveMethod
and implement `navigate(NavInput, NavOutput)`.

| Class | Algorithm |
|-------|-----------|
| mrpt::nav::CHolonomicVFF | Virtual Force Fields — repulsive forces from obstacles + attractive force toward target |
| mrpt::nav::CHolonomicND  | Nearness Diagram — gap-based obstacle avoidance (Minguez & Montano, 2004) |
| mrpt::nav::CHolonomicFullEval | Full-evaluation scoring across all TP-Space directions |

The navigation situation selected by `CHolonomicND` is recorded in
`CLogFileRecord_ND::situation` (mrpt::nav::CHolonomicND::TSituations):

| Value | Meaning |
|-------|---------|
| `TSituations::TARGET_DIRECTLY` | Straight free path to target |
| `TSituations::SMALL_GAP` | Narrow gap selected |
| `TSituations::WIDE_GAP` | Wide gap selected |
| `TSituations::NO_WAY_FOUND` | No traversable gap; robot stops |

---

## Parameterized Trajectory Generators (PTGs)

PTGs define families of robot trajectories parameterized by a heading angle α.
They transform between Workspace (WS) and TP-Space. All derive from
mrpt::nav::CParameterizedTrajectoryGenerator.

| Class | Robot kinematics |
|-------|-----------------|
| `CPTG_DiffDrive_C`   | Differential drive — circular arc |
| `CPTG_DiffDrive_CS`  | Differential drive — circular arc + straight |
| `CPTG_DiffDrive_CC`  | Differential drive — two circular arcs (same direction) |
| `CPTG_DiffDrive_CCS` | Differential drive — two arcs + straight |
| `CPTG_DiffDrive_alpha` | Differential drive — trapezoidal steering |
| `CPTG_Holo_Blend`    | Holonomic robot with velocity blending |

Collision behavior when an obstacle is detected *inside* the robot shape at the
start of a PTG path is controlled globally via
`CParameterizedTrajectoryGenerator::COLLISION_BEHAVIOR()`
(mrpt::nav::PTGCollisionBehavior):

| Value | Effect |
|-------|--------|
| `PTGCollisionBehavior::BACK_AWAY` | (default) Allow reverse motions to escape near-collision |
| `PTGCollisionBehavior::STOP` | Reject any motion when robot is already in near-collision |

---

## Path planning

| Class | Algorithm |
|-------|-----------|
| mrpt::nav::PlannerSimple2D | Simple 2-D A\* on an occupancy grid |
| mrpt::nav::PlannerRRT_SE2_TPS | RRT planner in SE(2) using PTG-space expansion |

---

## Robot interface

Users must implement mrpt::nav::CRobot2NavInterface, providing:
- `getCurrentPoseAndSpeeds()` — current robot pose and velocity
- `changeSpeed()` / `stop()` — velocity commands
- Event callbacks: `sendNavigationStartEvent()`, `sendNavigationEndEvent()`, etc.

mrpt::nav::CRobot2NavInterfaceForSimulator offers a ready-made implementation
backed by a kinematic simulator.

---

# Library contents
