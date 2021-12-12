\defgroup mrpt_nav_grp [mrpt-nav]

Autonomous navigation, path planning

[TOC]

# Library mrpt-nav

This library is part of MRPT and can be installed in Debian-based systems with:

		sudo apt install libmrpt-nav-dev

Read also [how to import MRPT into your CMake scripts](mrpt_from_cmake.html).

This library implements:

- Reactive navigation:
  - Holonomic navigation algorithms: Virtual Force Fields (VFF), Nearness
Diagram (ND), ... See mrpt::nav::CAbstractHolonomicReactiveMethod
  - A complex reactive navigator: Using space transformations (PTGs) to drive a
robot using an internal simpler holonomic algorithm. See
mrpt::nav::CReactiveNavigationSystem
  - A number of different PTGs: See mrpt::nav::CParameterizedTrajectoryGenerator
  - See the full list of classes in mrpt::nav, or the online page
https://www.mrpt.org/list-of-mrpt-apps/application-reactivenavigationdemo/ for a
working application (see MRPT/apps/ReactiveNavigationDemo).

* Planned / hybrid navigation:
  * See mrpt::nav

# Library contents
