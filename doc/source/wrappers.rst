
===============
Wrappers
===============

.. contents:: :local:


1. MRPT ROS packages
----------------------
The following ROS packages are provided wrapping MRPT functionality:

- MRPT libraries and pymrpt:
  - ROS 1: `mrpt2 <https://wiki.ros.org/mrpt2>`_
  - ROS 2: See instructions in: https://github.com/MRPT/mrpt_ros
- `mrpt_msgs <https://github.com/mrpt-ros-pkg/mrpt_msgs>`_: ROS messages for MRPT classes and objects
- `mrpt_navigation <https://wiki.ros.org/mrpt_navigation>`_: Reactive autonomous navigation algorithms.
- `mrpt_sensors <https://wiki.ros.org/mrpt_sensors>`_: Reading from diverse sensors (:ref:`supported-sensors`).
- `mrpt_slam <https://wiki.ros.org/mrpt_slam>`_: Basic 2D and 3D SLAM algorithms.
- `pose_cov_ops <https://wiki.ros.org/pose_cov_ops>`_: SE(2) and SE(3) pose probability density function manipulation library.

2. Python 
----------------------
All MRPT libraries are wrapped into one single Python3 module `pymrpt`.
See: 

- pydoc reference documentation for pymrpt: https://mrpt.github.io/pymrpt-docs/mrpt.pymrpt.mrpt.html
- MRPT `Python examples <python_examples.html>`_


3. Matlab
----------------------
Part of MRPT can be compiled into a .mex file and used directly from MATLAB.
Read more `here <https://github.com/MRPT/mrpt/wiki/MatlabBindings>`_.
