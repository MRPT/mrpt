
===============
Wrappers
===============

.. contents:: :local:


1. MRPT ROS packages
----------------------

The following ROS packages are provided wrapping MRPT functionality:

- `mrpt2 <https://wiki.ros.org/mrpt2>`_: For older Ubuntu distributions, this ROS package provides a more recent version of MRPT 2.x.x so users do not need to build it from sources.
- `mrpt_bridge <https://wiki.ros.org/mrpt_bridge>`_: Converters to/from ROS1 messages and MRPT data types.
- `mrpt_navigation <https://wiki.ros.org/mrpt_navigation>`_: Reactive autonomous navigation algorithms.
- `mrpt_sensors <https://wiki.ros.org/mrpt_sensors>`_: Reading from diverse sensors (:ref:`supported-sensors`).
- `mrpt_slam <https://wiki.ros.org/mrpt_slam>`_: Basic 2D and 3D SLAM algorithms.
- `pose_cov_ops <https://wiki.ros.org/pose_cov_ops>`_: SE(2) and SE(3) pose probability density function manipulation library.

2. Python 
----------------------

At present, all MRPT libraries are wrapped into one single Python module `pymrpt`.
Read more `here <https://github.com/MRPT/mrpt/wiki/PythonBindings>`_.


3. Matlab
----------------------

Part of MRPT can be compiled into a .mex file and used directly from MATLAB.
Read more `here <https://github.com/MRPT/mrpt/wiki/MatlabBindings>`_.
