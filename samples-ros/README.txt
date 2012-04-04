=============================================================================
               How to use MRPT libraries from a ROS node?
=============================================================================

Install the mrpt_common stack [1].

Simply declare mrpt_libs [2] as a ROS dependence of your node in the 
"manifest.xml". When creating a new package, just append mrpt_libs to the 
list of dependencies, for example:

$ roscreate-pkg my_package roscpp mrpt_libs

No further modifications are needed: you don't have to 
touch your CMakeLists.txt file.


[1] http://ros.org/wiki/mrpt_common
[2] http://ros.org/wiki/mrpt_libs


Preparation:
==============

Set-up your ROS environment to find the tutorial packages:

$ cd [MRPT_DIR]/samples-ros
$ export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd)/mrpt_ros_tutorial


Build example nodes:
=====================

$ rosmake mrpt_ros_tutorial


Execute them:
================

In a console, launch "roscore". In another one:

$ rosrun mrpt_ros_tutorial example1

Optionally, in another console run:

$ rostopic echo /chatter


