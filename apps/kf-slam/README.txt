~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 Application: KF_SLAM
 
 Part of "The Mobile Robot Programming Toolkit (MRPT)"
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


--------------
 DESCRIPTION
--------------

This application implements a Kalman Filter-based solution to SLAM.

It is actually a front-end for the class MRML::CRangeBearingKFSLAM. All the parameters to the
algorithm are passed through a config file in the command line. The filter processes actions and
observations from a rawlog file and optionally generates a number of files describing the evolution
of the filter and the maps. 

The Kalman Filter can be selected to be a EKF, a IEKF, or a UKF.

See the application description page for more information:
 http://www.mrpt.org/list-of-mrpt-apps/application-KF-SLAM

--------------
 COMPILING
--------------
Under Linux, invoke "make" at this directory. In windows build this from the global MRPT solution.

For more information check out: 
http://www.mrpt.org/Building_and_Installing_Instructions

--------------
  CHANGE LOG
--------------

- Version 0.1, (JAN-2008) initial development.
 
