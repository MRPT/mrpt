~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 Application: rbpf-slam
 
 Part of "The Mobile Robot Programming Toolkit (MRPT)"
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


--------------
 DESCRIPTION
--------------

This application implements a Rao-Blackwellized Particle Filter (RBPF)-based solution to SLAM.

It is actually a front-end for the class MRML::CMetricMapBuilderRBPF. All the parameters to the
algorithm are passed through a config file in the command line. The filter processes actions and
observations from a rawlog file and optionally generates a number of files describing the evolution
of the filter and the maps.

See the application description page for more information:
 
 http://www.mrpt.org/list-of-mrpt-apps/application-RBPF-SLAM

--------------
 COMPILING
--------------
Under Linux, invoke "make" at this directory. In windows build this from the global MRPT solution.

For more information check out: 
http://www.mrpt.org/Building_and_Installing_Instructions

--------------
  CHANGE LOG
--------------

- Version 0.2, released with the MRPT version 0.4 with the name "RBPF_SLAM".
	* All the options have been moved to an external .INI file.
	
- MapBuildingFromRawLog Version 0.1 (2005-2006), initial development.
 
