~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 Application: particleFilterLocalization
 
 Part of "The Mobile Robot Programming Toolkit (MRPT)"
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


--------------
 DESCRIPTION
--------------

This application implements a Particle Filter for (global) mobile robot localization in a generic
form for many kinds of map representations and sensors.

It is actually a front-end for the class mrpt::poses::CPosePDFParticles. All the parameters to the
algorithm are passed through a config file in the command line. The filter processes actions and
observations from a rawlog file and generates a number of files describing the evolution of the filter.

See the application description page for more information:
 
     http://www.mrpt.org/    app_particleFilterLocalization.html

--------------
 COMPILING
--------------
Under Linux, invoke "make" at this directory. In windows build this from the global MRPT solution.

For more information check out: 
    http://www.mrpt.org/    install.html

--------------
  CHANGE LOG
--------------

- Version 0.2, released with the MRPT version 0.4 with the name "particleFilterLocalization".
	* All the options have been moved to an external .INI file.
	
- "Global Localization Particles" Version 0.1 (2005), initial development.
 
