~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
               The Mobile Robot Programming Toolkit (MRPT) project	 
                         http://www.mrpt.org/

  The Machine Perception and Intelligent Robotics Lab, 
    University of Malaga:                   http://babel.isa.uma.es/mapir/
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

   Contents:
    1. Introduction
       2. Documentation
    3. Compiling and installing
       3.1. Five seconds instructions
       3.2. Further details
    4. News (Changelog)
    5. License

	
1. Introduction
------------------------------------------------------------------------------

The MRPT project comprises of libraries and tools aimed to help researchers
in the areas of mobile robots and computer vision in the development of 
efficient implementations with reusability of code as a priority. 
It includes classes for easily managing 3D(6D) geometry, probability density 
functions (pdfs) over many predefined variables (points,landmarks,poses,maps,...),
Bayesian inference (Kalman filters, particle filters), image processing, obstacle
avoidance, etc...

2. Documentation
------------------------------------------------------------------------------
Further information, the latest downloads, documentation, and tutorials are 
available online at the MRPT website: 
 http://www.mrpt.org/

The complete classes reference can be found at "doc/chm/", or online at:
 http://reference.mrpt.org/

For questions on using or compiling MRPT, write in the help forums:
 http://www.mrpt.org/forum-users/


3. Compiling and installing
------------------------------------------------------------------------------

3.1 *** FIVE SECONDS INSTRUCTIONS ***

 1) Invoke cmake-gui and select: 
      Where source is          --> MRPT source root directory
	  Where to build binaries  --> Pick a new, empty directory.
	  
   Note: If your platform doesn't support cmake-gui, open a console, 
         chdir to a new empty directory and execute:
		  $ ccmake <PATH_TO_MRPT_SOURCES>

 2) Within cmake-gui (or ccmake), set all the build options as 
    you desire, then press "Configure" and "Generate". To build 
	the examples, enable "BUILD_SAMPLES".
	
 3) Build with the IDE / compiler you selected from CMake (Visual Studio, 
    GNU Make,...)
 

3.2. Further details (RECOMMENDED!)

See the document "doc/COMPILE-INSTALL.html" for instructions on how to compile 
the MRPT C++ library, the samples, applications, and also how to regenerate the 
reference documentation. Probably a more updated version will be found at:
 http://www.mrpt.org/Building_and_Installing_Instructions

 
4. News / Changelog
------------------------------------------------------------------------------
The last changes can be found in "doc/NEWS.html". The updated online version, 
generated automatically from SVN comits, can be found at:

  http://reference.mrpt.org/svn/changelog.html


  
5. LICENSE
------------------------------------------------------------------------------
License for the Mobile Robot Programming Toolkit (MRPT)

 Copyright (c) 2005-2013, MAPIR group, University of Malaga                
 All rights reserved.                                                      
                                                                           
 Redistribution and use in source and binary forms, with or without        
 modification, are permitted provided that the following conditions are    
 met:                                                                      
    * Redistributions of source code must retain the above copyright       
      notice, this list of conditions and the following disclaimer.        
    * Redistributions in binary form must reproduce the above copyright    
      notice, this list of conditions and the following disclaimer in the  
      documentation and/or other materials provided with the distribution. 
    * Neither the name of the copyright holders nor the                    
      names of its contributors may be used to endorse or promote products 
      derived from this software without specific prior written permission.
                                                                           
 THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS       
 'AS IS' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED 
 TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDERS BE LIABLE 
 FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)       
 HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,       
 STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN  
 ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE           
 POSSIBILITY OF SUCH DAMAGE.                                               

