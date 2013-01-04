/* +---------------------------------------------------------------------------+
   |                 The Mobile Robot Programming Toolkit (MRPT)               |
   |                                                                           |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2013, Individual contributors, see AUTHORS file        |
   | Copyright (c) 2005-2013, MAPIR group, University of Malaga                |
   | Copyright (c) 2012-2013, University of Almeria                            |
   | All rights reserved.                                                      |
   |                                                                           |
   | Redistribution and use in source and binary forms, with or without        |
   | modification, are permitted provided that the following conditions are    |
   | met:                                                                      |
   |    * Redistributions of source code must retain the above copyright       |
   |      notice, this list of conditions and the following disclaimer.        |
   |    * Redistributions in binary form must reproduce the above copyright    |
   |      notice, this list of conditions and the following disclaimer in the  |
   |      documentation and/or other materials provided with the distribution. |
   |    * Neither the name of the copyright holders nor the                    |
   |      names of its contributors may be used to endorse or promote products |
   |      derived from this software without specific prior written permission.|
   |                                                                           |
   | THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS       |
   | 'AS IS' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED |
   | TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR|
   | PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDERS BE LIABLE |
   | FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL|
   | DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR|
   |  SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)       |
   | HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,       |
   | STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN  |
   | ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE           |
   | POSSIBILITY OF SUCH DAMAGE.                                               |
   +---------------------------------------------------------------------------+ */

/** \page dep-pcl External dependency: PCL
 *

<small> <a href="dependencies.html">Back to list of dependencies</a> </small>
<br>

<h2>Dependency: <code>PCL</code></h2>
<hr>

The <a href="http://www.pointclouds.org/" >PointCloud library</a> (PCL) is optinal for building MRPT. 
If available, it will provide the following functionality:

- <a href="http://www.mrpt.org/Application:rawlog-edit" >rawlog-edit</a> program: The --generate-pcd command.
- In point map classes:
	- mrpt::slam::CPointsMap::getPCLPointCloud() (*)
	- mrpt::slam::CPointsMap::setFromPCLPointCloud() (*)
	- mrpt::slam::CColouredPointsMap::getPCLPointCloudRGB() (*)
	- mrpt::slam::CPointsMap::savePCDFile()
	- mrpt::slam::CPointsMap::loadPCDFile()
- In mrpt::slam::CObservation3DRangeScan (observations from 3D cameras, e.g. Kinect):
	- mrpt::slam::CObservation3DRangeScan::project3DPointsFromDepthImageInto() (*)
- Read also: <a href="http://www.mrpt.org/Generating_3D_point_clouds_from_RGB_D_observations" >"Generating 3D point clouds from RGB+D observations"</a>.
		
<b>Note:</b> (*) means that the functionality is header-only. This means that will be
 available even if MRPT is built without PCL, but the user program includes both
 PCL and MRPT headers.

*/

