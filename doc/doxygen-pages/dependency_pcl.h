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
	- mrpt::slam::CPointsMap::savePCDFile()
- In mrpt::slam::CObservation3DRangeScan (observations from 3D cameras, e.g. Kinect):
	- mrpt::slam::CObservation3DRangeScan::project3DPointsFromDepthImageInto() (*)
- Read also: <a href="http://www.mrpt.org/Generating_3D_point_clouds_from_RGB_D_observations" >"Generating 3D point clouds from RGB+D observations"</a>.
		
<b>Note:</b> (*) means that the functionality is header-only. This means that will be
 available even if MRPT is built without PCL, but the user program includes both
 PCL and MRPT headers.

*/

