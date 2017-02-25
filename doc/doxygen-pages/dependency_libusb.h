/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

/** \page dep-libusb External dependency: libusb
 *

<small> <a href="dependencies.html">Back to list of dependencies</a> </small>
<br>

<h2>Dependency: <code>libusb</code></h2>
<hr>

This optional library is needed as a dependency of the embedded version of libfreenect for communicating with Kinect sensors. 

If not present, the following classes will raise exceptions upon usage:
- mrpt::hwdrivers::CKinect ( unless a different underlying implementation is used, see http://www.mrpt.org/Kinect_and_MRPT )

*/

