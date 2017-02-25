/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

/** \page dep-opencv External dependency: opencv
 *

<small> <a href="dependencies.html">Back to list of dependencies</a> </small>
<br>

<h2>Dependency: <code>opencv</code></h2>
<hr>

This optional library is used for almost everything related to computer vision in MRPT. Even MRPT-specific computer vision functions or classes cannot work without OpenCV since the basic structure for holding images is OpenCV's <code>IplImage</code>. 

If not present, the following classes will raise an exception upon usage:
- mrpt::utils::CImage. Note that creating mrpt::utils::CImage objects in the "external storage mode" will not raise an exception unless you really access the image contents.
- Everything needing access to an image (loading, saving, processing, etc.). 

*/

