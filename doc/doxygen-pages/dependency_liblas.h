/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

/** \page dep-liblas External dependency: liblas
 *

<small> <a href="dependencies.html">Back to list of dependencies</a> </small>
<br>

<h2>Dependency: <code>LAS (liblas)</code></h2>
<hr>

The <a href="http://www.liblas.org/" >ASPRS LiDAR LAS</a> file format for point cloud datasets. 
If available, it will provide the functionality of loading/saving mrpt::maps::CPointsMaps to LAS files.

See: \ref mrpt_maps_liblas_grp

Since MRPT 1.5.0, building MRPT against liblas is not required. In turn, user programs requiring this feature 
must make sure of adding the required compiler and linker flags to their programs, and including the 
additional file `#include <mrpt/maps/CPointsMap_liblas.h>`. 

Install libLAS in Ubuntu/Debian with: `sudo apt-get install liblas-dev liblas-c-dev`

*/

