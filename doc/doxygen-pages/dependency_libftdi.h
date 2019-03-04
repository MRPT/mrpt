/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          https://www.mrpt.org/                            |
   |                                                                           |
   | Copyright (c) 2005-2019, Individual contributors, see AUTHORS file        |
   | See: https://www.mrpt.org/Authors - All rights reserved.                  |
   | Released under BSD License. See details in https://www.mrpt.org/License   |
   +---------------------------------------------------------------------------+ */

/** \page dep-libftdi External dependency: libftdi
 *

<small> <a href="dependencies.html">Back to list of dependencies</a> </small>
<br>

<h2>Dependency: <code>libftdi</code></h2>
<hr>

This optional library is needed for interfacing FTDI USB chips in the FIFO parallel mode. 

If not present, the following classes will raise exceptions upon usage:
- mrpt::comms::CInterfaceFTDI
- All its children classes.

*/

