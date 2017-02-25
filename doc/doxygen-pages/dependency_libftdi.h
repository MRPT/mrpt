/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

/** \page dep-libftdi External dependency: libftdi
 *

<small> <a href="dependencies.html">Back to list of dependencies</a> </small>
<br>

<h2>Dependency: <code>libftdi</code></h2>
<hr>

This optional library is needed for interfacing FTDI USB chips in the FIFO parallel mode. 

If not present, the following classes will raise exceptions upon usage:
- mrpt::hwdrivers::CInterfaceFTDI
- All its children classes.

*/

