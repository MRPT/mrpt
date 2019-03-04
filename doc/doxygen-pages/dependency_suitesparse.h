/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          https://www.mrpt.org/                            |
   |                                                                           |
   | Copyright (c) 2005-2019, Individual contributors, see AUTHORS file        |
   | See: https://www.mrpt.org/Authors - All rights reserved.                  |
   | Released under BSD License. See details in https://www.mrpt.org/License   |
   +---------------------------------------------------------------------------+ */

/** \page dep-suitesparse External dependency: SuiteSparse
 *

<small> <a href="dependencies.html">Back to list of dependencies</a> </small>
<br>

<h2>Dependency: <code>suitesparse</code></h2>
<hr>

This suite provides many efficient sparse algebra routines, e.g. CHOLMOD.

<h3>SuiteSparse's CSparse library</h3>

Mandatory, but an embedded version exists if the library is not found in the system.

Used in these classes:
	- mrpt::math::CSparseMatrix relies on the CSparse library. 

	
*/
