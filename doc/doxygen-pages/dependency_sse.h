/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

/** \page dep-sse Optimizations: SSE*
 *

<small> <a href="dependencies.html">Back to list of dependencies</a> </small>
<br>

<h2>Dependency: <code>SSE2/SSE3/SSE4 optimized instruction sets</code></h2>
<hr>

By default, all SSE* optimizations are enabled. If your processor does not support them you will have to disable them 
by enabling viewing the "Advanced" options from CMake and checking the corresponding <code>DISABLE_SSE*</code> option. 

You'll know that your processor does not support SSE* instructions if you see unexplainable <b>illegal instruction errors</b> when executing MRPT programs.

See <a href="group__sse__optimizations.html" >SEE optimizations</a> to learn which functions have explicit vectorization optimizations in MRPT.

*/

