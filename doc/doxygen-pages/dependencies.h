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

/** \page dependencies External library dependencies and build options
 *

<small> <a href="index.html">Back to main page</a> </small>
<br>

<h2>List of all MRPT dependencies / build options</h2>
<hr>

<center>
<table style="border:1px solid black; " >
<tr>
	<td align="center" bgcolor="#E0E0E0" ><b>Library/Build option</b></td>
	<td align="center" bgcolor="#E0E0E0" ><b>Mandatory?</b></td>
</tr>
<tr>
	<td align="center" > \ref dep-eigen3 "eigen3" </td>
	<td align="center" > Mandatory (embedded if not external found) </td>
</tr>
<tr>
	<td align="center" > \ref dep-opencv "opencv" </td>
	<td align="center" > Optional but recommended </td>
</tr>
<tr>
	<td align="center" > \ref  dep-wxwidgets "wxWidgets" </td>
	<td align="center" > Optional but recommended</td>
</tr>
<tr>
	<td align="center" > \ref  dep-opengl-glut "opengl+glut" </td>
	<td align="center" > Optional but recommended </td>
</tr>
<tr>
	<td align="center" > \ref  dep-sse "SSE* optimizations" </td>
	<td align="center" > Optional but recommended</td>
</tr>
<tr>
	<td align="center" > \ref dep-libftdi "libftdi" </td>
	<td align="center" > Optional </td>
</tr>
<tr>
	<td align="center" > \ref dep-libusb "libfusb" </td>
	<td align="center" > Optional (needed for Kinect) </td>
</tr>
<tr>
	<td align="center" > \ref  dep-pcl "pcl" </td>
	<td align="center" > Optional </td>
</tr>
<tr>
	<td align="center" > \ref dep-suitesparse "suitesparse" </td>
	<td align="center" > Optional (but partly embedded) </td>
</tr>
</table>
</center>

*/

