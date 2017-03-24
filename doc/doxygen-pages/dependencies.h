/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
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
	<td align="center" > \ref dep-libpcap "libpcap (or WinPCap)" </td>
	<td align="center" > Optional </td>
</tr>
<tr>
	<td align="center" > \ref dep-libusb "libusb" </td>
	<td align="center" > Optional (needed for Kinect) </td>
</tr>
<tr>
	<td align="center" > \ref dep-libzmq "libzmq (ZeroMQ)" </td>
	<td align="center" > Optional (*) (Network transport layer) </td>
</tr>
<tr>
	<td align="center" > \ref  dep-pcl "pcl" </td>
	<td align="center" > Optional (*) </td>
</tr>
<tr>
	<td align="center" > \ref  dep-liblas "LAS (liblas)" </td>
	<td align="center" > Optional </td>
</tr>
<tr>
	<td align="center" > \ref dep-suitesparse "suitesparse" </td>
	<td align="center" > Optional (but partly embedded) </td>
</tr>
</table>
</center>

<b>Note:</b> (*) means that the functionality is header-only. This means that
will be available even if MRPT is built without those libraries, but the user
program includes both, the external library, and MRPT headers.

*/
