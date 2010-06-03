/* +---------------------------------------------------------------------------+
   |          The Mobile Robot Programming Toolkit (MRPT) C++ library          |
   |                                                                           |
   |                   http://mrpt.sourceforge.net/                            |
   |                                                                           |
   |   Copyright (C) 2005-2010  University of Malaga                           |
   |                                                                           |
   |    This software was written by the Machine Perception and Intelligent    |
   |      Robotics Lab, University of Malaga (Spain).                          |
   |    Contact: Jose-Luis Blanco  <jlblanco@ctima.uma.es>                     |
   |                                                                           |
   |  This file is part of the MRPT project.                                   |
   |                                                                           |
   |     MRPT is free software: you can redistribute it and/or modify          |
   |     it under the terms of the GNU General Public License as published by  |
   |     the Free Software Foundation, either version 3 of the License, or     |
   |     (at your option) any later version.                                   |
   |                                                                           |
   |   MRPT is distributed in the hope that it will be useful,                 |
   |     but WITHOUT ANY WARRANTY; without even the implied warranty of        |
   |     MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         |
   |     GNU General Public License for more details.                          |
   |                                                                           |
   |     You should have received a copy of the GNU General Public License     |
   |     along with MRPT.  If not, see <http://www.gnu.org/licenses/>.         |
   |                                                                           |
   +---------------------------------------------------------------------------+ */
#ifndef CPOSEORPOINT_H
#define CPOSEORPOINT_H

#include <mrpt/utils/CSerializable.h>
#include <mrpt/math/CMatrixFixedNumeric.h>
#include <mrpt/math/lightweight_geom_data.h>
#include <mrpt/math/ops_matrices.h>  // Added here so many classes have access to these templates

namespace mrpt
{
	/** Classes for 2D/3D geometry representation, both of single values and probability density distributions (PDFs) in many forms.
	  */
	namespace poses
	{
		using namespace mrpt::utils;  // For square
		using namespace mrpt::math;  // For ligh. geom data

		// For use in some constructors (eg. CPose3D)
		#define UNINITIALIZED_POSE  false,false

		class CPoint2D;
		class CPoint3D;
		class CPose2D;
		class CPose3D;

		// This must be added to any CSerializable derived class:
		DEFINE_SERIALIZABLE_PRE_CUSTOM_BASE( CPoseOrPoint, mrpt::utils::CSerializable )

		/** The base class for 2D points, 3D points, 2D poses and 3D poses.
		 *   The common part defined in this base is the N-dimensional (N=2,3)
		 *    <b>position</b> vector. The existence of orientation angles is
		 *    left to derived classes.
		 *  In this class euclidean distance methods and operators are implemented.
		 *
		 *  For more information and examples, refer
		 *    to the <a href="http://www.mrpt.org/2D_3D_Geometry">2D/3D Geometry tutorial</a> in the wiki.
		 *

		  <center><h2>Introduction to 2D and 3D representation classes</h2></center>
		  <hr>
		  <p>
		  There are two class of spatial representation classes:
			- Point: A point in the common mathematical sense, with no directional information.
				- 2D: A 2D point is represented just by its coordinates (x,y).
				- 3D: A 3D point is represented by its coordinates (x,y,z).
			- Pose: It is a point, plus a direction.
				- 2D: A 2D pose is a 2D point plus a single angle, the yaw or &#966; angle: the angle from the positive X angle.
				- 3D: A 3D point is a 3D point plus three orientation angles (More details above).
		  </p>
			In the case for a 3D orientation many representation angles can be used (Euler angles,yaw/pitch/roll,...)
			but all of them can be handled by a 4x4 matrix called "Homogeneous Matrix". This matrix includes both, the
			translation and the orientation for a point or a pose, and it can be obtained using
			the method getHomogeneousMatrix() which is defined for any pose or point. Note that when the YPR angles are
			 used to define a 3D orientation, these three values can not be extracted from the matrix again.<br><br>

			<b>Operators:</b> There are operators defined for the pose compounding (+) and inverse pose
			 compounding (-) of poses and points. For example, let "a" and "b" be 2D or 3D poses. Then "a+b"
			 returns the resulting pose of "moving b" from "a"; and "b-a" returns the pose of "b" as it is seen
			 "from a". They can be mixed points and poses, being 2D or 3D, in these operators, with the following
			 results: <br>
			 <center>
			 Does "a+b"returns a Pose or a Point?
			 <table>
				<tr>
					<td><b>a  \  b</b></td>
					<td><b>Pose</b></td>
					<td><b>Point</b></td>
				</tr>
				<tr>
					<td><b>Pose</b></td>
					<td>Pose</td>
					<td>Point</td>
				</tr>
				<tr>
					<td><b>Point</b></td>
					<td>Pose</td>
					<td>Point</td>
				</tr>
			</table>
			</center>
			<br>
			 <center>
			 Does "a-b"returns a Pose or a Point?
			 <table>
				<tr>
					<td><b>a  \  b</b></td>
					<td><b>Pose</b></td>
					<td><b>Point</b></td>
				</tr>
				<tr>
					<td><b>Pose</b></td>
					<td>Pose</td>
					<td>Pose</td>
				</tr>
				<tr>
					<td><b>Point</b></td>
					<td>Point</td>
					<td>Point</td>
				</tr>
			</table>
			</center>
			<br>
			 <center>
			 Does "a+b"(and "a-b") returns a 2D or a 3D object?
			 <table>
				<tr>
					<td><b>a  \  b</b></td>
					<td><b>2D</b></td>
					<td><b>3D</b></td>
				</tr>
				<tr>
					<td><b>2D</b></td>
					<td>2D</td>
					<td>3D</td>
				</tr>
				<tr>
					<td><b>3D</b></td>
					<td>3D</td>
					<td>3D</td>
				</tr>
			</table>
			</center>
		 <br><br>
			<b>Homogeneous matrices:</b> The matrices computation follows the equations that can be found in
				any introductory text to spatial orientations, which are exposed next:<br>

	<div align=center>

	<table class=MsoTableGrid border=1 cellspacing=0 cellpadding=0
	 style='border-collapse:collapse;border:none'>
	 <tr>
	  <td width=576 colspan=2 style='width:432.2pt;border:solid windowtext 1.0pt;
	  background:#E6E6E6;padding:0cm 5.4pt 0cm 5.4pt'>
	  <p   align=center style='text-align:center'>poses::CPoint2D</p>
	  </td>
	 </tr>
	 <tr>
	  <td width=288 style='width:216.1pt;border:solid windowtext 1.0pt;border-top:
	  none;padding:0cm 5.4pt 0cm 5.4pt'>
	  <p   align=center style='text-align:center'>Homogeneous
	  transfomation matrix</p>
	  </td>
	  <td width=288 style='width:216.1pt;border-top:none;border-left:none;
	  border-bottom:solid windowtext 1.0pt;border-right:solid windowtext 1.0pt;
	  padding:0cm 5.4pt 0cm 5.4pt'>
	  <p   align=center style='text-align:center'>Spatial
	  representation</p>
	  </td>
	 </tr>
	 <tr style='height:108.3pt'>
	  <td width=288 style='width:216.1pt;border:solid windowtext 1.0pt;border-top:
	  none;padding:0cm 5.4pt 0cm 5.4pt;height:108.3pt'>
	  <div align=center>
	  <table  Table border=0 cellspacing=0 cellpadding=0 width="46%"
	   style='width:46.84%;border-collapse:collapse'>
	   <tr style='height:16.5pt'>
		<td width=32 style='width:24.0pt;padding:0cm 5.4pt 0cm 5.4pt;height:16.5pt'>
		<p   align=center style='text-align:center'>1</p>
		</td>
		<td width=32 style='width:24.05pt;padding:0cm 5.4pt 0cm 5.4pt;height:16.5pt'>
		<p   align=center style='text-align:center'>0</p>
		</td>
		<td width=32 style='width:24.05pt;padding:0cm 5.4pt 0cm 5.4pt;height:16.5pt'>
		<p   align=center style='text-align:center'>0</p>
		</td>
		<td width=32 style='width:24.05pt;padding:0cm 5.4pt 0cm 5.4pt;height:16.5pt'>
		<p   align=center style='text-align:center'>x</p>
		</td>
	   </tr>
	   <tr style='height:16.5pt'>
		<td width=32 style='width:24.0pt;padding:0cm 5.4pt 0cm 5.4pt;height:16.5pt'>
		<p   align=center style='text-align:center'>0</p>
		</td>
		<td width=32 style='width:24.05pt;padding:0cm 5.4pt 0cm 5.4pt;height:16.5pt'>
		<p   align=center style='text-align:center'>1</p>
		</td>
		<td width=32 style='width:24.05pt;padding:0cm 5.4pt 0cm 5.4pt;height:16.5pt'>
		<p   align=center style='text-align:center'>0</p>
		</td>
		<td width=32 style='width:24.05pt;padding:0cm 5.4pt 0cm 5.4pt;height:16.5pt'>
		<p   align=center style='text-align:center'>y</p>
		</td>
	   </tr>
	   <tr style='height:16.5pt'>
		<td width=32 style='width:24.0pt;padding:0cm 5.4pt 0cm 5.4pt;height:16.5pt'>
		<p   align=center style='text-align:center'>0</p>
		</td>
		<td width=32 style='width:24.05pt;padding:0cm 5.4pt 0cm 5.4pt;height:16.5pt'>
		<p   align=center style='text-align:center'>0</p>
		</td>
		<td width=32 style='width:24.05pt;padding:0cm 5.4pt 0cm 5.4pt;height:16.5pt'>
		<p   align=center style='text-align:center'>1</p>
		</td>
		<td width=32 style='width:24.05pt;padding:0cm 5.4pt 0cm 5.4pt;height:16.5pt'>
		<p   align=center style='text-align:center'>0</p>
		</td>
	   </tr>
	   <tr style='height:16.5pt'>
		<td width=32 style='width:24.0pt;padding:0cm 5.4pt 0cm 5.4pt;height:16.5pt'>
		<p   align=center style='text-align:center'>0</p>
		</td>
		<td width=32 style='width:24.05pt;padding:0cm 5.4pt 0cm 5.4pt;height:16.5pt'>
		<p   align=center style='text-align:center'>0</p>
		</td>
		<td width=32 style='width:24.05pt;padding:0cm 5.4pt 0cm 5.4pt;height:16.5pt'>
		<p   align=center style='text-align:center'>0</p>
		</td>
		<td width=32 style='width:24.05pt;padding:0cm 5.4pt 0cm 5.4pt;height:16.5pt'>
		<p   align=center style='text-align:center'>1</p>
		</td>
	   </tr>
	  </table>
	  </div>
	  <p   align=center style='text-align:center'></p>
	  </td>
	  <td width=288 style='width:216.1pt;border-top:none;border-left:none;
	  border-bottom:solid windowtext 1.0pt;border-right:solid windowtext 1.0pt;
	  padding:0cm 5.4pt 0cm 5.4pt;height:108.3pt'>
	  <p   align=center style='text-align:center'><img src="CPoint2D.gif"></p>
	  </td>
	 </tr>
	</table>

	</div>

	<p></p>

	<div align=center>

	<table class=MsoTableGrid border=1 cellspacing=0 cellpadding=0
	 style='border-collapse:collapse;border:none'>
	 <tr>
	  <td width=576 colspan=2 style='width:432.2pt;border:solid windowtext 1.0pt;
	  background:#E6E6E6;padding:0cm 5.4pt 0cm 5.4pt'>
	  <p   align=center style='text-align:center'>poses::CPoint3D</p>
	  </td>
	 </tr>
	 <tr>
	  <td width=288 style='width:216.1pt;border:solid windowtext 1.0pt;border-top:
	  none;padding:0cm 5.4pt 0cm 5.4pt'>
	  <p   align=center style='text-align:center'>Homogeneous
	  transfomation matrix</p>
	  </td>
	  <td width=288 style='width:216.1pt;border-top:none;border-left:none;
	  border-bottom:solid windowtext 1.0pt;border-right:solid windowtext 1.0pt;
	  padding:0cm 5.4pt 0cm 5.4pt'>
	  <p   align=center style='text-align:center'>Spatial
	  representation</p>
	  </td>
	 </tr>
	 <tr style='height:108.3pt'>
	  <td width=288 style='width:216.1pt;border:solid windowtext 1.0pt;border-top:
	  none;padding:0cm 5.4pt 0cm 5.4pt;height:108.3pt'>
	  <div align=center>
	  <table  Table border=0 cellspacing=0 cellpadding=0 width="46%"
	   style='width:46.84%;border-collapse:collapse'>
	   <tr style='height:16.5pt'>
		<td width=32 style='width:24.0pt;padding:0cm 5.4pt 0cm 5.4pt;height:16.5pt'>
		<p   align=center style='text-align:center'>1</p>
		</td>
		<td width=32 style='width:24.05pt;padding:0cm 5.4pt 0cm 5.4pt;height:16.5pt'>
		<p   align=center style='text-align:center'>0</p>
		</td>
		<td width=32 style='width:24.05pt;padding:0cm 5.4pt 0cm 5.4pt;height:16.5pt'>
		<p   align=center style='text-align:center'>0</p>
		</td>
		<td width=32 style='width:24.05pt;padding:0cm 5.4pt 0cm 5.4pt;height:16.5pt'>
		<p   align=center style='text-align:center'>x</p>
		</td>
	   </tr>
	   <tr style='height:16.5pt'>
		<td width=32 style='width:24.0pt;padding:0cm 5.4pt 0cm 5.4pt;height:16.5pt'>
		<p   align=center style='text-align:center'>0</p>
		</td>
		<td width=32 style='width:24.05pt;padding:0cm 5.4pt 0cm 5.4pt;height:16.5pt'>
		<p   align=center style='text-align:center'>1</p>
		</td>
		<td width=32 style='width:24.05pt;padding:0cm 5.4pt 0cm 5.4pt;height:16.5pt'>
		<p   align=center style='text-align:center'>0</p>
		</td>
		<td width=32 style='width:24.05pt;padding:0cm 5.4pt 0cm 5.4pt;height:16.5pt'>
		<p   align=center style='text-align:center'>y</p>
		</td>
	   </tr>
	   <tr style='height:16.5pt'>
		<td width=32 style='width:24.0pt;padding:0cm 5.4pt 0cm 5.4pt;height:16.5pt'>
		<p   align=center style='text-align:center'>0</p>
		</td>
		<td width=32 style='width:24.05pt;padding:0cm 5.4pt 0cm 5.4pt;height:16.5pt'>
		<p   align=center style='text-align:center'>0</p>
		</td>
		<td width=32 style='width:24.05pt;padding:0cm 5.4pt 0cm 5.4pt;height:16.5pt'>
		<p   align=center style='text-align:center'>1</p>
		</td>
		<td width=32 style='width:24.05pt;padding:0cm 5.4pt 0cm 5.4pt;height:16.5pt'>
		<p   align=center style='text-align:center'>z</p>
		</td>
	   </tr>
	   <tr style='height:16.5pt'>
		<td width=32 style='width:24.0pt;padding:0cm 5.4pt 0cm 5.4pt;height:16.5pt'>
		<p   align=center style='text-align:center'>0</p>
		</td>
		<td width=32 style='width:24.05pt;padding:0cm 5.4pt 0cm 5.4pt;height:16.5pt'>
		<p   align=center style='text-align:center'>0</p>
		</td>
		<td width=32 style='width:24.05pt;padding:0cm 5.4pt 0cm 5.4pt;height:16.5pt'>
		<p   align=center style='text-align:center'>0</p>
		</td>
		<td width=32 style='width:24.05pt;padding:0cm 5.4pt 0cm 5.4pt;height:16.5pt'>
		<p   align=center style='text-align:center'>1</p>
		</td>
	   </tr>
	  </table>
	  </div>
	  <p   align=center style='text-align:center'></p>
	  </td>
	  <td width=288 style='width:216.1pt;border-top:none;border-left:none;
	  border-bottom:solid windowtext 1.0pt;border-right:solid windowtext 1.0pt;
	  padding:0cm 5.4pt 0cm 5.4pt;height:108.3pt'>
	  <p   align=center style='text-align:center'><img src="CPoint3D.gif"></p>
	  </td>
	 </tr>
	</table>

	</div>

	<p></p>

	<div align=center>

	<table class=MsoTableGrid border=1 cellspacing=0 cellpadding=0
	 style='border-collapse:collapse;border:none'>
	 <tr>
	  <td width=576 colspan=2 style='width:432.2pt;border:solid windowtext 1.0pt;
	  background:#E6E6E6;padding:0cm 5.4pt 0cm 5.4pt'>
	  <p   align=center style='text-align:center'>poses::CPose2D</p>
	  </td>
	 </tr>
	 <tr>
	  <td width=288 style='width:216.1pt;border:solid windowtext 1.0pt;border-top:
	  none;padding:0cm 5.4pt 0cm 5.4pt'>
	  <p   align=center style='text-align:center'>Homogeneous
	  transfomation matrix</p>
	  </td>
	  <td width=288 style='width:216.1pt;border-top:none;border-left:none;
	  border-bottom:solid windowtext 1.0pt;border-right:solid windowtext 1.0pt;
	  padding:0cm 5.4pt 0cm 5.4pt'>
	  <p   align=center style='text-align:center'>Spatial
	  representation</p>
	  </td>
	 </tr>
	 <tr style='height:108.3pt'>
	  <td width=288 style='width:216.1pt;border:solid windowtext 1.0pt;border-top:
	  none;padding:0cm 5.4pt 0cm 5.4pt;height:108.3pt'>
	  <div align=center>
	  <table  Table border=0 cellspacing=0 cellpadding=0 width="67%"
	   style='width:67.92%;border-collapse:collapse'>
	   <tr style='height:20.6pt'>
		<td width=46 style='width:34.85pt;padding:0cm 5.4pt 0cm 5.4pt;height:20.6pt'>
		<p   align=center style='text-align:center'>cos<span
		style='font-family:Symbol'>j</span></p>
		</td>
		<td width=46 style='width:34.85pt;padding:0cm 5.4pt 0cm 5.4pt;height:20.6pt'>
		<p   align=center style='text-align:center'>-sin<span
		style='font-family:Symbol'>j</span></p>
		</td>
		<td width=46 style='width:34.85pt;padding:0cm 5.4pt 0cm 5.4pt;height:20.6pt'>
		<p   align=center style='text-align:center'>0</p>
		</td>
		<td width=47 style='width:34.9pt;padding:0cm 5.4pt 0cm 5.4pt;height:20.6pt'>
		<p   align=center style='text-align:center'>x</p>
		</td>
	   </tr>
	   <tr style='height:20.6pt'>
		<td width=46 style='width:34.85pt;padding:0cm 5.4pt 0cm 5.4pt;height:20.6pt'>
		<p   align=center style='text-align:center'>sin<span
		style='font-family:Symbol'>j</span></p>
		</td>
		<td width=46 style='width:34.85pt;padding:0cm 5.4pt 0cm 5.4pt;height:20.6pt'>
		<p   align=center style='text-align:center'>cos<span
		style='font-family:Symbol'>j</span></p>
		</td>
		<td width=46 style='width:34.85pt;padding:0cm 5.4pt 0cm 5.4pt;height:20.6pt'>
		<p   align=center style='text-align:center'>0</p>
		</td>
		<td width=47 style='width:34.9pt;padding:0cm 5.4pt 0cm 5.4pt;height:20.6pt'>
		<p   align=center style='text-align:center'>y</p>
		</td>
	   </tr>
	   <tr style='height:20.6pt'>
		<td width=46 style='width:34.85pt;padding:0cm 5.4pt 0cm 5.4pt;height:20.6pt'>
		<p   align=center style='text-align:center'>0</p>
		</td>
		<td width=46 style='width:34.85pt;padding:0cm 5.4pt 0cm 5.4pt;height:20.6pt'>
		<p   align=center style='text-align:center'>0</p>
		</td>
		<td width=46 style='width:34.85pt;padding:0cm 5.4pt 0cm 5.4pt;height:20.6pt'>
		<p   align=center style='text-align:center'>1</p>
		</td>
		<td width=47 style='width:34.9pt;padding:0cm 5.4pt 0cm 5.4pt;height:20.6pt'>
		<p   align=center style='text-align:center'>0</p>
		</td>
	   </tr>
	   <tr style='height:20.6pt'>
		<td width=46 style='width:34.85pt;padding:0cm 5.4pt 0cm 5.4pt;height:20.6pt'>
		<p   align=center style='text-align:center'>0</p>
		</td>
		<td width=46 style='width:34.85pt;padding:0cm 5.4pt 0cm 5.4pt;height:20.6pt'>
		<p   align=center style='text-align:center'>0</p>
		</td>
		<td width=46 style='width:34.85pt;padding:0cm 5.4pt 0cm 5.4pt;height:20.6pt'>
		<p   align=center style='text-align:center'>0</p>
		</td>
		<td width=47 style='width:34.9pt;padding:0cm 5.4pt 0cm 5.4pt;height:20.6pt'>
		<p   align=center style='text-align:center'>1</p>
		</td>
	   </tr>
	  </table>
	  </div>
	  <p   align=center style='text-align:center'></p>
	  </td>
	  <td width=288 style='width:216.1pt;border-top:none;border-left:none;
	  border-bottom:solid windowtext 1.0pt;border-right:solid windowtext 1.0pt;
	  padding:0cm 5.4pt 0cm 5.4pt;height:108.3pt'>
	  <p   align=center style='text-align:center'><img src="CPose2D.gif"></p>
	  </td>
	 </tr>
	</table>

	</div>

	<p></p>

	<div align=center>

	<table class=MsoTableGrid border=1 cellspacing=0 cellpadding=0
	 style='border-collapse:collapse;border:none'>
	 <tr style='height:15.8pt'>
	  <td width=676 colspan=2 style='width:507.25pt;border:solid windowtext 1.0pt;
	  background:#E6E6E6;padding:0cm 5.4pt 0cm 5.4pt;height:15.8pt'>
	  <p   align=center style='text-align:center'>poses::CPose3D</p>
	  </td>
	 </tr>
	 <tr style='height:15.8pt'>
	  <td width=350 style='width:262.65pt;border:solid windowtext 1.0pt;border-top:
	  none;padding:0cm 5.4pt 0cm 5.4pt;height:15.8pt'>
	  <p   align=center style='text-align:center'>Homogeneous
	  transfomation matrix</p>
	  </td>
	  <td width=326 style='width:244.6pt;border-top:none;border-left:none;
	  border-bottom:solid windowtext 1.0pt;border-right:solid windowtext 1.0pt;
	  padding:0cm 5.4pt 0cm 5.4pt;height:15.8pt'>
	  <p   align=center style='text-align:center'>Spatial
	  representation</p>
	  </td>
	 </tr>
	 <tr style='height:202.65pt'>
	  <td width=350 style='width:262.65pt;border:solid windowtext 1.0pt;border-top:
	  none;padding:0cm 5.4pt 0cm 5.4pt;height:202.65pt'>
	  <div align=center>
	  <table  Table border=0 cellspacing=0 cellpadding=0 width=334
	   style='width:250.65pt;border-collapse:collapse'>
	   <tr style='height:16.65pt'>
		<td width=66 style='width:49.65pt;padding:0cm 5.4pt 0cm 5.4pt;height:16.65pt'>
		<p   align=center style='text-align:center'>cycp</p>
		</td>
		<td width=99 style='width:74.15pt;padding:0cm 5.4pt 0cm 5.4pt;height:16.65pt'>
		<p   align=center style='text-align:center'>cyspsr-sycr</p>
		</td>
		<td width=87 style='width:65.55pt;padding:0cm 5.4pt 0cm 5.4pt;height:16.65pt'>
		<p   align=center style='text-align:center'>cyspcr+sysr</p>
		</td>
		<td width=82 style='width:61.3pt;padding:0cm 5.4pt 0cm 5.4pt;height:16.65pt'>
		<p   align=center style='text-align:center'>x</p>
		</td>
	   </tr>
	   <tr style='height:17.25pt'>
		<td width=66 style='width:49.65pt;padding:0cm 5.4pt 0cm 5.4pt;height:17.25pt'>
		<p   align=center style='text-align:center'>sycp</p>
		</td>
		<td width=99 style='width:74.15pt;padding:0cm 5.4pt 0cm 5.4pt;height:17.25pt'>
		<p   align=center style='text-align:center'>syspsr+cycr</p>
		</td>
		<td width=87 style='width:65.55pt;padding:0cm 5.4pt 0cm 5.4pt;height:17.25pt'>
		<p   align=center style='text-align:center'>syspcr-cysr</p>
		</td>
		<td width=82 style='width:61.3pt;padding:0cm 5.4pt 0cm 5.4pt;height:17.25pt'>
		<p   align=center style='text-align:center'>y</p>
		</td>
	   </tr>
	   <tr style='height:19.65pt'>
		<td width=66 style='width:49.65pt;padding:0cm 5.4pt 0cm 5.4pt;height:19.65pt'>
		<p   align=center style='text-align:center'>-sp</p>
		</td>
		<td width=99 style='width:74.15pt;padding:0cm 5.4pt 0cm 5.4pt;height:19.65pt'>
		<p   align=center style='text-align:center'>cpsr</p>
		</td>
		<td width=87 style='width:65.55pt;padding:0cm 5.4pt 0cm 5.4pt;height:19.65pt'>
		<p   align=center style='text-align:center'>cpcr</p>
		</td>
		<td width=82 style='width:61.3pt;padding:0cm 5.4pt 0cm 5.4pt;height:19.65pt'>
		<p   align=center style='text-align:center'>z</p>
		</td>
	   </tr>
	   <tr style='height:11.0pt'>
		<td width=66 style='width:49.65pt;padding:0cm 5.4pt 0cm 5.4pt;height:11.0pt'>
		<p   align=center style='text-align:center'>0</p>
		</td>
		<td width=99 style='width:74.15pt;padding:0cm 5.4pt 0cm 5.4pt;height:11.0pt'>
		<p   align=center style='text-align:center'>0</p>
		</td>
		<td width=87 style='width:65.55pt;padding:0cm 5.4pt 0cm 5.4pt;height:11.0pt'>
		<p   align=center style='text-align:center'>0</p>
		</td>
		<td width=82 style='width:61.3pt;padding:0cm 5.4pt 0cm 5.4pt;height:11.0pt'>
		<p   align=center style='text-align:center'>1</p>
		</td>
	   </tr>
	  </table>
	  </div>
	  <p   align=center style='text-align:center'><span lang=EN-GB>where:</span></p>
	  <p   align=center style='text-align:center'><span lang=EN-GB>cy
	  = cos Yaw ;  sy = sin Yaw</span></p>
	  <p   align=center style='text-align:center'><span lang=EN-GB>cp
	  = cos Pitch ; sp = sin Pitch</span></p>
	  <p   align=center style='text-align:center'><span lang=EN-GB>cr
	  = cos Roll ; sr = sin Roll</span></p>
	  </td>
	  <td width=326 style='width:244.6pt;border-top:none;border-left:none;
	  border-bottom:solid windowtext 1.0pt;border-right:solid windowtext 1.0pt;
	  padding:0cm 5.4pt 0cm 5.4pt;height:202.65pt'>
	  <p   align=center style='text-align:center'><span lang=EN-GB><img  src="CPose3D.gif"></span></p>
	  </td>
	 </tr>
	</table>

	</div>

		 * \sa CPose,CPoint
		 */
		class BASE_IMPEXP CPoseOrPoint : public mrpt::utils::CSerializable
		{
			// This must be added to any CSerializable derived class:
			DEFINE_VIRTUAL_SERIALIZABLE( CPoseOrPoint )

		protected:
			bool	m_is3D;	//!< Will be false for 2D points or poses, and true for 3D points or poses.
			double	m_x, m_y, m_z;		//!< The x y and z coordinates of the point or pose.

		public:
			 CPoseOrPoint() : m_is3D(), m_x(),m_y(),m_z()
			 {  }

			 inline double x() const { return m_x; }  //!< Get the X coordinate
			 inline double y() const { return m_y; }  //!< Get the Y coordinate
			 inline double z() const { return m_z; }  //!< Get the Z coordinate

			 inline double &x() { return m_x; }  //!< Get a reference to the X coordinate
			 inline double &y() { return m_y; }  //!< Get a reference to the Y coordinate
			 inline double &z() { return m_z; }  //!< Get a reference to the Z coordinate

			 virtual void x(const double x_) { m_x=x_; }  //!< Set the X coordinate
			 virtual void y(const double y_) { m_y=y_; }  //!< Set the Y coordinate
			 virtual void z(const double z_) { m_z=z_; }  //!< Set the Z coordinate

			 virtual void x_incr(const double Ax) { m_x+=Ax; }  //!< Increment the X coordinate
			 virtual void y_incr(const double Ay) { m_y+=Ay; }  //!< Increment the Y coordinate
			 virtual void z_incr(const double Az) { m_z+=Az; }  //!< Increment the Z coordinate

			 /** Return true for poses or points with a Z component, false otherwise.
			   */
			 inline bool is3DPoseOrPoint() const { return m_is3D; }

			 /** Returns the euclidean distance to another pose/point:
			  */
			inline double distanceTo(const CPoseOrPoint &b) const
			{
				if (m_is3D || b.m_is3D)
						return sqrt(square(m_x-b.m_x) + square(m_y-b.m_y) + square(m_z-b.m_z));
				else	return sqrt(square(m_x-b.m_x) + square(m_y-b.m_y));
			}

			/** Returns the euclidean distance to another pose/point:
			  */
			inline double distanceTo(const mrpt::math::TPoint3D &b) const
			{
				if (m_is3D)
						return sqrt(square(m_x-b.x) + square(m_y-b.y) + square(m_z-b.z));
				else	return sqrt(square(m_x-b.x) + square(m_y-b.y));
			}

			 /** Returns the squared euclidean distance to another pose/point:
			  */
			inline double sqrDistanceTo(const CPoseOrPoint &b) const
			{
				if (m_is3D || b.m_is3D)
						return square(m_x-b.m_x) + square(m_y-b.m_y) + square(m_z-b.m_z);
				else	return square(m_x-b.m_x) + square(m_y-b.m_y);
			}

			 /** Returns the euclidean norm of vector: \f$ ||\mathbf{x}|| = \sqrt{x^2_1+y^2_i+...} \f$
			  */
			 double  norm() const;

			 /** Scalar multiplication.
			   */
			 virtual void operator *=(const double s) = 0;

			 /** Return the pose or point as a 1xN vector with all the components (see derived classes for each implementation) */
			 virtual void getAsVector(vector_double &v) const = 0;

			 /** Return the pose or point as a 1xN vector with all the components (see derived classes for each implementation) */
			 vector_double getAsVectorVal() const
			 {
				 vector_double v;
				 this->getAsVector(v);
				 return v;
			 }

			 /** Returns the corresponding 4x4 homogeneous transformation matrix for the point(translation) or pose (translation+orientation).
			   * \sa getInverseHomogeneousMatrix
			   */
			 CMatrixDouble44 getHomogeneousMatrixVal() const {
				 CMatrixDouble44 m(UNINITIALIZED_MATRIX);
				 getHomogeneousMatrix(m);
				 return m;
			 }

			 /** Returns the corresponding 4x4 homogeneous transformation matrix for the point(translation) or pose (translation+orientation).
			   * \sa getHomogeneousMatrixVal, getInverseHomogeneousMatrix
			   */
			 virtual  void  getHomogeneousMatrix(CMatrixDouble44 & out_HM ) const =0;

			 /** Returns a human-readable textual representation of the object (eg: "[0.02 1.04 -0.8]" )
			   * \sa fromString
			   */
			 virtual void asString(std::string &s) const = 0;

			 /** Set the current object value from a string generated by 'asString' (eg: "[0.02 1.04 -0.8]" )
			   * \sa asString
			   * \exception std::exception On invalid format
			   */
			 virtual void fromString(const std::string &s) = 0;


			 /** Returns the corresponding inverse homogeneous
			   *  transformation matrix for the point(translation),
			   *  or pose (translation+orientation).
			   * \sa getHomogeneousMatrix
			   */
			 void getInverseHomogeneousMatrix( math::CMatrixDouble44 &out_HM ) const;

			 /** Returns the 2D distance from this pose/point to another given by its coordinates:
			   * \sa distance2DToSquare,distance3DTo
			   */
			 inline double  distance2DTo( double ax, double ay ) const { return sqrt( square(ax-m_x)+square(ay-m_y) );	 }

			 /** Returns the 3D distance from this pose/point to another given by its coordinates:
			   * \sa distance3DToSquare,distance2DTo
			   */
			 inline double  distance3DTo( double ax, double ay, double az  ) const { return sqrt(  square(ax-m_x)+square(ay-m_y)+square(az-m_z)); }

			 /** Returns the square of the 2D distance from this pose/point to another given by its coordinates:
			   * \sa distance2DTo,distance3DToSquare
			   */
			 inline double  distance2DToSquare( double ax, double ay ) const { return  square(ax-m_x)+square(ay-m_y);	 }

			 /** Returns the square of the the 3D distance from this pose/point to another given by its coordinates:
			   * \sa distance3DTo,distance2DToSquare
			   */
			 inline double  distance3DToSquare( double ax, double ay, double az  ) const { return square(ax-m_x)+square(ay-m_y)+square(az-m_z);	 }

		}; // End of class def.


	} // End of namespace
} // End of namespace

#endif
