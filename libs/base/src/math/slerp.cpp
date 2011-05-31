/* +---------------------------------------------------------------------------+
   |          The Mobile Robot Programming Toolkit (MRPT) C++ library          |
   |                                                                           |
   |                       http://www.mrpt.org/                                |
   |                                                                           |
   |   Copyright (C) 2005-2011  University of Malaga                           |
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

#include <mrpt/base.h>  // Precompiled headers

#include <mrpt/math/slerp.h>
#include <mrpt/poses.h>

using namespace mrpt;
using namespace mrpt::poses;
using namespace mrpt::math;


void mrpt::math::slerp(
	const CPose3D  & p0,
	const CPose3D  & p1,
	const double     t,
	CPose3D        & p)
{
	CQuaternionDouble q0,q1,q;
	p0.getAsQuaternion(q0);
	p1.getAsQuaternion(q1);
	// The quaternion part (this will raise exception on t not in [0,1])
	mrpt::math::slerp(q0,q1,t, q);
	// XYZ:
	p = CPose3D(
		q,
		(1-t)*p0.x()+t*p1.x(),
		(1-t)*p0.y()+t*p1.y(),
		(1-t)*p0.z()+t*p1.z() );
}

void mrpt::math::slerp(
	const CPose3DQuat & q0,
	const CPose3DQuat & q1,
	const double        t,
	CPose3DQuat       & q)
{
	// The quaternion part (this will raise exception on t not in [0,1])
	mrpt::math::slerp(q0.quat(), q1.quat(),t, q.quat());
	// XYZ:
	q.x( (1-t)*q0.x()+t*q1.x() );
	q.y( (1-t)*q0.y()+t*q1.y() );
	q.z( (1-t)*q0.z()+t*q1.z() );
}
