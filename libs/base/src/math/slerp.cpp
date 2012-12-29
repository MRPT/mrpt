/* +---------------------------------------------------------------------------+
   |                 The Mobile Robot Programming Toolkit (MRPT)               |
   |                                                                           |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2013, Individual contributors, see AUTHORS file        |
   | Copyright (c) 2005-2013, MAPIR group, University of Malaga                |
   | Copyright (c) 2012-2013, University of Almeria                                 |
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
