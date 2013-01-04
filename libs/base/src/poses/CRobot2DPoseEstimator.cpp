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

#include <mrpt/base.h>  // Precompiled headers


#include <mrpt/poses/CPose2D.h>
#include <mrpt/poses/CRobot2DPoseEstimator.h>

using namespace mrpt;
using namespace mrpt::poses;
using namespace mrpt::math;
using namespace mrpt::utils;
using namespace mrpt::synch;
using namespace mrpt::system;
using namespace std;


/* --------------------------------------------------------
				Ctor
   -------------------------------------------------------- */
CRobot2DPoseEstimator::CRobot2DPoseEstimator()
{
	reset();
}

/* --------------------------------------------------------
				Dtor
   -------------------------------------------------------- */
CRobot2DPoseEstimator::~CRobot2DPoseEstimator()
{
}

/* --------------------------------------------------------
				reset
   -------------------------------------------------------- */
void CRobot2DPoseEstimator::reset()
{
	CCriticalSectionLocker   lock(&m_cs);

	m_last_loc_time	= INVALID_TIMESTAMP;
	m_last_odo_time	= INVALID_TIMESTAMP;

	m_last_loc		= TPose2D(0,0,0);
	m_loc_odo_ref	= TPose2D(0,0,0);
	m_last_odo		= TPose2D(0,0,0);

	m_robot_v    = 0;
	m_robot_w    = 0;
}

/** Updates the filter so the pose is tracked to the current time */
void CRobot2DPoseEstimator::processUpdateNewPoseLocalization(
	const TPose2D &newPose,
	const CMatrixDouble33 &newPoseCov,
	TTimeStamp cur_tim)
{
	CCriticalSectionLocker   lock(&m_cs);

	// Overwrite old localization data:
	m_last_loc_time	= cur_tim;
	m_last_loc		= newPose;
	m_last_loc_cov  = newPoseCov;

	// And create interpolated odometry data to work as "reference pose":
	if (m_last_odo_time!=INVALID_TIMESTAMP)
	{
		const double dT = timeDifference(m_last_odo_time,cur_tim);
		extrapolateRobotPose(
			m_last_odo,m_robot_v,m_robot_w,
			dT,
			m_loc_odo_ref);
	}
}

/** Updates the filter so the pose is tracked to the current time */
void CRobot2DPoseEstimator::processUpdateNewOdometry(
	const TPose2D &newGlobalOdometry,
	TTimeStamp cur_tim,
	bool hasVelocities,
	float v,
	float w)
{
	MRPT_START

	CCriticalSectionLocker   lock(&m_cs);

	if (m_last_odo_time!=INVALID_TIMESTAMP)
	{
		const double dT = timeDifference(m_last_odo_time,cur_tim);
		if (dT<=0) std::cerr << "[CRobot2DPoseEstimator::processUpdateNewOdometry] WARNING: Diff. in timestamps between odometry should be >0, and it's " << dT << "\n";
	}

	// First, update velocities:
	if (hasVelocities)
	{
		m_robot_v = v;
		m_robot_w = w;
	}
	else
	{
		if (m_last_odo_time!=INVALID_TIMESTAMP)
		{
			// make something up!
			const double dT = timeDifference(m_last_odo_time,cur_tim);
			ASSERTMSG_(dT>0, "timestamp must be newer than the last one")

			m_robot_w = (newGlobalOdometry.phi-m_last_odo.phi)/dT;
			m_robot_v = ::hypot(newGlobalOdometry.x-m_last_odo.x, newGlobalOdometry.y-m_last_odo.y)/dT;
			// sign of v:
			if (fabs(atan2(newGlobalOdometry.y-m_last_odo.y,newGlobalOdometry.x-m_last_odo.x))>0.5*M_PI)
				m_robot_v *=-1;
		}
		else
		{	// Nothing we can do here...
			m_robot_v = 0;
			m_robot_w = 0;
		}
	}

	// And now times & odo:
	m_last_odo_time = cur_tim;
	m_last_odo		= newGlobalOdometry;

	MRPT_END
}

/** get the current estimate
* \return true is the estimate can be trusted. False if the real observed data is too old.
*/
bool CRobot2DPoseEstimator::getCurrentEstimate(
	TPose2D &pose, float &v, float &w,
	TTimeStamp tim_query
	) const
{
	if (m_last_odo_time==INVALID_TIMESTAMP || m_last_loc_time==INVALID_TIMESTAMP )
		return false;

	const double dTimeLoc = timeDifference(m_last_loc_time,tim_query);
	if (dTimeLoc>params.max_localiz_age)
		return false;

	// Constant speed model:
	v = m_robot_v;
	w = m_robot_w;

	//  Overall estimate:
	// last_loc (+) [ last_odo (-) odo_ref ] (+) extrapolation_from_vw
	const TPose2D p = TPose2D(  CPose2D(m_last_loc) + ( CPose2D(m_last_odo) - CPose2D(m_loc_odo_ref) ) );

	// Add the extrapolation:
	const double dTimeOdo = timeDifference(m_last_odo_time,tim_query);
	if (dTimeOdo >params.max_odometry_age)
		return false;

	extrapolateRobotPose(
		p,m_robot_v,m_robot_w,
		dTimeOdo,
		pose);

	return true;
}

/** get the current estimate
* \return true is the estimate can be trusted. False if the real observed data is too old.
*/
bool CRobot2DPoseEstimator::getLatestRobotPose(TPose2D &pose) const
{
	if (m_last_odo_time==INVALID_TIMESTAMP && m_last_loc_time==INVALID_TIMESTAMP )
		return false;

	bool ret_odo;
	if (m_last_odo_time!=INVALID_TIMESTAMP && m_last_loc_time!=INVALID_TIMESTAMP)
			ret_odo = (m_last_odo_time>m_last_loc_time);
	else if (m_last_odo_time!=INVALID_TIMESTAMP)
			ret_odo=true;
	else 	ret_odo=false;

	if (ret_odo)
		pose= CPose2D(m_last_loc) + ( CPose2D(m_last_odo) - CPose2D(m_loc_odo_ref) );
	else
		pose= m_last_loc;

	return true;
}


// An auxiliary method to extrapolate the pose of a robot located at "p"
//  with velocities (v,w) after a time delay "delta_time".
void CRobot2DPoseEstimator::extrapolateRobotPose(
	const TPose2D &p,
	const float v,
	const float w,
	const double delta_time,
	TPose2D &new_p)
{
	if (v==0 && w==0)
	{	// Still
		new_p = p;
	}
	else
	if (w==0)
	{	// Straight
		const double len = delta_time*v;
		new_p.x = p.x + cos(p.phi)*len;
		new_p.y = p.y + sin(p.phi)*len;
		new_p.phi = p.phi;
	}
	else
	{
		// generic arc:
		const double R = v/w; // Radius
		const double theta = w*delta_time; // traversed arc
		const double cc = cos(p.phi);
		const double ss = sin(p.phi);

		const double arc_x = R*sin(theta);
		const double arc_y = R*(1-cos(theta));

		new_p.x = p.x + cc*arc_x - ss*arc_y;
		new_p.y = p.y + ss*arc_x + cc*arc_y;
		new_p.phi = p.phi + theta;
	}
}




