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




