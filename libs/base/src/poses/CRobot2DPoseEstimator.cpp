/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include "base-precomp.h"  // Precompiled headers


#include <mrpt/poses/CPose2D.h>
#include <mrpt/poses/CPose3D.h>
#include <mrpt/poses/CRobot2DPoseEstimator.h>
#include <mrpt/math/wrap2pi.h>

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

	m_robot_vel_local = TTwist2D(.0, .0, .0);
}

/** Updates the filter so the pose is tracked to the current time */
void CRobot2DPoseEstimator::processUpdateNewPoseLocalization(
	const TPose2D &newPose,
	TTimeStamp cur_tim)
{
	CCriticalSectionLocker   lock(&m_cs);

	// Overwrite old localization data:
	m_last_loc_time	= cur_tim;
	m_last_loc		= newPose;

	// And create interpolated odometry data to work as "reference pose":
	if (m_last_odo_time!=INVALID_TIMESTAMP)
	{
		const double dT = timeDifference(m_last_odo_time,cur_tim);
		extrapolateRobotPose(m_last_odo,m_robot_vel_local,dT,m_loc_odo_ref);
	}
}

/** Updates the filter so the pose is tracked to the current time */
void CRobot2DPoseEstimator::processUpdateNewOdometry(
	const TPose2D &newGlobalOdometry,
	TTimeStamp cur_tim,
	bool hasVelocities,
	const mrpt::math::TTwist2D &velLocal
	)
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
		m_robot_vel_local = velLocal;
	}
	else
	{ // Note: JLBC 23/Nov/2016: I have removed an estimation of velocity from increments of odometry
		// because it was really bad. Just don't make up things: if the user doesn't provide us velocities,
		// we don't use velocities.
		m_robot_vel_local = TTwist2D(.0, .0, .0);
	}

	// And now times & odo:
	m_last_odo_time = cur_tim;
	m_last_odo		= newGlobalOdometry;

	MRPT_END
}

bool CRobot2DPoseEstimator::getCurrentEstimate(mrpt::math::TPose2D &pose, mrpt::math::TTwist2D &velLocal, mrpt::math::TTwist2D &velGlobal, mrpt::system::TTimeStamp tim_query) const
{
	if (m_last_odo_time == INVALID_TIMESTAMP || m_last_loc_time == INVALID_TIMESTAMP)
		return false;

	const double dTimeLoc = timeDifference(m_last_loc_time, tim_query);
	if (dTimeLoc>params.max_localiz_age)
		return false;

	//  Overall estimate:
	// last_loc (+) [ last_odo (-) odo_ref ] (+) extrapolation_from_vw
	const TPose2D p = TPose2D(CPose2D(m_last_loc) + (CPose2D(m_last_odo) - CPose2D(m_loc_odo_ref)));

	// Add the extrapolation:
	const double dTimeOdo = timeDifference(m_last_odo_time, tim_query);
	if (dTimeOdo >params.max_odometry_age)
		return false;

	extrapolateRobotPose(p, m_robot_vel_local,dTimeOdo,pose);

	// Constant speed model:
	velLocal = m_robot_vel_local;
	velGlobal = velLocal;
	velGlobal.rotate(pose.phi);

	return true;
}

bool CRobot2DPoseEstimator::getCurrentEstimate(
	TPose2D &pose, float &v, float &w,
	TTimeStamp tim_query
	) const
{
	mrpt::math::TTwist2D velLocal, velGlobal;
	bool ret = getCurrentEstimate(pose, velLocal, velGlobal, tim_query);
	v = velLocal.vx;
	w = velLocal.omega;
	return ret;
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
	const mrpt::math::TTwist2D &velLocal,
	const double delta_time,
	TPose2D &new_p)
{
	if (velLocal.vx==0 && velLocal.vy ==0 && velLocal.omega == 0)
	{	// Still
		new_p = p;
	}
	else
	if (std::abs(velLocal.vy)>1e-2)
	{	// non-Ackermann-like vehicle: extrapolate as a straight line:
		const TPoint2D dp=TPoint2D(delta_time*velLocal.vx, delta_time*velLocal.vy);
		TPoint2D pg;
		CPose2D(p).composePoint(dp, pg);
		new_p.x = pg.x;
		new_p.y = pg.y;
		new_p.phi = p.phi + delta_time * velLocal.omega;
	}
	else
	{
		// vy==0, assume we are in a Ackermann-like steering vehicle: compute an arc:
		const double R = velLocal.vx / velLocal.omega; // Radius
		const double theta = velLocal.omega*delta_time; // traversed arc
		const double cc = cos(p.phi);
		const double ss = sin(p.phi);

		const double arc_x = R*sin(theta);
		const double arc_y = R*(1-cos(theta));

		new_p.x = p.x + cc*arc_x - ss*arc_y;
		new_p.y = p.y + ss*arc_x + cc*arc_y;
		new_p.phi = p.phi + theta;
	}
}

bool CRobot2DPoseEstimator::getCurrentEstimate( mrpt::poses::CPose2D &pose, float &v, float &w, mrpt::system::TTimeStamp tim_query) const
{
	mrpt::math::TPose2D  p;
	mrpt::math::TTwist2D velLocal, velGlobal;
	bool ret = getCurrentEstimate(p, velLocal, velGlobal, tim_query);
	if (ret)
		pose = CPose2D(p);
	v = velLocal.vx;
	w = velLocal.omega;
	return ret;
}

bool CRobot2DPoseEstimator::getLatestRobotPose(CPose2D &pose) const
{
	mrpt::math::TPose2D p;
	bool v = getLatestRobotPose(p);
	if (v)
	{
		pose.x(p.x);
		pose.y(p.y);
		pose.phi(p.phi);
	}
	return v;
}
