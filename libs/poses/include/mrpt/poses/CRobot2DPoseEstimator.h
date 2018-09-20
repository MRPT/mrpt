/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */
#pragma once

#include <mrpt/math/lightweight_geom_data.h>
#include <mrpt/core/Clock.h>
#include <mrpt/poses/poses_frwds.h>
#include <mutex>
#include <optional>

namespace mrpt::poses
{
/** A simple filter to estimate and extrapolate the robot 2D (x,y,phi) pose from
 *asynchronous odometry and localization/SLAM data.
 *  The implemented model is a state vector:
 *		- TPose2D (x,y,phi) + TTwist2D (vx,vy,omega)
 *  The filter can be asked for an extrapolation for some arbitrary time `t'`,
 *and it'll do a simple linear prediction.
 *  **All methods are thread-safe**.
 * \ingroup poses_grp poses_pdf_grp
 */
class CRobot2DPoseEstimator
{
   public:
	/** Default constructor */
	CRobot2DPoseEstimator();
	/** Destructor */
	virtual ~CRobot2DPoseEstimator();
	/** Resets all internal state. */
	void reset();

	/** Updates the filter with new global-coordinates localization data from a
	 * localization or SLAM source.
	 * \param tim The timestamp of the sensor readings used to evaluate
	 * localization / SLAM.
	 */
	void processUpdateNewPoseLocalization(
		const mrpt::math::TPose2D& newPose, mrpt::Clock::time_point tim);

	/** Updates the filter with new odometry readings. */
	void processUpdateNewOdometry(
		const mrpt::math::TPose2D& newGlobalOdometry,
		mrpt::Clock::time_point cur_tim, bool hasVelocities = false,
		const mrpt::math::TTwist2D& newRobotVelLocal = mrpt::math::TTwist2D());

	/** Get the estimate for a given timestamp (defaults to `now()`), obtained
	 * as:
	 *
	 *   last_loc (+) [ last_odo (-) odo_ref ] (+) extrapolation_from_vw
	 *
	 * \return true is the estimate can be trusted. False if the real observed
	 * data is too old or there is no valid data yet.
	 * \sa getLatestRobotPose
	 */
	bool getCurrentEstimate(
		mrpt::math::TPose2D& pose, mrpt::math::TTwist2D& velLocal,
		mrpt::math::TTwist2D& velGlobal,
		mrpt::Clock::time_point tim_query = mrpt::Clock::now()) const;

	/** Get the latest known robot pose, either from odometry or localization.
	 *  This differs from getCurrentEstimate() in that this method does NOT
	 * extrapolate as getCurrentEstimate() does.
	 * \return false if there is not estimation yet.
	 * \sa getCurrentEstimate
	 */
	bool getLatestRobotPose(mrpt::math::TPose2D& pose) const;

	/** \overload */
	bool getLatestRobotPose(mrpt::poses::CPose2D& pose) const;

	struct TOptions
	{
		TOptions() = default;
		/** To consider data old, in seconds */
		double max_odometry_age{1.0};
		/** To consider data old, in seconds */
		double max_localiz_age{4.0};
	};

	/** parameters of the filter. */
	TOptions params;

   private:
	std::mutex m_cs;

	std::optional<mrpt::Clock::time_point> m_last_loc_time;
	/** Last pose as estimated by the localization/SLAM subsystem. */
	mrpt::math::TPose2D m_last_loc;

	/** The interpolated odometry position for the last "m_robot_pose" (used as
	 * "coordinates base" for subsequent odo readings) */
	mrpt::math::TPose2D m_loc_odo_ref;

	std::optional<mrpt::Clock::time_point> m_last_odo_time;
	mrpt::math::TPose2D m_last_odo;
	/** Robot odometry-based velocity in a local frame of reference. */
	mrpt::math::TTwist2D m_robot_vel_local;

	/** An auxiliary method to extrapolate the pose of a robot located at "p"
	 * with velocities (v,w) after a time delay "delta_time". */
	static void extrapolateRobotPose(
		const mrpt::math::TPose2D& p,
		const mrpt::math::TTwist2D& robot_vel_local, const double delta_time,
		mrpt::math::TPose2D& new_p);

};  // end of class

}  // namespace mrpt::poses
