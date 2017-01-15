/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */
#ifndef CRobot2DPoseEstimator_H
#define CRobot2DPoseEstimator_H

#include <mrpt/synch/CCriticalSection.h>
#include <mrpt/math/lightweight_geom_data.h>
#include <mrpt/system/datetime.h>

namespace mrpt
{
	namespace poses
	{
		/** A simple filter to estimate and extrapolate the robot 2D (x,y,phi) pose from asynchronous odometry and localization/SLAM data.
		  *  The implemented model is a state vector:
		  *		- TPose2D (x,y,phi) + TTwist2D (vx,vy,omega)
		  *  The filter can be asked for an extrapolation for some arbitrary time `t'`, and it'll do a simple linear prediction.
		  *  **All methods are thread-safe**.
		  * \ingroup poses_grp poses_pdf_grp
		  */
		class BASE_IMPEXP CRobot2DPoseEstimator
		{
		public:
			CRobot2DPoseEstimator( ); //!< Default constructor
			virtual ~CRobot2DPoseEstimator();	//!< Destructor
			void reset(); //!< Resets all internal state.

			/** Updates the filter with new global-coordinates localization data from a localization or SLAM source. 
			  * \param tim The timestamp of the sensor readings used to evaluate localization / SLAM.
			  */
			void processUpdateNewPoseLocalization(
				const mrpt::math::TPose2D &newPose,
				mrpt::system::TTimeStamp tim);

			/** Updates the filter with new odometry readings. */
			void processUpdateNewOdometry(
				const mrpt::math::TPose2D &newGlobalOdometry,
				mrpt::system::TTimeStamp cur_tim,
				bool hasVelocities = false,
				const mrpt::math::TTwist2D &newRobotVelLocal = mrpt::math::TTwist2D() );

			/** Get the estimate for a given timestamp (defaults to `now()`), obtained as:
			*
			*   last_loc (+) [ last_odo (-) odo_ref ] (+) extrapolation_from_vw
			*
			* \return true is the estimate can be trusted. False if the real observed data is too old or there is no valid data yet.
			* \sa getLatestRobotPose
			*/
			bool getCurrentEstimate(mrpt::math::TPose2D &pose, mrpt::math::TTwist2D &velLocal, mrpt::math::TTwist2D &velGlobal, mrpt::system::TTimeStamp tim_query = mrpt::system::now()) const;

			MRPT_DEPRECATED("Use the other signature with TTwist2D output for velocities")
			bool getCurrentEstimate( mrpt::math::TPose2D &pose, float &v, float &w, mrpt::system::TTimeStamp tim_query = mrpt::system::now() ) const;
			MRPT_DEPRECATED("Use the other signature with TTwist2D output for velocities")
			bool getCurrentEstimate( mrpt::poses::CPose2D &pose, float &v, float &w, mrpt::system::TTimeStamp tim_query = mrpt::system::now() ) const;

			/** Get the latest known robot pose, either from odometry or localization.
			*  This differs from getCurrentEstimate() in that this method does NOT extrapolate as getCurrentEstimate() does.
			* \return false if there is not estimation yet.
			* \sa getCurrentEstimate
			*/
			bool getLatestRobotPose(mrpt::math::TPose2D &pose) const;

			/** \overload */
			bool getLatestRobotPose(CPose2D &pose) const;

			struct TOptions
			{
			TOptions() :
				max_odometry_age	( 1.0 ),
				max_localiz_age		( 4.0 )
			{}

			double  max_odometry_age; //!< To consider data old, in seconds
			double  max_localiz_age; //!< To consider data old, in seconds
			};

			TOptions params; //!< parameters of the filter.

		private:
			mrpt::synch::CCriticalSection  m_cs;

			mrpt::system::TTimeStamp    m_last_loc_time;
			mrpt::math::TPose2D         m_last_loc;   //!< Last pose as estimated by the localization/SLAM subsystem.

			mrpt::math::TPose2D         m_loc_odo_ref;  //!< The interpolated odometry position for the last "m_robot_pose" (used as "coordinates base" for subsequent odo readings)

			mrpt::system::TTimeStamp    m_last_odo_time;
			mrpt::math::TPose2D         m_last_odo;
			mrpt::math::TTwist2D        m_robot_vel_local; //!< Robot odometry-based velocity in a local frame of reference.

			/** An auxiliary method to extrapolate the pose of a robot located at "p" with velocities (v,w) after a time delay "delta_time". */
			static void extrapolateRobotPose(
				const mrpt::math::TPose2D  &p,
				const mrpt::math::TTwist2D &robot_vel_local,
				const double delta_time,
				mrpt::math::TPose2D &new_p);

		}; // end of class

	} // End of namespace
} // End of namespace

#endif
