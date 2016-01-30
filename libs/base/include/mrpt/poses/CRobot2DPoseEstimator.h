/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2016, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */
#ifndef CRobot2DPoseEstimator_H
#define CRobot2DPoseEstimator_H

#include <mrpt/synch/CCriticalSection.h>
#include <mrpt/math/lightweight_geom_data.h>
#include <mrpt/math/CMatrixFixedNumeric.h>
#include <mrpt/system/datetime.h>

namespace mrpt
{
	namespace poses
	{
		/** A simple filter to estimate and extrapolate the robot 2D (x,y,phi) pose from asynchronous odometry and localization data.
		  *  The implemented model is a state vector:
		  *		- (x,y,phi,v,w)
		  *  for the robot pose (x,y,phi) and velocities (v,w).
		  *
		  *  The filter can be asked for an extrapolation for some arbitrary time "t'", and it'll do a simple linear prediction.
		  *  All methods are thread-safe.
		  * \ingroup poses_grp poses_pdf_grp
		  */
		class BASE_IMPEXP CRobot2DPoseEstimator
		{
		public:
			 CRobot2DPoseEstimator( ); //!< Default constructor
			 virtual ~CRobot2DPoseEstimator();	//!< Destructor
			 void reset();

			 /** Updates the filter so the pose is tracked to the current time */
			 void processUpdateNewPoseLocalization(
				 const mrpt::math::TPose2D &newPose,
				 const mrpt::math::CMatrixDouble33 &newPoseCov,
				 mrpt::system::TTimeStamp cur_tim);

			 /** Updates the filter so the pose is tracked to the current time */
			 void processUpdateNewOdometry(
				 const mrpt::math::TPose2D &newGlobalOdometry,
				 mrpt::system::TTimeStamp cur_tim,
				 bool hasVelocities = false,
				 float v = 0,
				 float w = 0);

			 /** Get the current estimate, obtained as:
			   *
			   *   last_loc (+) [ last_odo (-) odo_ref ] (+) extrapolation_from_vw
			   *
			   * \return true is the estimate can be trusted. False if the real observed data is too old or there is no valid data yet.
			   * \sa getLatestRobotPose
			   */
			 bool getCurrentEstimate( mrpt::math::TPose2D &pose, float &v, float &w, mrpt::system::TTimeStamp tim_query = mrpt::system::now() ) const;

			 /** \overload */
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

			mrpt::system::TTimeStamp		m_last_loc_time;
			mrpt::math::TPose2D			m_last_loc;   //!< Last pose as estimated by the localization/SLAM subsystem.
			mrpt::math::CMatrixDouble33 m_last_loc_cov;

			mrpt::math::TPose2D			m_loc_odo_ref;  //!< The interpolated odometry position for the last "m_robot_pose" (used as "coordinates base" for subsequent odo readings)

			mrpt::system::TTimeStamp		m_last_odo_time;
			mrpt::math::TPose2D			m_last_odo;
			float			m_robot_v;
			float			m_robot_w;

			/** An auxiliary method to extrapolate the pose of a robot located at "p" with velocities (v,w) after a time delay "delta_time".
			  */
			static void extrapolateRobotPose(
				const mrpt::math::TPose2D &p,
				const float v,
				const float w,
				const double delta_time,
				mrpt::math::TPose2D &new_p);

		}; // end of class

	} // End of namespace
} // End of namespace

#endif
