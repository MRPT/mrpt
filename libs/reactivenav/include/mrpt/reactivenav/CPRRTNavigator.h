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
#ifndef CPRRTNavigator_H
#define CPRRTNavigator_H

#include <mrpt/maps.h>
#include <mrpt/poses.h>
#include <mrpt/synch.h>
#include <mrpt/system/threads.h>
#include <mrpt/reactivenav/CParameterizedTrajectoryGenerator.h>

#include <mrpt/reactivenav/link_pragmas.h>

namespace mrpt
{
  namespace reactivenav
  {
	using namespace mrpt;
	using namespace mrpt::slam;
	using namespace mrpt::math;
	using namespace mrpt::synch;
	using namespace mrpt::poses;

	/** This class is a multi-threaded mobile robot navigator, implementing an (anytime) PTG-based Rapidly-exploring Random Tree (PRRT) search algorithm.
	 *
	 *  <b>Usage:</b><br>
	 *		- Create an instance of the CPRRTNavigator class (an object on the heap, i.e. no 'new', is preferred, but just for the convenience of the user).
	 *		- Set all the parameters in CPRRTNavigator::params
	 *		- Call CPRRTNavigator::initialize. If all the params are OK, true is returned and you can go on.
	 *		- Start a navigation by calling CPRRTNavigator::navigate.
	 *		- From your application, you must update all the sensory data (from the real robot or a simulator) on a timely-fashion. The navigator will stop the robot if the last sensory data is too old.
	 *
	 *   Note that all the public methods are thread-safe.
	 *
	 *  <b>About the algorithm:</b><br>
	 *
	 *
	 *
	 *
	 *
	 * <b>Changes history</b>
	 *		- 05/JUL/2009: Creation (JLBC). This is an evolution from leassons learnt from the pure reactive navigator based on PTGs.
	 *  \ingroup mrpt_reactivenav_grp
	 */
	class REACTIVENAV_IMPEXP CPRRTNavigator
	{
	public:
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	public:
		CPRRTNavigator();	//!< Constructor
        virtual ~CPRRTNavigator();	//!< Destructor

		/** @name Navigation control methods
		@{*/
			/** Initialize the navigator using the parameters in "params"; read the usage instructions in CPRRTNavigator.
			  * This method should be called only once at the beginning of the robot operation.
			  * \return true on sucess, false on any error preparing the navigator (and no navigation command will be processed until that's fixed).
			  */
			bool initialize();

			/** Starts a navigation to a 2D pose (including a desired heading at the target).
			  * \note CPRRTNavigator::initialize must be called first.
			  */
			void  navigate( const mrpt::math::TPose2D  &target_pose);

			/** Starts a navigation to a given 2D point (any heading at the target).
			  * \note CPRRTNavigator::initialize must be called first.
			  */
			void  navigate( const mrpt::math::TPoint2D  &target_point);

			/** Cancel current navegacion.
			  * \note CPRRTNavigator::initialize must be called first.
			  */
			void cancel();

			/** Continues with suspended navigation.
			 * \sa suspend
			 * \note CPRRTNavigator::initialize must be called first.
			 */
			void resume();

			/** Suspend current navegation
			 * \sa resume
			 * \note CPRRTNavigator::initialize must be called first.
			 */
			void  suspend();

		/** @} */

		/** @name Navigator data structures
		@{*/
			/** Each data point in m_planned_path */
			struct REACTIVENAV_IMPEXP TPathData
			{
				TPathData() :
					p(0,0,0),
					max_v(0.1),max_w(0.2),
					trg_v(0.1)
				{}

				TPose2D p;			//!< Coordinates are "global"
				double max_v, max_w;	//!< Maximum velocities along this path segment.
				double trg_v;			//!< Desired linear velocity at the target point, ie: the robot should program its velocities such as after this arc the speeds are the given ones.
			};

			typedef std::list<TPathData> TPlannedPath; //!< An ordered list of path poses.

			class REACTIVENAV_IMPEXP TOptions : public mrpt::utils::CLoadableOptions
			{
			public:
				TOptions(); //!< Initial values

				/** This method load the options from a ".ini"-like file or memory-stored string list.
				 */
				void  loadFromConfigFile(
					const mrpt::utils::CConfigFileBase	&source,
					const std::string		&section);

				/** This method displays clearly all the contents of the structure in textual form, sending it to a CStream. */
				void  dumpToTextStream( CStream	&out) const;

				// Data:
				double  absolute_max_v; //!< Max linear speed (m/s)
				double  absolute_max_w; //!< Max angular speed (rad/s)
				double  max_accel_v;    //!< Max desired linear speed acceleration (m/s^2)
				double  max_accel_w;    //!< Max desired angular speed acceleration (rad/s^2)

				double  max_age_observations; //!< Max age (in seconds) for an observation to be considered invalid for navigation purposes.

				/** The robot shape used when computing collisions; it's loaded from the
				  *  config file/text as a single 2xN matrix in MATLAB format, first row are Xs, second are Ys, e.g:
				  *  \code
				  *   robot_shape = [-0.2 0.2 0.2 -0.2; -0.1 -0.1 0.1 0.1]
				  *  \endcode
				  */
				TPolygon2D robot_shape;

				struct REACTIVENAV_IMPEXP TPathTrackingOpts
				{
					double radius_checkpoints;  //!< Radius of each checkpoint in the path, ie: when the robot get closer than this distance, the point is considered as visited and the next one is processed.
				};
				TPathTrackingOpts pathtrack;

				struct REACTIVENAV_IMPEXP TPlannerOpts
				{
					double max_time_expend_planning; //!< Maximum time to spend when planning, in seconds.
				};
				TPlannerOpts planner;

			};

			TOptions  params; //!< Change here all the parameters of the navigator.

		/** @} */

		/** @name Debug and logging
		@{*/
			/** Manually sets the short-time path to be followed by the robot (use 'navigate' instead, this method is for debugging mainly)
			  */
			void setPathToFollow(const TPlannedPath &path );

			/** Returns the current planned path the robot is following */
			void getCurrentPlannedPath(TPlannedPath &path ) const;


		/** @} */


		/** @name Sensory data methods: call them to update the navigator knowledge on the outside world.
		@{*/
			/** Updates the navigator with a low or high-rate estimate from a localization (or SLAM) algorithm running somewhere else.
			  * \param new_robot_pose The (global) coordinates of the mean robot pose as estimated by some external module.
			  * \param new_robot_cov The 3x3 covariance matrix of that estimate.
			  * \param timestamp The associated timestamp of the data.
			  */
			void processNewLocalization(const TPose2D &new_robot_pose, const CMatrixDouble33 &new_robot_cov, TTimeStamp timestamp );

			/** Updates the navigator with high-rate odometry from the mobile base.
			  *  The odometry poses are dealed internally as increments only, so there is no problem is
			  *    arbitrary mismatches between localization (from a particle filter or SLAM) and the odometry.
			  * \param newOdoPose The global, cummulative odometry as computed by the robot.
			  * \param timestamp The associated timestamp of the measurement.
			  * \param hasVelocities If false, the next arguments are ignored.
			  * \param v Measured linear speed, in m/s.
			  * \param w Measured angular speed, in rad/s.
			  */
			void processNewOdometryInfo( const TPose2D &newOdoPose, TTimeStamp timestamp, bool hasVelocities =false, float v =0, float w=0 );

			/** Must be called in a timely fashion to let the navigator know about the obstacles in the environment.
			  * \param obstacles
			  * \param timestamp The associated timestamp of the sensed points.
			  */
			void processNewObstaclesData(const mrpt::slam::CPointsMap* obstacles, TTimeStamp timestamp );

		/** @} */

		/** @name Virtual methods to be implemented by the user.
		@{*/
			/** This is the most important method the user must provide: to send an instantaneous velocity command to the robot.
			  * \param v Desired linear speed, in meters/second.
			  * \param w Desired angular speed, in rads/second.
			  * \return false on any error. In that case, the navigator will immediately stop the navigation and announce the error.
			  */
			virtual bool onMotionCommand(float v, float w ) = 0;

			/** Re-implement this method if you want to know when a new navigation has started.
			  */
			virtual void onNavigationStart( ) { }

			/** Re-implement this method if you want to know when and why a navigation has ended.
			  * \param targetReachedOK Will be false if the navigation failed.
			  */
			virtual void onNavigationEnd( bool targetReachedOK ) { }

			/** Re-implement this method if you want to know when the robot is approaching the target:
			  *  this event is raised before onNavigationEnd, when the robot is closer than a certain distance to the target.
			  */
			virtual void onApproachingTarget( ) { }

		/** @} */

		static const double INVALID_HEADING; //!< Used to refer to undefined or "never mind" headings values.

	private:
		// ----------- Internal methods & threads -----------

		TThreadHandle  m_thr_planner; //!< Thread handle
		TThreadHandle  m_thr_testcol; //!< Thread handle
		TThreadHandle  m_thr_pathtrack; //!< Thread handle

		void thread_planner();  //!< Thread function
		void thread_test_collision();  //!< Thread function
		void thread_path_tracking();  //!< Thread function


		// ----------- Internal data   -----------

		bool  m_initialized;   //!< The object is ready to navigate. Users must call "initialize" first.
		bool  m_closingThreads; //!< set to true at destructor.

		TPose2D				m_target_pose; //!< Heading may be INVALID_HEADING to indicate "don't mind"
		TTimeStamp			m_target_pose_time;
		CCriticalSection	m_target_pose_cs;

		// Instead of a CSimplePointsMap, just store the X & Y vectors, since
		//  this is a temporary variable. We'll let the planning thread to build
		//  a CSimplePointsMap object with these points.
		//mrpt::slam::CSimplePointsMap	m_last_obstacles;
		std::vector<float>	m_last_obstacles_x,m_last_obstacles_y;
		TTimeStamp			m_last_obstacles_time;
		CCriticalSection	m_last_obstacles_cs;

		// The planned path, to be followed:
		CCriticalSection	m_planned_path_cs;
		TTimeStamp			m_planned_path_time;  //!< The last modification time. INVALID_TIMESTAMP means there is no path.
		TPlannedPath		m_planned_path;

		/** The list of PTGs used by the anytime planner to explore the free-space.  */
		TListPTGs			m_PTGs;
		CCriticalSection	m_PTGs_cs;

	public:

		mrpt::poses::CRobot2DPoseEstimator m_robotStateFilter; //!< Object maintained by the robot-tracking thread (All methods are thread-safe).



	};
  }
}


#endif

