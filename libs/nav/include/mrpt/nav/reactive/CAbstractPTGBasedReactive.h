/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */
#ifndef CAbstractPTGBasedReactive_H
#define CAbstractPTGBasedReactive_H

#include <mrpt/nav/reactive/CWaypointsNavigator.h>
#include <mrpt/nav/tpspace/CParameterizedTrajectoryGenerator.h>
#include <mrpt/nav/reactive/CLogFileRecord.h>
#include <mrpt/nav/holonomic/CAbstractHolonomicReactiveMethod.h>
#include <mrpt/nav/holonomic/ClearanceDiagram.h>
#include <mrpt/nav/reactive/TCandidateMovementPTG.h>
#include <mrpt/nav/reactive/CMultiObjectiveMotionOptimizerBase.h>
#include <mrpt/utils/CTimeLogger.h>
#include <mrpt/system/datetime.h>
#include <mrpt/math/filters.h>
#include <mrpt/synch/CCriticalSection.h>
#include <mrpt/math/CPolygon.h>
#include <mrpt/maps/CPointCloudFilterBase.h>

namespace mrpt
{
  namespace nav
  {
	/** Base class for reactive navigator systems based on TP-Space, with an arbitrary holonomic
	  * reactive method running on it and any number of PTGs for transforming the navigation space.
	  * Both, the holonomic method and the PTGs can be customized by the apropriate user derived classes.
	  *
	  * How to use:
	  *  - Instantiate a reactive navigation object (one of the derived classes of this virtual class).
	  *  - A class with callbacks must be defined by the user and provided to the constructor (derived from CRobot2NavInterface)
	  *  - loadConfigFile() must be called to set up the bunch of parameters from a config file (could be a memory-based virtual config file).
	  *  - navigationStep() must be called periodically in order to effectively run the navigation. This method will internally call the callbacks to gather sensor data and robot positioning data.
	  *
	  * For working examples, refer to the source code of the apps:
	  *  - [ReactiveNavigationDemo](http://www.mrpt.org/list-of-mrpt-apps/application-reactivenavigationdemo/)
	  *  - [ReactiveNav3D-Demo](http://www.mrpt.org/list-of-mrpt-apps/application-reactivenav3d-demo/)
	  *
	  * Publications:
	  *  - See derived classes for papers on each specific method.
	  *
	  * Available "variables" or "score names" for each motion candidate (these can be used in runtime-compiled expressions
	  * in the configuration files of motion deciders):
	  *
	  * - `clearance`: Clearance (larger means larger distances to obstacles) for the path from "current pose" up to "end of trajectory".
	  * - `collision_free_distance`: Normalized [0,1] collision-free distance in selected path. For NOP candidates, the traveled distances is substracted.
	  * - `dist_eucl_final`: Euclidean distance (in the real-world WordSpace) between "end of trajectory" and target.
	  * - `eta`: Estimated Time of Arrival at "end of trajectory".
	  * - `hysteresis`: Measure of similarity with previous command [0,1]
	  * - `is_PTG_cont`: 1 (is "NOP" motion command), 0 otherwise
	  * - `is_slowdown`: 1 if PTG returns true in CParameterizedTrajectoryGenerator::supportSpeedAtTarget() for this step.
	  * - `move_cur_d`: Normalized distance already traveled over the selected PTG. Normally 0, unless in a "NOP motion".
	  * - `move_k`: Motion candidate path 0-based index.
	  * - `num_paths`: Number of paths in the PTG
	  * - `original_col_free_dist`: Only for "NOP motions", the collision-free distance when the motion command was originally issued.
	  * - `ptg_idx`: PTG index (0-based)
	  * - `ptg_priority`: Product of PTG getScorePriority() times PTG evalPathRelativePriority()
	  * - `ref_dist`: PTG ref distance [m]
	  * - `robpose_x`, `robpose_y`, `robpose_phi`: Robot pose ([m] and [rad]) at the "end of trajectory": at collision or at target distance.
	  * - `target_d_norm`: Normalized target distance. Can be >1 if distance is larger than ref_distance.
	  * - `target_dir`: Angle of target in TP-Space [rad]
	  * - `target_k`: Same as target_dir but in discrete path 0-based indices.
	  * - `WS_target_x`, `WS_target_y`: Target coordinates in realworld [m]
	  *
	  * \sa CReactiveNavigationSystem, CReactiveNavigationSystem3D
	  *  \ingroup nav_reactive
	  */
	class NAV_IMPEXP CAbstractPTGBasedReactive: public CWaypointsNavigator
	{
	public:
		MRPT_MAKE_ALIGNED_OPERATOR_NEW

		/** The struct for configuring navigation requests to CAbstractPTGBasedReactive and derived classes. */
		struct NAV_IMPEXP TNavigationParamsPTG : public CWaypointsNavigator::TNavigationParamsWaypoints
		{
			/** (Default=empty) Optionally, a list of PTG indices can be sent such that
			 *  the navigator will restrict itself to only employ those PTGs. */
			std::vector<size_t>    restrict_PTG_indices;

			virtual std::string getAsText() const MRPT_OVERRIDE;
			virtual TNavigationParamsBase* clone() const MRPT_OVERRIDE { return new TNavigationParamsPTG(*this); }
		protected:
			virtual bool isEqual(const CAbstractNavigator::TNavigationParamsBase& o) const MRPT_OVERRIDE;
		};

		/** Constructor.
		  * \param[in] react_iterf_impl An instance of an object that implement all the required interfaces to read from and control a robot.
		  * \param[in] enableConsoleOutput Can be set to false to reduce verbosity.
		  * \param[in] enableLogFile Set to true to enable creation of navigation log files, useful for inspection and debugging.
		  */
		CAbstractPTGBasedReactive(
			CRobot2NavInterface &react_iterf_impl,
			bool enableConsoleOutput = true,
			bool enableLogFile = false,
			const std::string &logFileDirectory = std::string("./reactivenav.logs"));

		virtual ~CAbstractPTGBasedReactive();

		/** Must be called for loading collision grids, or the first navigation command may last a long time to be executed.
		  * Internally, it just calls STEP1_CollisionGridsBuilder().
		  */
		void initialize() MRPT_OVERRIDE;

		/** Selects which one from the set of available holonomic methods will be used
			*  into transformed TP-Space, and sets its configuration from a configuration file.
			* Available methods: class names of those derived from CAbstractHolonomicReactiveMethod
			*/
		void setHolonomicMethod(const std::string & method, const mrpt::utils::CConfigFileBase & cfgBase);

		MRPT_DEPRECATED("Use the signature with a class name string instead of an enum.")
		void setHolonomicMethod(THolonomicMethod method, const mrpt::utils::CConfigFileBase & cfgBase);

		/** Provides a copy of the last log record with information about execution.
			* \param o An object where the log will be stored into.
			* \note Log records are not prepared unless either "enableLogFile" is enabled in the constructor or "enableLogFile()" has been called.
			*/
		void getLastLogRecord( CLogFileRecord &o );

		/** Enables keeping an internal registry of navigation logs that can be queried with getLastLogRecord() */
		void enableKeepLogRecords(bool enable=true) { m_enableKeepLogRecords=enable; }

		/** Enables/disables saving log files. */
		void enableLogFile(bool enable);

		/** Changes the prefix for new log files. */
		void setLogFileDirectory(const std::string &sDir) { m_navlogfiles_dir = sDir; }
		std::string getLogFileDirectory() const { return m_navlogfiles_dir; }


		struct NAV_IMPEXP TAbstractPTGNavigatorParams : public mrpt::utils::CLoadableOptions
		{
			std::string holonomic_method; //!< C++ class name of the holonomic navigation method to run in the transformed TP-Space
			std::string motion_decider_method; //!< C++ class name of the motion chooser

			std::string ptg_cache_files_directory; //!< (Default: ".")
			double ref_distance;          //!< Maximum distance up to obstacles will be considered (D_{max} in papers).
			double speedfilter_tau;     //!< Time constant (in seconds) for the low-pass filter applied to kinematic velocity commands (default=0: no filtering)

			/** In normalized distances, the start and end of a ramp function that scales the velocity
			*  output from the holonomic navigator:
			*
			* \code
			*  velocity scale
			*   ^
			*   |           _____________
			*   |          /
			* 1 |         /
			*   |        /
			* 0 +-------+---|----------------> normalized distance
			*         Start
			*              End
			* \endcode
			*
			*/
			double secure_distance_start, secure_distance_end;
			bool   use_delays_model;
			double max_distance_predicted_actual_path; //!< Max distance [meters] to discard current PTG and issue a new vel cmd (default= 0.05)
			double min_normalized_free_space_for_ptg_continuation; //!< Min normalized dist [0,1] after current pose in a PTG continuation to allow it.

			mrpt::kinematics::CVehicleVelCmd::TVelCmdParams robot_absolute_speed_limits; //!< Params related to speed limits.
			bool  enable_obstacle_filtering;
			bool  evaluate_clearance; //!< Default: false
			double max_dist_for_timebased_path_prediction; //!< Max dist [meters] to use time-based path prediction for NOP evaluation.

			virtual void loadFromConfigFile(const mrpt::utils::CConfigFileBase &c, const std::string &s) MRPT_OVERRIDE;
			virtual void saveToConfigFile(mrpt::utils::CConfigFileBase &c, const std::string &s) const MRPT_OVERRIDE;
			TAbstractPTGNavigatorParams();
		};

		TAbstractPTGNavigatorParams params_abstract_ptg_navigator;

		virtual void loadConfigFile(const mrpt::utils::CConfigFileBase &c) MRPT_OVERRIDE; // See base class docs!
		virtual void saveConfigFile(mrpt::utils::CConfigFileBase &c) const MRPT_OVERRIDE; // See base class docs!


		/** Enables/disables the detailed time logger (default:disabled upon construction)
			*  When enabled, a report will be dumped to std::cout upon destruction.
			* \sa getTimeLogger
			*/
		void enableTimeLog(bool enable=true) {
			m_timelogger.enable(enable);
		}

		/** Gives access to a const-ref to the internal time logger \sa enableTimeLog */
		const mrpt::utils::CTimeLogger & getTimeLogger() const { return m_timelogger; }

		virtual size_t getPTG_count() const = 0;  //!< Returns the number of different PTGs that have been setup
		virtual CParameterizedTrajectoryGenerator* getPTG(size_t i) = 0; //!< Gets the i'th PTG
		virtual const CParameterizedTrajectoryGenerator* getPTG(size_t i) const = 0; //!< Gets the i'th PTG

		/** Get the current, global (honored for all PTGs) robot speed limits */
		const mrpt::kinematics::CVehicleVelCmd::TVelCmdParams & getCurrentRobotSpeedLimits() const {
			return params_abstract_ptg_navigator.robot_absolute_speed_limits;
		}

		/** Changes the current, global (honored for all PTGs) robot speed limits, via returning a reference to a structure that holds those limits */
		mrpt::kinematics::CVehicleVelCmd::TVelCmdParams & changeCurrentRobotSpeedLimits() {
			return params_abstract_ptg_navigator.robot_absolute_speed_limits;
		}

		void setTargetApproachSlowDownDistance(const double dist);  //!< Changes this parameter in all inner holonomic navigator instances [m].
		double getTargetApproachSlowDownDistance() const;  //!< Returns this parameter for the first inner holonomic navigator instances  [m] (should be the same in all of them?)

	protected:
		/** The main method for the navigator */
		virtual void performNavigationStep() MRPT_OVERRIDE;

		std::vector<CAbstractHolonomicReactiveMethod*>  m_holonomicMethod;   //!< The holonomic navigation algorithm (one object per PTG, so internal states are maintained)
		mrpt::utils::CStream  *m_logFile, *m_prev_logfile;         //!< The current log file stream, or NULL if not being used
		bool                   m_enableKeepLogRecords; //!< See enableKeepLogRecords
		CLogFileRecord lastLogRecord;  //!< The last log
		mrpt::kinematics::CVehicleVelCmdPtr m_last_vel_cmd ; //!< Last velocity commands

		mrpt::synch::CCriticalSectionRecursive  m_critZoneLastLog; //!< Critical zones

		bool    m_enableConsoleOutput;  //!< Enables / disables the console debug output.
		bool    m_init_done;            //!< Whether \a loadConfigFile() has been called or not.
		mrpt::utils::CTicTac	timerForExecutionPeriod;

		mrpt::utils::CTimeLogger m_timelogger; //!< A complete time logger \sa enableTimeLog()
		bool  m_PTGsMustBeReInitialized;

		/** @name Variables for CReactiveNavigationSystem::performNavigationStep
			@{ */
		mrpt::utils::CTicTac totalExecutionTime, executionTime, tictac;
		mrpt::math::LowPassFilter_IIR1  meanExecutionTime;
		mrpt::math::LowPassFilter_IIR1  meanTotalExecutionTime;
		mrpt::math::LowPassFilter_IIR1  meanExecutionPeriod;    //!< Runtime estimation of execution period of the method.
		mrpt::math::LowPassFilter_IIR1  tim_changeSpeed_avr, timoff_obstacles_avr, timoff_curPoseAndSpeed_avr, timoff_sendVelCmd_avr;
		/** @} */

		virtual bool impl_waypoint_is_reachable(const mrpt::math::TPoint2D &wp_local_wrt_robot) const MRPT_OVERRIDE; // See docs in base class

		// Steps for the reactive navigation sytem.
		// ----------------------------------------------------------------------------
		virtual void STEP1_InitPTGs() = 0;

		/** Return false on any fatal error */
		virtual bool implementSenseObstacles(mrpt::system::TTimeStamp &obs_timestamp) = 0;
		bool STEP2_SenseObstacles();

		/** Builds TP-Obstacles from Workspace obstacles for the given PTG.
		  * "out_TPObstacles" is already initialized to the proper length and maximum collision-free distance for each "k" trajectory index.
		  * Distances are in "pseudo-meters". They will be normalized automatically to [0,1] upon return. */
		virtual void STEP3_WSpaceToTPSpace(const size_t ptg_idx,std::vector<double> &out_TPObstacles, mrpt::nav::ClearanceDiagram &out_clearance, const mrpt::math::TPose2D &rel_pose_PTG_origin_wrt_sense, const bool eval_clearance) = 0;

		/** Generates a pointcloud of obstacles, and the robot shape, to be saved in the logging record for the current timestep */
		virtual void loggingGetWSObstaclesAndShape(CLogFileRecord &out_log) = 0;

		struct PTGTarget
		{
			bool                 valid_TP;   //!< For each PTG, whether target falls into the PTG domain.
			mrpt::math::TPoint2D TP_Target;  //!< The Target, in TP-Space (x,y)
			double               target_alpha, target_dist;  //!< TP-Target
			int                  target_k; //!< The discrete version of target_alpha

			PTGTarget() : valid_TP(false) {}
		};

		/** Scores \a holonomicMovement */
		void calc_move_candidate_scores(TCandidateMovementPTG         & holonomicMovement,
			const std::vector<double>        & in_TPObstacles,
			const mrpt::nav::ClearanceDiagram & in_clearance,
			const std::vector<mrpt::math::TPose2D>  & WS_Targets,
			const std::vector<PTGTarget> & TP_Targets,
			CLogFileRecord::TInfoPerPTG & log,
			CLogFileRecord & newLogRec,
			const bool this_is_PTG_continuation,
			const mrpt::math::TPose2D &relPoseVelCmd_NOP,
			const unsigned int ptg_idx4weights,
			const mrpt::system::TTimeStamp tim_start_iteration);

		/** Return the [0,1] velocity scale of raw PTG cmd_vel */
		virtual double generate_vel_cmd(const TCandidateMovementPTG &in_movement, mrpt::kinematics::CVehicleVelCmdPtr &new_vel_cmd );
		void STEP8_GenerateLogRecord(CLogFileRecord &newLogRec, const std::vector<mrpt::math::TPose2D>& relTargets, int nSelectedPTG, const mrpt::kinematics::CVehicleVelCmdPtr &new_vel_cmd, int nPTGs, const bool best_is_NOP_cmdvel, const math::TPose2D &rel_cur_pose_wrt_last_vel_cmd_NOP, const math::TPose2D &rel_pose_PTG_origin_wrt_sense_NOP, const double executionTimeValue, const double tim_changeSpeed, const mrpt::system::TTimeStamp &tim_start_iteration);
		void preDestructor(); //!< To be called during children destructors to assure thread-safe destruction, and free of shared objects.
		virtual void onStartNewNavigation() MRPT_OVERRIDE;

		bool m_closing_navigator; //!< Signal that the destructor has been called, so no more calls are accepted from other threads

		mrpt::system::TTimeStamp m_WS_Obstacles_timestamp;
		mrpt::maps::CPointCloudFilterBasePtr m_WS_filter;   //!< Default: none

		mrpt::nav::CMultiObjectiveMotionOptimizerBasePtr m_multiobjopt;


		struct TInfoPerPTG
		{
			std::vector<PTGTarget> targets;
			std::vector<double>  TP_Obstacles; //!< One distance per discretized alpha value, describing the "polar plot" of TP obstacles.
			ClearanceDiagram     clearance;    //!< Clearance for each path
		};

		std::vector<TInfoPerPTG> m_infoPerPTG; //!< Temporary buffers for working with each PTG during a navigationStep()
		mrpt::system::TTimeStamp m_infoPerPTG_timestamp;

		void build_movement_candidate(CParameterizedTrajectoryGenerator * ptg,
			const size_t indexPTG,
			const std::vector<mrpt::math::TPose2D> &relTargets,
			const mrpt::math::TPose2D &rel_pose_PTG_origin_wrt_sense,
			TInfoPerPTG &ipf,
			TCandidateMovementPTG &holonomicMovement,
			CLogFileRecord &newLogRec,
			const bool this_is_PTG_continuation,
			mrpt::nav::CAbstractHolonomicReactiveMethod *holoMethod,
			const mrpt::system::TTimeStamp tim_start_iteration,
			const TNavigationParams &navp = TNavigationParams(),
			const mrpt::math::TPose2D &relPoseVelCmd_NOP = mrpt::poses::CPose2D()
		);

		struct NAV_IMPEXP TSentVelCmd
		{
			int ptg_index; //!< 0-based index of used PTG
			int ptg_alpha_index; //!< Path index for selected PTG
			mrpt::system::TTimeStamp tim_send_cmd_vel; //!< Timestamp of when the cmd was sent
			TRobotPoseVel  poseVel;  //!< Robot pose & velocities and timestamp of when it was queried
			double colfreedist_move_k; //!< TP-Obstacles in the move direction at the instant of picking this movement
			bool   was_slowdown;
			double speed_scale;        //!< [0,1] scale of the raw cmd_vel as generated by the PTG
			CParameterizedTrajectoryGenerator::TNavDynamicState ptg_dynState;

			bool isValid() const;
			void reset();
			TSentVelCmd();
		};

		TSentVelCmd m_lastSentVelCmd;

	private:
		void deleteHolonomicObjects(); //!< Delete m_holonomicMethod

		std::string m_navlogfiles_dir; //!< Default: "./reactivenav.logs"

		double m_expr_var_k, m_expr_var_k_target, m_expr_var_num_paths;
		TNavigationParams  *m_copy_prev_navParams; //!< A copy of last-iteration navparams, used to detect changes
		
	}; // end of CAbstractPTGBasedReactive
  }
}


#endif
