/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2016, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */
#ifndef CAbstractPTGBasedReactive_H
#define CAbstractPTGBasedReactive_H

#include <mrpt/nav/reactive/CWaypointsNavigator.h>
#include <mrpt/nav/tpspace/CParameterizedTrajectoryGenerator.h>
#include <mrpt/nav/reactive/CLogFileRecord.h>
#include <mrpt/nav/holonomic/CAbstractHolonomicReactiveMethod.h>
#include <mrpt/utils/CTimeLogger.h>
#include <mrpt/system/datetime.h>
#include <mrpt/synch/CCriticalSection.h>
#include <mrpt/math/CPolygon.h>

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
	  * \sa CReactiveNavigationSystem, CReactiveNavigationSystem3D
	  *  \ingroup nav_reactive
	  */
	class NAV_IMPEXP CAbstractPTGBasedReactive: public CWaypointsNavigator
	{
	public:
		MRPT_MAKE_ALIGNED_OPERATOR_NEW

		/** The struct for configuring navigation requests to CAbstractPTGBasedReactive and derived classes. */
		struct NAV_IMPEXP TNavigationParamsPTG : public CAbstractNavigator::TNavigationParams
		{
			/** (Default=empty) Optionally, a list of PTG indices can be sent such that
			 *  the navigator will restrict itself to only employ those PTGs. */
			std::vector<size_t>    restrict_PTG_indices;

			TNavigationParamsPTG() { }
			virtual ~TNavigationParamsPTG() { }
			virtual std::string getAsText() const;
			virtual TNavigationParams* clone() const { return new TNavigationParamsPTG(*this); }
		};


		/** Constructor.
		  * \param[in] react_iterf_impl An instance of an object that implement all the required interfaces to read from and control a robot.
		  * \param[in] enableConsoleOutput Can be set to false to reduce verbosity.
		  * \param[in] enableLogFile Set to true to enable creation of navigation log files, useful for inspection and debugging.
		  */
		CAbstractPTGBasedReactive(
			CRobot2NavInterface &react_iterf_impl,
			bool enableConsoleOutput = true,
			bool enableLogFile = false);

		virtual ~CAbstractPTGBasedReactive();

		/** Must be called for loading collision grids, or the first navigation command may last a long time to be executed.
		  * Internally, it just calls STEP1_CollisionGridsBuilder().
		  */
		void initialize() MRPT_OVERRIDE;

		/** Selects which one from the set of available holonomic methods will be used
			*  into transformed TP-Space, and sets its configuration from a configuration file.*/
		void setHolonomicMethod(
			THolonomicMethod	method,
			const mrpt::utils::CConfigFileBase & cfgBase);

		/** Just loads the holonomic method params from the given config source \sa setHolonomicMethod */
		void loadHolonomicMethodConfig(
			const mrpt::utils::CConfigFileBase &ini,
			const std::string &section );

		/** Provides a copy of the last log record with information about execution.
			* \param o An object where the log will be stored into.
			* \note Log records are not prepared unless either "enableLogFile" is enabled in the constructor or "enableLogFile()" has been called.
			*/
		void getLastLogRecord( CLogFileRecord &o );

		/** Enables keeping an internal registry of navigation logs that can be queried with getLastLogRecord() */
		void enableKeepLogRecords(bool enable=true) { m_enableKeepLogRecords=enable; }

		/** Enables/disables saving log files. */
		void enableLogFile(bool enable);

		/** Enables/disables the detailed time logger (default:disabled upon construction)
			*  When enabled, a report will be dumped to std::cout upon destruction.
			* \sa getTimeLogger
			*/
		void enableTimeLog(bool enable=true) {
			MRPT_UNUSED_PARAM(enable);
			m_timelogger.enable(true);
		}

		/** Gives access to a const-ref to the internal time logger \sa enableTimeLog */
		const mrpt::utils::CTimeLogger & getTimeLogger() const { return m_timelogger; }

		
		virtual size_t getPTG_count() const = 0;  //!< Returns the number of different PTGs that have been setup
		virtual CParameterizedTrajectoryGenerator* getPTG(size_t i) = 0; //!< Gets the i'th PTG
		virtual const CParameterizedTrajectoryGenerator* getPTG(size_t i) const = 0; //!< Gets the i'th PTG

		/** Reload the configuration from a file. See details in CReactiveNavigationSystem docs.
			* Section to be read is "{sect_prefix}ReactiveParams". */
		void loadConfigFile(const mrpt::utils::CConfigFileBase &cfg, const std::string &section_prefix="") MRPT_OVERRIDE;

	protected:
		// ------------------------------------------------------
		//					INTERNAL DEFINITIONS
		// ------------------------------------------------------
		/** The structure used for storing a movement generated by a holonomic-method. */
		struct THolonomicMovement {
				CParameterizedTrajectoryGenerator	*PTG;	//!< The associated PTG
				double	direction, speed;					//!< The holonomic movement. Speed is normalized wrt to [0,1]
				double	evaluation;							//!< An evaluation in the range [0,1] for the goodness of the movement

				THolonomicMovement() : PTG(NULL),direction(0),speed(0),evaluation(0) {}
		};

		// ------------------------------------------------------
		//					PRIVATE METHODS
		// ------------------------------------------------------
		/** The main method for the navigator */
		virtual void  performNavigationStep() MRPT_OVERRIDE;

		// ------------------------------------------------------
		//					PRIVATE	VARIABLES
		// ------------------------------------------------------
		std::vector<CAbstractHolonomicReactiveMethod*>  m_holonomicMethod;   //!< The holonomic navigation algorithm (one object per PTG, so internal states are maintained)
		mrpt::utils::CStream  *m_logFile;           //!< The current log file stream, or NULL if not being used
		bool                   m_enableKeepLogRecords; //!< See enableKeepLogRecords
		CLogFileRecord lastLogRecord;  //!< The last log
		//float last_cmd_v, last_cmd_w, new_cmd_v, new_cmd_w;  //!< Speed actual and last commands
		std::vector<double> m_last_vel_cmd, m_new_vel_cmd; //!< Actual and last velocity commands
		std::vector<std::vector<double> > m_cmd_vel_filterings; //!< Logged values of temporary vel cmds before reaching at the final value sent to the robot (for logging)

		mrpt::synch::CCriticalSectionRecursive  m_critZoneLastLog; //!< Critical zones

		bool    m_enableConsoleOutput;  //!< Enables / disables the console debug output.
		bool    m_init_done;            //!< Whether \a loadConfigFile() has been called or not.
		mrpt::utils::CTicTac	timerForExecutionPeriod;

		// PTG params loaded from INI file:
		std::string ptg_cache_files_directory; //!< (Default: ".")
		std::string robotName;       //!< Robot name
		double refDistance;          //!< "D_{max}" in papers.
		double SPEEDFILTER_TAU;     //!< Time constant (in seconds) for the low-pass filter applied to kinematic velocity commands (default=0: no filtering)
		std::vector<float> weights;  //!< length: 6 [0,5]

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
		double secureDistanceStart,secureDistanceEnd;

		float  meanExecutionPeriod;	//!< Runtime estimation of execution period of the method.
		mrpt::utils::CTimeLogger m_timelogger;			//!< A complete time logger \sa enableTimeLog()
		bool  m_PTGsMustBeReInitialized;

		/** @name Variables for CReactiveNavigationSystem::performNavigationStep
			@{ */
		mrpt::utils::CTicTac totalExecutionTime, executionTime, tictac;
		float meanExecutionTime, meanTotalExecutionTime;
		/** @} */

		/** Loads derived-class specific parameters */
		virtual void internal_loadConfigFile(const mrpt::utils::CConfigFileBase &cfg, const std::string &section_prefix="") = 0;

		virtual bool impl_waypoint_is_reachable(const mrpt::math::TPoint2D &wp_local_wrt_robot) const MRPT_OVERRIDE; // See docs in base class

		// Steps for the reactive navigation sytem.
		// ----------------------------------------------------------------------------
		virtual void STEP1_InitPTGs() = 0;

		/** Return false on any fatal error */
		virtual bool STEP2_SenseObstacles() = 0;

		/** Builds TP-Obstacles from Workspace obstacles for the given PTG.
		  * "out_TPObstacles" is already initialized to the proper length and maximum collision-free distance for each "k" trajectory index.
		  * Distances are in "pseudo-meters". They will be normalized automatically to [0,1] upon return. */
		virtual void STEP3_WSpaceToTPSpace(const size_t ptg_idx,std::vector<double> &out_TPObstacles) = 0;

		/** Generates a pointcloud of obstacles, and the robot shape, to be saved in the logging record for the current timestep */
		virtual void loggingGetWSObstaclesAndShape(CLogFileRecord &out_log) = 0;


		/** Scores \a holonomicMovement */
		void STEP5_PTGEvaluator(
			THolonomicMovement         & holonomicMovement,
			const std::vector<double>        & in_TPObstacles,
			const mrpt::math::TPose2D  & WS_Target,
			const mrpt::math::TPoint2D & TP_Target,
			CLogFileRecord::TInfoPerPTG & log );

		virtual void STEP7_GenerateSpeedCommands(const THolonomicMovement &in_movement);


		void preDestructor(); //!< To be called during children destructors to assure thread-safe destruction, and free of shared objects.


	private:
		bool m_closing_navigator; //!< Signal that the destructor has been called, so no more calls are accepted from other threads

		struct TInfoPerPTG
		{
			bool                 valid_TP;   //!< For each PTG, whether the target falls into the PTG domain.
			mrpt::math::TPoint2D TP_Target; //!< The Target, in TP-Space (x,y)
			double               target_alpha,target_dist;  //!< TP-Target
			int                  target_k; //!< The discrete version of target_alpha
			std::vector<double>  TP_Obstacles; //!< One distance per discretized alpha value, describing the "polar plot" of TP obstacles.
		};

		std::vector<TInfoPerPTG> m_infoPerPTG; //!< Temporary buffers for working with each PTG during a navigationStep()
		mrpt::system::TTimeStamp m_infoPerPTG_timestamp;


		void deleteHolonomicObjects(); //!< Delete m_holonomicMethod


	}; // end of CAbstractPTGBasedReactive
  }
}


#endif

