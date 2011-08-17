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
#ifndef CReactiveNavigationSystem_H
#define CReactiveNavigationSystem_H

#include <mrpt/maps.h>
#include <mrpt/poses.h>
#include <mrpt/math.h>
#include <mrpt/synch.h>
#include <mrpt/reactivenav/link_pragmas.h>

#include "CAbstractReactiveNavigationSystem.h"
#include "CParameterizedTrajectoryGenerator.h"
#include "CLogFileRecord.h"
#include "CAbstractHolonomicReactiveMethod.h"
#include "CHolonomicVFF.h"
#include "CHolonomicND.h"

namespace mrpt
{
	/** This namespace contains classes for building a TP-Space Reactive Navigation System.
	*/
  namespace reactivenav
  {
	  /**  The implemented reactive navigation methods
	    *  \ingroup mrpt_reactivenav_grp
	    */
	  enum THolonomicMethod
	  {
		  hmVIRTUAL_FORCE_FIELDS = 0,
		  hmSEARCH_FOR_BEST_GAP = 1
	  };

		/** Implements a reactive navigation system based on TP-Space, with an arbitrary holonomic
		*  reactive method running on it, and any desired number of PTG for transforming the navigation space.
		*  Both, the holonomic method and the PTGs can be customized by the apropriate user derived classes.
		*
		*   How to use:
		*      - A class with callbacks must be defined by the user and provided to the constructor.
		*      - loadConfigFile() must be called to set up the bunch of parameters from a config file (could be a memory-based virtual config file).
		*      - navigationStep() must be called periodically in order to effectively run the navigation. This method will internally call the callbacks to gather sensor data and robot positioning data.
		*
		* - 17/JUN/2004: First design.
		* - 16/SEP/2004: Totally redesigned, according to document "MultiParametric Based Space Transformation for Reactive Navigation"
		* - 29/SEP/2005: Totally rewritten again, for integration into MRPT library and according to the ICRA paper.
		* - 17/OCT/2007: Whole code updated to accomodate to MRPT 0.5 and make it portable to Linux.
		*
		*  \sa CAbstractReactiveNavigationSystem, CParameterizedTrajectoryGenerator, CAbstractHolonomicReactiveMethod
		*  \ingroup mrpt_reactivenav_grp
		*/
		class REACTIVENAV_IMPEXP  CReactiveNavigationSystem : public CAbstractReactiveNavigationSystem
		{
		public:
			EIGEN_MAKE_ALIGNED_OPERATOR_NEW
		public:
			/** Constructor
			 *  \param configINIFile The file to load the configuration from. See loadConfigFile
			 *  \param robotConfigFile The file to load the robot specific configuration from.
			 *	\param rmc A set of wrappers that must be filled in.
			 *	\param sensors A set of wrappers that must be filled in.
			 *	\param dbg A set of wrappers that must be filled in.
			 *	\param evnts A set of wrappers that must be filled in.
			 *  \param enableConsoleOutput Set to false if console output is not desired.
			 *  \param enableLogFile Set to true to enable logging to file.
			 */
			CReactiveNavigationSystem(
				CReactiveInterfaceImplementation   &react_iterf_impl,
				bool					enableConsoleOutput = true,
				bool					enableLogFile = false);

			/** Destructor
			 */
			virtual ~CReactiveNavigationSystem();

			/** Reload the configuration from a file
			 */
			void loadConfigFile(const mrpt::utils::CConfigFileBase &ini, const mrpt::utils::CConfigFileBase &robotIni);

			/** Must be called for loading collision grids, or the first navigation
			  *   command may last a long time to be executed.
			  */
			void initialize();

			/** Evaluate navigation hardness:
			  */
			float  evaluate( TNavigationParams *params );

			/** Start navigation:
			  */
			void  navigate( TNavigationParams *params );

			/** Change current navigation params:
			  */
			void  setParams( TNavigationParams *params );

			/** Selects which one from the set of available holonomic methods will be used
			  *  into transformed TP-Space, and sets its configuration from a configuration file.
			  */
			void setHolonomicMethod(
				THolonomicMethod	method,
				const char			*config_INIfile = "./CONFIG_ReactiveNavigator.ini");

			/** Change the robot shape, which is taken into account for collision
			  *  grid building.
			  */
			void changeRobotShape( math::CPolygon &shape );

			/** Provides a copy of the last log record with information about execution. On any unexpected error "*o" will be NULL.
			  * \param o An object where the log will be stored into.
			  */
			void getLastLogRecord( CLogFileRecord &o );

			/** Enables / disables the logging into a file.
			  */
			void enableLogFile(bool enable);

		private:
			// ------------------------------------------------------
			//					PRIVATE DEFINITIONS
			// ------------------------------------------------------
			/** The structure used for storing a movement generated by a holonomic-method .
			  */
			struct THolonomicMovement {
					CParameterizedTrajectoryGenerator		*PTG;	/// The associated PTG
					double	direction, speed;	/// The holonomic movement
					double	evaluation;	/// An evaluation in the range [0,1] for the goodness of the movement.
			};

			/** The last log.
			  */
			CLogFileRecord		lastLogRecord;

			/** For the histeresis: */
			float		last_cmd_v,last_cmd_w;

			/** Will be false until the navigation end is sent, and it is reset with each new command
			  */
			bool		navigationEndEventSent;

			/** Critical zones: */
            synch::CCriticalSection  m_critZoneLastLog,m_critZoneNavigating;

			// ------------------------------------------------------
			//					PRIVATE METHODS
			// ------------------------------------------------------
			/** The main method for the navigator */
			void  performNavigationStep( );

			// ------------------------------------------------------
			//					PRIVATE	VARIABLES
			// ------------------------------------------------------

			CAbstractHolonomicReactiveMethod    *holonomicMethod;   //!< The holonomic navigation algorithm.
			mrpt::utils::CStream                *logFile;           //!< The current log file stream, or NULL if not being used

			bool  m_enableConsoleOutput;  //!< Enables / disables the console debug output.

			bool  m_init_done;            //!< Whether \a loadConfigFile() has been called or not.

			CTicTac	timerForExecutionPeriod;

			// Loaded from INI file:
			std::string robotName;				// El nombre del robot donde estamos
			float   refDistance;				// "dmax" in papers.
			float   colGridRes_x,colGridRes_y;  // Resolucion de la rejilla de distancias de choque precalculadas
			float   robotMax_V_mps;				// Max. vel del robot en m/s
			float   robotMax_W_degps;              // Max. vel del robot en rad/s
			float	ROBOTMODEL_TAU,ROBOTMODEL_DELAY; // Params for the motor system modelation
			std::vector<float> weights; // length: 6 [0,5]
			float	minObstaclesHeight, maxObstaclesHeight; // The range of "z" coordinates for obstacles to be considered
			float	DIST_TO_TARGET_FOR_SENDING_EVENT;


			unsigned long	nIteration;	//!< The iteration count.
			float			meanExecutionPeriod;	//!< Runtime estimation of execution period of the method.


			/** For sending an alarm (error event) when it seems that we are not approaching toward the target in a while...
			  */
			float			    badNavAlarm_minDistTarget;
			mrpt::system::TTimeStamp	badNavAlarm_lastMinDistTime;
			float			    badNavAlarm_AlarmTimeout;


			/** The robot 2D shape model
			  */
			math::CPolygon		robotShape;
			bool				collisionGridsMustBeUpdated;


			/** @name Variables for CReactiveNavigationSystem::performNavigationStep
			    @{ */
			mrpt::utils::CTicTac				totalExecutionTime, executionTime, tictac;
			std::vector<vector_double>			TP_Obstacles;
			std::vector<poses::CPoint2D,Eigen::aligned_allocator<poses::CPoint2D> >			TP_Targets;		// Target location (x,y) in TP-Space
			std::vector<THolonomicMovement>		holonomicMovements;
			std::vector<float>					times_TP_transformations, times_HoloNav;
			std::vector<bool>					valid_TP;
			float								meanExecutionTime;
			float								meanTotalExecutionTime;
			int									nLastSelectedPTG;
			int                                 m_decimateHeadingEstimate;
			/** @} */

			/** The set of transformations to be used:
			  */
			std::vector<CParameterizedTrajectoryGenerator*>	PTGs;


			// Steps for the reactive navigation sytem.
			// ----------------------------------------------------------------------------
			void            STEP1_CollisionGridsBuilder();

			bool            STEP2_Sense(
									mrpt::slam::CSimplePointsMap					&out_obstacles);

			void            STEP3_SpaceTransformer(
									poses::CPointsMap					&in_obstacles,
									CParameterizedTrajectoryGenerator	*in_PTG,
									vector_double						&out_TPObstacles);

			void            STEP4_HolonomicMethod(
									vector_double						&in_Obstacles,
									poses::CPoint2D						&in_Target,
									float								in_maxRobotSpeed,
									THolonomicMovement					&out_selectedMovement,
									CHolonomicLogFileRecordPtr			&in_HLFR );

			void            STEP5_Evaluator(
									THolonomicMovement					&in_holonomicMovement,
									vector_double						&in_TPObstacles,
									poses::CPoint2D						&WS_Target,
									poses::CPoint2D						&TP_Target,
									bool								wasSelectedInLast,
									CLogFileRecord::TInfoPerPTG			&log );

			void            STEP6_Selector(
									std::vector<THolonomicMovement>		&in_holonomicMovements,
									THolonomicMovement					&out_selectedHolonomicMovement,
									int									&out_nSelectedPTG);

			void			STEP7_NonHolonomicMovement(
									THolonomicMovement					&in_movement,
									float								&out_v,
									float								&out_w);

			//
			bool			m_closing_navigator;

			/** Stops the robot and set navigation state to error */
			void			doEmergencyStop( const char *msg );

		};
	}
}


#endif





