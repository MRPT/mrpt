/* +---------------------------------------------------------------------------+
   |          The Mobile Robot Programming Toolkit (MRPT) C++ library          |
   |                                                                           |
   |                   http://mrpt.sourceforge.net/                            |
   |                                                                           |
   |   Copyright (C) 2005-2010  University of Malaga                           |
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
	    */
	  enum THolonomicMethod
	  {
		  hmVIRTUAL_FORCE_FIELDS = 0,
		  hmSEARCH_FOR_BEST_GAP = 1
	  };

	  /** Implements a reactive navigation system based on TP-Space, with an arbitrary holonomic
			reactive method running on it, and any desired number of PTG for transforming the navigation space.
			Both, the holonomic method and the PTGs can be customized by the apropriate user derived classes.
			For running it, the method NavigateStep must be invoked periodically.

				- 17/JUN/2004: First design.
				- 16/SEP/2004: Totally redesigned, according to document "MultiParametric Based Space
                     Transformation for Reactive Navigation"
				- 29/SEP/2005: Totally rewritten again, for integration into MRPT library and according to the ICRA paper.
				- 17/OCT/2007: Whole code updated to accomodate to MRPT 0.5 and make it portable to Linux.

			\sa CAbstractReactiveNavigationSystem, CParameterizedTrajectoryGenerator, CAbstractHolonomicReactiveMethod
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


            /** See 'navigatorBehavior'
              */
            enum TNavigatorBehavior
            {
            	/** The robot tries to get to the given target point
                  */
            	beNormalNavigation = 0,
            	/** The robot rotates to head the direction "m_beHeadDirection_rad", then goes into normal behavior
                  */
            	beHeadDirection,
            	/** The robot tries to get to a given auxiliar target "m_beAuxTarget", then goes into behavior 'beDoorCrosing2'
                  */
                beDoorCrosing1,
            	/** The robot rotates to head the direction "m_beHeadDirection_rad", then goes into behavior 'beDoorCrosing3'
                  */
                beDoorCrosing2,
            	/** The robot rotates tries to get to a given auxiliar target "m_beAuxTarget", then goes into normal behavior
                  */
                beDoorCrosing3
            };

			/** Enables / disables the logging into a file.
			  */
			void enableLogFile(bool enable);

		private:
			// ------------------------------------------------------
			//					PRIVATE DEFINITIONS
			// ------------------------------------------------------
			std::vector<float>		prevV,prevW,prevSelPTG;

            /** This defines the 'behavior' of the navigator (see posible values in TNavigatorBehavior)
              */
            TNavigatorBehavior 		navigatorBehavior;

            /** The desired heading direction (in rads), for behaviors 'beHeadDirection' and 'beDoorCrosing2'.
              */
            float					m_beHeadDirection_rad;

            /** Auxiliary target position, for behaviors 'beDoorCrosing1' and 'beDoorCrosing3'.
              */
            CPoint2D				m_beAuxTarget;

            /**  This is the desired "path" for passing a door (in GLOBAL coordinates!)
              */
            CPoint2D				m_bePassPoint1,m_bePassPoint2;

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

			/** For the histeresis:
			 */
			float		last_cmd_v,last_cmd_w;

			/** Will be false until the navigation end is sent, and it is reset with each new command
			  */
			bool		navigationEndEventSent;

			/** Critical zones:
			  */
            synch::CCriticalSection  m_critZoneLastLog,m_critZoneNavigating;

			// ------------------------------------------------------
			//					PRIVATE METHODS
			// ------------------------------------------------------
			/** The main method for the navigator
			  */
			void  performNavigationStep( );


			// ------------------------------------------------------
			//					PRIVATE	VARIABLES
			// ------------------------------------------------------
			/** The current log file stream, or NULL if not being used
			  */
			CAbstractHolonomicReactiveMethod	*holonomicMethod;
			/** The current log file stream, or NULL if not being used
			  */
			utils::CStream						*logFile;
			/** Enables / disables the console debug output.
			  */
			bool enableConsoleOutput;

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

            float	DOOR_CROSSING_SEARCH_TARGET_DISTANCEx2;
            float	VORONOI_MINIMUM_CLEARANCE;
            float	DISABLE_PERIOD_AFTER_FAIL;
            float	VORONOI_PATH_DIST_FROM_DOORWAY;
            float	DOORCROSSING_HEADING_ACCURACY_DEG;
            float	DOORCROSSING_ROTATION_CTE_DEG;
			float	DOOR_CROSSING_DIST_TO_AUX_TARGETS;
			float	DOOR_CROOSING_BEH3_TIMEOUT;
			float	DOOR_CROSSING_MAXIMUM_DOORWAY_SIZE;

			/** The iterations count.
			  */
			unsigned long	nIteration;

			/** Runtime estimation of execution period of the method.
			  */
			float			meanExecutionPeriod;


			/** For sending an alarm (error event) when it seems that we are not approaching toward the target in a while...
			  */
			float			    badNavAlarm_minDistTarget;
			mrpt::system::TTimeStamp	badNavAlarm_lastMinDistTime;
			float			    badNavAlarm_AlarmTimeout;


			/** The robot 2D shape model
			  */
			math::CPolygon		robotShape;
			bool				collisionGridsMustBeUpdated;

			/** For taken the dynamics of the robot into account.
			  */
			class CDynamicWindow
			{
			public:
				float	v_max, v_min;	//  m/sec
				float	w_max, w_min;	//  rad/sec

			private:
				float	c1,c2,c3,c4;	// Curvature of corners.

			public:

				/** Finds the max/min curvatures in the DW.
				  */
				void  findMinMaxCurvatures(float &minCurv, float &maxCurv);

				/** Returns the corner which is closer (in curvature, and abs. values) to the desired command.
				  */
				void  findBestApproximation(float desV,float desW, float &outV,float &outW);

			private:
				/** Find the closest cut of a line with the DW
				  */
				bool  findClosestCut( float cmd_v, float cmd_w,	// IN
												float &out_v,float &out_w);	// OUT

			};



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
			CDynamicWindow						DW;
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
			bool			CerrandoHilo;

			// Para casos de errores:
			void			Error_ParadaDeEmergencia( const char *msg );

		};
	}
}


#endif





