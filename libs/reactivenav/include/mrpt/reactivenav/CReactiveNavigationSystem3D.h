/* +---------------------------------------------------------------------------+
   |                 The Mobile Robot Programming Toolkit (MRPT)               |
   |                                                                           |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2013, Individual contributors, see AUTHORS file        |
   | Copyright (c) 2005-2013, MAPIR group, University of Malaga                |
   | Copyright (c) 2012-2013, University of Almeria                            |
   | All rights reserved.                                                      |
   |                                                                           |
   | Redistribution and use in source and binary forms, with or without        |
   | modification, are permitted provided that the following conditions are    |
   | met:                                                                      |
   |    * Redistributions of source code must retain the above copyright       |
   |      notice, this list of conditions and the following disclaimer.        |
   |    * Redistributions in binary form must reproduce the above copyright    |
   |      notice, this list of conditions and the following disclaimer in the  |
   |      documentation and/or other materials provided with the distribution. |
   |    * Neither the name of the copyright holders nor the                    |
   |      names of its contributors may be used to endorse or promote products |
   |      derived from this software without specific prior written permission.|
   |                                                                           |
   | THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS       |
   | 'AS IS' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED |
   | TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR|
   | PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDERS BE LIABLE |
   | FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL|
   | DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR|
   |  SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)       |
   | HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,       |
   | STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN  |
   | ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE           |
   | POSSIBILITY OF SUCH DAMAGE.                                               |
   +---------------------------------------------------------------------------+ */
#ifndef CReactiveNavigationSystem3D_H
#define CReactiveNavigationSystem3D_H

#include <mrpt/maps.h>
#include <mrpt/poses.h>
#include <mrpt/math.h>
#include <mrpt/synch.h>
#include <mrpt/utils/CTimeLogger.h>


#include "CAbstractPTGBasedReactive.h"

namespace mrpt
{
	/** This namespace contains classes for building a 3D TP-Space Reactive Navigation System.
	*/
  namespace reactivenav
  {
	  /**  The implemented reactive navigation methods
	    *  \ingroup mrpt_reactivenav_grp
	    */

		/** Implements a 3D reactive navigation system based on TP-Space, with an arbitrary holonomic
		*  reactive method running on it, and any desired number of PTG for transforming the navigation space.
		*  Both, the holonomic method and the PTGs can be customized by the apropriate user derived classes.
		*
		*   How to use:
		*      - A class with callbacks must be defined by the user and provided to the constructor.
		*      - loadConfigFile() must be called to set up the bunch of parameters from a config file (could be a memory-based virtual config file).
		*      - navigationStep() must be called periodically in order to effectively run the navigation. This method will internally call the callbacks to gather sensor data and robot positioning data.
		*
		* - SEP/2012: First design.
		* - JUL/2013: Integrated into MRPT library.
		* - DEC/2013: Code refactoring between this class and CAbstractHolonomicReactiveMethod
		*
		*  \sa CAbstractReactiveNavigationSystem, CParameterizedTrajectoryGenerator, CAbstractHolonomicReactiveMethod
		*  \ingroup mrpt_reactivenav_grp
		*/

		/**Struct used to store the 3D robot shape
		  */
		struct TRobotShape {
				std::vector<math::CPolygon>	polygons;		// The polygonal bases
				std::vector<float>			heights;		// Heights of the prisms
		};


		class REACTIVENAV_IMPEXP CReactiveNavigationSystem3D : public CAbstractPTGBasedReactive
		{
		public:
			EIGEN_MAKE_ALIGNED_OPERATOR_NEW
		public:
			/** See docs in ctor of base class */
			CReactiveNavigationSystem3D(
				CReactiveInterfaceImplementation &react_iterf_impl,
				bool enableConsoleOutput = true,
				bool enableLogFile = false);

			/** Destructor
			 */
			virtual ~CReactiveNavigationSystem3D();

			/** Reload the configuration from a file
			 */
			void loadConfigFile(const mrpt::utils::CConfigFileBase &ini);

			/** Change the robot shape, which is taken into account for collision
			  *  grid building.
			  */
			void changeRobotShape( TRobotShape robotShape );

			/** Returns the number of different PTGs that have been setup */
			virtual size_t getPTG_count() const { return m_ptgmultilevel.size(); } 

			/** Gets the i'th PTG */
			virtual CParameterizedTrajectoryGenerator* getPTG(size_t i) 
			{
				ASSERT_(i<m_ptgmultilevel.size() && !m_ptgmultilevel[i].PTGs.empty())
				return m_ptgmultilevel[i].PTGs[0];  // Return the 0'th because the PTG itself is the same, what changes is the collision grid.
			}

		private:
			// ------------------------------------------------------
			//					PRIVATE DEFINITIONS
			// ------------------------------------------------------

			/** A set of PTGs of the same type, one per "height level" */
			struct REACTIVENAV_IMPEXP TPTGmultilevel
			{
				std::vector <CParameterizedTrajectoryGenerator*> PTGs;
				mrpt::vector_double		TPObstacles;
				TPoint2D				TP_Target;
				THolonomicMovement		holonomicmov;

				TPTGmultilevel();
				~TPTGmultilevel();
			};

			// ------------------------------------------------------
			//					PRIVATE	VARIABLES
			// ------------------------------------------------------

			/** The unsorted set of obstacles
			  */
			mrpt::slam::CSimplePointsMapPtr		WS_Obstacles_unsorted;

			vector <CAbstractHolonomicReactiveMethod*>  holonomicMethod;   //!< The holonomic navigation algorithm.
			mrpt::utils::CStream						*logFile;           //!< The current log file stream, or NULL if not being used
			unsigned int								holonomicMethodSel; //(0 - VFF, 1 - ND)

			bool  m_enableConsoleOutput;  //!< Enables / disables the console debug output.

			bool  m_init_done;            //!< Whether \a loadConfigFile() has been called or not.

			CTicTac	timerForExecutionPeriod;

			// PTG params loaded from INI file:
			std::string robotName;						// Robot name
			float		refDistance;					// "dmax" in papers.
			float		colGridRes;						// CollisionGrid resolution
			float		robotMax_V_mps;					// Max. linear speed (m/s)
			float		robotMax_W_degps;				// Max. angular speed (deg/s)
			float		SPEEDFILTER_TAU;				// Time constant for the low-pass filter applied to the speed commands
			std::vector<float> weights;					// length: 6 [0,5]
			float		DIST_TO_TARGET_FOR_SENDING_EVENT;


			float						meanExecutionPeriod;	//!< Runtime estimation of execution period of the method.
			mrpt::utils::CTimeLogger	m_timelogger;			//!< A complete time logger \sa enableTimeLog()


			/** For sending an alarm (error event) when it seems that we are not approaching toward the target in a while...
			  */
			float						badNavAlarm_minDistTarget;
			mrpt::system::TTimeStamp	badNavAlarm_lastMinDistTime;
			float						badNavAlarm_AlarmTimeout;


			/** The robot 3D shape model
			  */
			TRobotShape		m_robotShape;


			bool			m_collisionGridsMustBeUpdated;


			/** @name Variables for CReactiveNavigationSystem::performNavigationStep
			    @{ */
			mrpt::utils::CTicTac				totalExecutionTime, executionTime, tictac;
			//std::vector<float>				times_TP_transformations, times_HoloNav;
			float								meanExecutionTime;
			float								meanTotalExecutionTime;
			int                                 m_decimateHeadingEstimate;
			/** @} */

			/** The set of PTG-transformations to be used:
			  */
			std::vector <TPTGmultilevel>	m_ptgmultilevel;


			// Steps for the reactive navigation sytem.
			// ----------------------------------------------------------------------------
			virtual void STEP1_CollisionGridsBuilder();

			bool            STEP2_LoadAndSortObstacles(vector <mrpt::slam::CSimplePointsMap>	&out_obstacles);

			void            STEP3_WSpaceToTPSpace(
									vector <mrpt::slam::CSimplePointsMap>	&in_obstacles,
									CPoint2D								&relTarget);

			void            STEP4_HolonomicMethod(vector <CHolonomicLogFileRecordPtr>  &in_HLFR );

			void            STEP5_PTGEvaluator(
									THolonomicMovement					&in_holonomicMovement,
									vector_double						&in_TPObstacles,
									poses::CPoint2D						&WS_Target,
									poses::CPoint2D						&TP_Target,
									CLogFileRecord::TInfoPerPTG			&log );

			void            STEP6_Selector(
									THolonomicMovement					&out_selectedHolonomicMovement,
									int									&out_nSelectedPTG);

			void			STEP7_GenerateSpeedCommands(THolonomicMovement	&in_movement);


			bool			m_closing_navigator;

			/** Stops the robot and set navigation state to error */
			void			doEmergencyStop( const char *msg );

		};
	}
}


#endif





