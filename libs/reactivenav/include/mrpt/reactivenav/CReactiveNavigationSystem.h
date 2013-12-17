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
#ifndef CReactiveNavigationSystem_H
#define CReactiveNavigationSystem_H

#include "CAbstractPTGBasedReactive.h"

namespace mrpt
{
	/** This namespace contains classes for building a TP-Space Reactive Navigation System.
	*/
  namespace reactivenav
  {
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
		* - DEC/2013: Code refactoring between this class and CAbstractHolonomicReactiveMethod
		*
		*  \sa CAbstractReactiveNavigationSystem, CParameterizedTrajectoryGenerator, CAbstractHolonomicReactiveMethod
		*  \ingroup mrpt_reactivenav_grp
		*/
		class REACTIVENAV_IMPEXP  CReactiveNavigationSystem : public CAbstractPTGBasedReactive
		{
		public:
			EIGEN_MAKE_ALIGNED_OPERATOR_NEW
		public:
			/** See docs in ctor of base class */
			CReactiveNavigationSystem(
				CReactiveInterfaceImplementation &react_iterf_impl,
				bool enableConsoleOutput = true,
				bool enableLogFile = false);

			/** Destructor
			 */
			virtual ~CReactiveNavigationSystem();

			/** Reload the configuration from a file
			 */
			void loadConfigFile(const mrpt::utils::CConfigFileBase &ini, const mrpt::utils::CConfigFileBase &robotIni);

			/** Change the robot shape, which is taken into account for collision
			  *  grid building.
			  */
			void changeRobotShape( const math::CPolygon &shape );

			/** Returns the number of different PTGs that have been setup */
			virtual size_t getPTG_count() const { return PTGs.size(); } 

			/** Gets the i'th PTG */
			virtual CParameterizedTrajectoryGenerator* getPTG(size_t i) 
			{
				ASSERT_(i<PTGs.size())
				return PTGs[i];
			}


		private:
			// ------------------------------------------------------
			//					PRIVATE	VARIABLES
			// ------------------------------------------------------
			float	minObstaclesHeight, maxObstaclesHeight; // The range of "z" coordinates for obstacles to be considered

			/** The robot 2D shape model */
			math::CPolygon		m_robotShape;

			/** @name Variables for CReactiveNavigationSystem::performNavigationStep
			    @{ */
			std::vector<vector_double>			TP_Obstacles;
			std::vector<math::TPoint2D>			TP_Targets;		// Target location (x,y) in TP-Space
			std::vector<THolonomicMovement>		holonomicMovements;
			std::vector<float>					times_TP_transformations, times_HoloNav;
			std::vector<bool>					valid_TP;
			int									nLastSelectedPTG;
			int                                 m_decimateHeadingEstimate;
			/** @} */

			/** The set of transformations to be used:
			  */
			std::vector<CParameterizedTrajectoryGenerator*>	PTGs;

			// Steps for the reactive navigation sytem.
			// ----------------------------------------------------------------------------
			virtual void STEP1_CollisionGridsBuilder();
		
			/** Return false on any fatal error */
			virtual bool STEP2_SenseObstacles();

			virtual void STEP3_WSpaceToTPSpace(const mrpt::poses::CPose2D &relTarget,vector_double &out_TPObstacles);

			virtual void STEP4_HolonomicMethod();

			mrpt::slam::CSimplePointsMap m_WS_Obstacles;

		};
	}
}


#endif





