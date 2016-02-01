/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2016, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */
#ifndef CReactiveNavigationSystem_H
#define CReactiveNavigationSystem_H

#include "CAbstractPTGBasedReactive.h"

namespace mrpt
{
  namespace nav
  {
	/** \defgroup nav_reactive Reactive navigation classes
	  * \ingroup mrpt_nav_grp
	  */
	  
		/** See base class CAbstractPTGBasedReactive for a description and instructions of use.
		* This particular implementation assumes a 2D robot shape.
		*
		* Publications:
		*  - Blanco, Jose-Luis, Javier Gonzalez, and Juan-Antonio Fernandez-Madrigal. "[Extending obstacle avoidance methods through multiple parameter-space transformations](http://citeseerx.ist.psu.edu/viewdoc/download?doi=10.1.1.190.4672&rep=rep1&type=pdf)." Autonomous Robots 24.1 (2008): 29-48.
		*
		* Class history:
		* - 17/JUN/2004: First design.
		* - 16/SEP/2004: Totally redesigned, according to document "MultiParametric Based Space Transformation for Reactive Navigation"
		* - 29/SEP/2005: Totally rewritten again, for integration into MRPT library and according to the ICRA paper.
		* - 17/OCT/2007: Whole code updated to accomodate to MRPT 0.5 and make it portable to Linux.
		* - DEC/2013: Code refactoring between this class and CAbstractHolonomicReactiveMethod
		*
		* This class requires a number of parameters which are usually provided via an external config (".ini") file.
		* Alternatively, a memory-only object can be used to avoid physical files, see mrpt::utils::CConfigFileMemory.
		*
		* Next we provide a self-documented template config file: 
		* \verbinclude reactive2d_config.ini
		*
		*  \sa CAbstractReactiveNavigationSystem, CParameterizedTrajectoryGenerator, CAbstractHolonomicReactiveMethod
		*  \ingroup nav_reactive
		*/
		class NAV_IMPEXP  CReactiveNavigationSystem : public CAbstractPTGBasedReactive
		{
		public:
			MRPT_MAKE_ALIGNED_OPERATOR_NEW
		public:
			/** See docs in ctor of base class */
			CReactiveNavigationSystem(
				CReactiveInterfaceImplementation &react_iterf_impl,
				bool enableConsoleOutput = true,
				bool enableLogFile = false);

			/** Destructor
			 */
			virtual ~CReactiveNavigationSystem();

			/** Reload the configuration from a file. See details in CReactiveNavigationSystem docs. 
			 * \param[in] ini The main source of configuration parameters.
			 * \param[in] robotIni Deprecated (kept for backwards compatibility). It is recommended to use the newer loadConfigFile() method with one argument in new code.
			 */
			void loadConfigFile(const mrpt::utils::CConfigFileBase &ini, const mrpt::utils::CConfigFileBase &robotIni);

			/** Reload the configuration from a file. See details in CReactiveNavigationSystem docs. */
			void loadConfigFile(const mrpt::utils::CConfigFileBase &ini);

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

			/** The set of transformations to be used:
			  */
			std::vector<CParameterizedTrajectoryGenerator*>	PTGs;

			// Steps for the reactive navigation sytem.
			// ----------------------------------------------------------------------------
			virtual void STEP1_CollisionGridsBuilder();

			// See docs in parent class
			virtual bool STEP2_SenseObstacles();

			// See docs in parent class
			virtual void STEP3_WSpaceToTPSpace(const size_t ptg_idx,std::vector<float> &out_TPObstacles);

			/** Generates a pointcloud of obstacles, and the robot shape, to be saved in the logging record for the current timestep */
			virtual void loggingGetWSObstaclesAndShape(CLogFileRecord &out_log);


			mrpt::maps::CSimplePointsMap m_WS_Obstacles;  //!< The obstacle points, as seen from the local robot frame.

		}; // end class
	}
}


#endif





