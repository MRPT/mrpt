/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
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
		* This particular implementation assumes a 2D robot shape which can be polygonal or circular (depending on the selected PTGs).
		*
		* Publications:
		*  - Blanco, Jose-Luis, Javier Gonzalez, and Juan-Antonio Fernandez-Madrigal. ["Extending obstacle avoidance methods through multiple parameter-space transformations"](http://citeseerx.ist.psu.edu/viewdoc/download?doi=10.1.1.190.4672&rep=rep1&type=pdf). Autonomous Robots 24.1 (2008): 29-48.
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
		*  \sa CAbstractNavigator, CParameterizedTrajectoryGenerator, CAbstractHolonomicReactiveMethod
		*  \ingroup nav_reactive
		*/
		class NAV_IMPEXP  CReactiveNavigationSystem : public CAbstractPTGBasedReactive
		{
		public:
			MRPT_MAKE_ALIGNED_OPERATOR_NEW
		public:
			/** See docs in ctor of base class */
			CReactiveNavigationSystem(
				CRobot2NavInterface &react_iterf_impl,
				bool enableConsoleOutput = true,
				bool enableLogFile = false);

			/** Destructor
			 */
			virtual ~CReactiveNavigationSystem();

			/** Defines the 2D polygonal robot shape, used for some PTGs for collision checking. */
			void changeRobotShape( const mrpt::math::CPolygon &shape );
			/** Defines the 2D circular robot shape radius, used for some PTGs for collision checking. */
			void changeRobotCircularShapeRadius( const double R );

			// See base class docs:
			virtual size_t getPTG_count() const  MRPT_OVERRIDE { return PTGs.size(); }
			virtual CParameterizedTrajectoryGenerator* getPTG(size_t i)  MRPT_OVERRIDE { ASSERT_(i<PTGs.size()); return PTGs[i]; }
			virtual const CParameterizedTrajectoryGenerator* getPTG(size_t i) const  MRPT_OVERRIDE { ASSERT_(i<PTGs.size()); return PTGs[i]; }

		protected:
			float	minObstaclesHeight, maxObstaclesHeight; // The range of "z" coordinates for obstacles to be considered

			math::CPolygon m_robotShape;               //!< The robot 2D shape model. Only one of `m_robotShape` or `m_robotShape` will be used in each PTG
			double         m_robotShapeCircularRadius; //!< Radius of the robot if approximated as a circle. Only one of `m_robotShape` or `m_robotShape` will be used in each PTG

		private:
			std::vector<CParameterizedTrajectoryGenerator*>	PTGs;  //!< The list of PTGs to use for navigation

			// Steps for the reactive navigation sytem.
			// ----------------------------------------------------------------------------
			virtual void STEP1_InitPTGs() MRPT_OVERRIDE;

			// See docs in parent class
			bool implementSenseObstacles(mrpt::system::TTimeStamp &obs_timestamp) MRPT_OVERRIDE;
		protected:
			/** Generates a pointcloud of obstacles, and the robot shape, to be saved in the logging record for the current timestep */
			virtual void loggingGetWSObstaclesAndShape(CLogFileRecord &out_log) MRPT_OVERRIDE;

		
			void internal_loadConfigFile(const mrpt::utils::CConfigFileBase &ini, const std::string &section_prefix="") MRPT_OVERRIDE;
			mrpt::maps::CSimplePointsMap m_WS_Obstacles;  //!< The obstacle points, as seen from the local robot frame.
			// See docs in parent class
			void STEP3_WSpaceToTPSpace(const size_t ptg_idx, std::vector<double> &out_TPObstacles, mrpt::nav::ClearanceDiagram &out_clearance, const mrpt::poses::CPose2D &rel_pose_PTG_origin_wrt_sense) MRPT_OVERRIDE;

		}; // end class
	}
}


#endif





