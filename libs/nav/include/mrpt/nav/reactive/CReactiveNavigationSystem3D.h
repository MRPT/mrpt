/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2016, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */
#ifndef CReactiveNavigationSystem3D_H
#define CReactiveNavigationSystem3D_H

#include "CAbstractPTGBasedReactive.h"

namespace mrpt
{
  namespace nav
  {
		/** A 3D robot shape stored as a "sliced" stack of 2D polygons, used for CReactiveNavigationSystem3D */
		struct TRobotShape {
				std::vector<math::CPolygon>	polygons;		// The polygonal bases
				std::vector<float>			heights;		// Heights of the prisms
		};
		
		/** See base class CAbstractPTGBasedReactive for a description and instructions of use.
		* This particular implementation assumes a 3D (or "2.5D") robot shape model, build as a vertical stack of "2D slices".
		*
		*  Paper describing the method:
		*  - M. Jaimez-Tarifa, J. Gonzalez-Jimenez, J.L. Blanco, 
		*    "Efficient Reactive Navigation with Exact Collision Determination for 3D Robot Shapes", 
		*     International Journal of Advanced Robotic Systems, 2015.
		*
		* Class history:
		* - SEP/2012: First design.
		* - JUL/2013: Integrated into MRPT library.
		* - DEC/2013: Code refactoring between this class and CAbstractHolonomicReactiveMethod
		*
		* This class requires a number of parameters which are usually provided via an external config (".ini") file.
		* Alternatively, a memory-only object can be used to avoid physical files, see mrpt::utils::CConfigFileMemory.
		*
		* Next we provide a self-documented template config file: 
		* \verbinclude reactive3d_config.ini
		*
		*  \sa CAbstractReactiveNavigationSystem, CParameterizedTrajectoryGenerator, CAbstractHolonomicReactiveMethod
		*  \ingroup nav_reactive
		*/
		class NAV_IMPEXP CReactiveNavigationSystem3D : public CAbstractPTGBasedReactive
		{
		public:
			MRPT_MAKE_ALIGNED_OPERATOR_NEW
		public:
			/** See docs in ctor of base class */
			CReactiveNavigationSystem3D(
				CReactiveInterfaceImplementation &react_iterf_impl,
				bool enableConsoleOutput = true,
				bool enableLogFile = false);

			/** Destructor */
			virtual ~CReactiveNavigationSystem3D();

			/** Reload the configuration from a file. See CReactiveNavigationSystem3D for details. */
			void loadConfigFile(const mrpt::utils::CConfigFileBase &ini);

			/** Change the robot shape, which is taken into account for collision grid building. */
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
			struct NAV_IMPEXP TPTGmultilevel
			{
				std::vector<CParameterizedTrajectoryGenerator*> PTGs;
				mrpt::math::TPoint2D TP_Target;
				THolonomicMovement   holonomicmov;

				TPTGmultilevel();
				~TPTGmultilevel();
			};

			// ------------------------------------------------------
			//					PRIVATE	VARIABLES
			// ------------------------------------------------------
			mrpt::maps::CSimplePointsMap              m_WS_Obstacles_unsorted;  //!< The unsorted set of obstacles from the sensors
			std::vector<mrpt::maps::CSimplePointsMap> m_WS_Obstacles_inlevels; //!< One point cloud per 2.5D robot-shape-slice, coordinates relative to the robot local frame


			/** The robot 3D shape model */
			TRobotShape		m_robotShape;

			/** The set of PTG-transformations to be used: */
			std::vector <TPTGmultilevel>	m_ptgmultilevel;


			// Steps for the reactive navigation sytem.
			// ----------------------------------------------------------------------------
			virtual void STEP1_CollisionGridsBuilder();

			// See docs in parent class
			virtual bool STEP2_SenseObstacles();

			// See docs in parent class
			virtual void STEP3_WSpaceToTPSpace(const size_t ptg_idx,std::vector<float> &out_TPObstacles);

			/** Generates a pointcloud of obstacles, and the robot shape, to be saved in the logging record for the current timestep */
			virtual void loggingGetWSObstaclesAndShape(CLogFileRecord &out_log);



		}; // end class
	}
}


#endif





