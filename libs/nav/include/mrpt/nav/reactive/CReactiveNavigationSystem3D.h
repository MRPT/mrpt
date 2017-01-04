/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
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
		/** A 3D robot shape stored as a "sliced" stack of 2D polygons, used for CReactiveNavigationSystem3D 
		  * Depending on each PTG, only the 2D polygon or the circular radius will be taken into account
		  *  \ingroup nav_reactive
		  */
		struct NAV_IMPEXP TRobotShape 
		{
			size_t size() const { return polygons.size(); }
			void resize(size_t num_levels)
			{ 
				polygons.resize(num_levels); 
				radius.resize(num_levels);
				heights.resize(num_levels);
			}

			const mrpt::math::CPolygon & polygon(size_t level) const { return polygons[level]; }
			const double & getRadius(size_t level) const { return radius[level]; }
			const double & getHeight(size_t level) const { return heights[level]; }
			mrpt::math::CPolygon & polygon(size_t level) { return polygons[level]; }
			void setRadius(size_t level, double r) { radius[level]=r; }
			void setHeight(size_t level, double h) { heights[level]=h; }
			
			const std::vector<double> &getHeights() const { return heights; }
		private:
			std::vector<mrpt::math::CPolygon> polygons;  // The polygonal bases
			std::vector<double>               radius;    // The radius of each prism.
			std::vector<double>               heights;   // Heights of the prisms
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
		*  \sa CAbstractNavigator, CParameterizedTrajectoryGenerator, CAbstractHolonomicReactiveMethod
		*  \ingroup nav_reactive
		*/
		class NAV_IMPEXP CReactiveNavigationSystem3D : public CAbstractPTGBasedReactive
		{
		public:
			MRPT_MAKE_ALIGNED_OPERATOR_NEW
		public:
			/** See docs in ctor of base class */
			CReactiveNavigationSystem3D(
				CRobot2NavInterface &react_iterf_impl,
				bool enableConsoleOutput = true,
				bool enableLogFile = false);

			/** Destructor */
			virtual ~CReactiveNavigationSystem3D();

			/** Change the robot shape, which is taken into account for collision grid building. */
			void changeRobotShape( TRobotShape robotShape );

			// See base class docs:
			virtual size_t getPTG_count() const  MRPT_OVERRIDE { ASSERT_(!m_ptgmultilevel.empty());  return m_ptgmultilevel.begin()->PTGs.size(); }
			virtual CParameterizedTrajectoryGenerator* getPTG(size_t i)  MRPT_OVERRIDE {
				ASSERT_(!m_ptgmultilevel.empty() && !m_ptgmultilevel[0].PTGs.empty())
				return m_ptgmultilevel[0].PTGs[i];  // Return for the 0'th level (ptgs are replicated at each level)
			}
			virtual const CParameterizedTrajectoryGenerator* getPTG(size_t i) const  MRPT_OVERRIDE {
				ASSERT_(!m_ptgmultilevel.empty() && !m_ptgmultilevel[0].PTGs.empty())
				return m_ptgmultilevel[0].PTGs[i];  // Return for the 0'th level (ptgs are replicated at each level)
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
			virtual void STEP1_InitPTGs() MRPT_OVERRIDE;

			// See docs in parent class
			bool implementSenseObstacles(mrpt::system::TTimeStamp &obs_timestamp) MRPT_OVERRIDE;

			// See docs in parent class
			void STEP3_WSpaceToTPSpace(const size_t ptg_idx,std::vector<double> &out_TPObstacles, mrpt::nav::ClearanceDiagram &out_clearance, const mrpt::poses::CPose2D &rel_pose_PTG_origin_wrt_sense) MRPT_OVERRIDE;

			/** Generates a pointcloud of obstacles, and the robot shape, to be saved in the logging record for the current timestep */
			virtual void loggingGetWSObstaclesAndShape(CLogFileRecord &out_log) MRPT_OVERRIDE;

		protected:
			void internal_loadConfigFile(const mrpt::utils::CConfigFileBase &ini, const std::string &section_prefix="") MRPT_OVERRIDE;

		}; // end class
	}
}


#endif





