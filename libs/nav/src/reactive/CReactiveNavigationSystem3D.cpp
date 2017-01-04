/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include "nav-precomp.h" // Precomp header

#include <mrpt/nav/reactive/CReactiveNavigationSystem3D.h>
#include <mrpt/nav/tpspace/CPTG_DiffDrive_CollisionGridBased.h>
#include <mrpt/poses/CPose3D.h>
#include <typeinfo>  // For typeid()

using namespace mrpt;
using namespace mrpt::poses;
using namespace mrpt::math;
using namespace mrpt::utils;
using namespace mrpt::nav;
using namespace std;

// ---------   CReactiveNavigationSystem3D::TPTGmultilevel -----------------
// Ctor:
CReactiveNavigationSystem3D::TPTGmultilevel::TPTGmultilevel()
{
}
// Dtor: free PTG memory
CReactiveNavigationSystem3D::TPTGmultilevel::~TPTGmultilevel()
{
	for (size_t i=0;i<PTGs.size();i++)
		delete PTGs[i];
	PTGs.clear();
}


/*---------------------------------------------------------------
					Constructor
  ---------------------------------------------------------------*/
CReactiveNavigationSystem3D::CReactiveNavigationSystem3D(
	CRobot2NavInterface   &react_iterf_impl,
    bool					enableConsoleOutput,
    bool					enableLogToFile)
	:
	CAbstractPTGBasedReactive(react_iterf_impl,enableConsoleOutput,enableLogToFile)
{
}

// Dtor:
CReactiveNavigationSystem3D::~CReactiveNavigationSystem3D()
{
	this->preDestructor();

	// Free PTGs:
	m_ptgmultilevel.clear();
}


/*---------------------------------------------------------------
						changeRobotShape
  ---------------------------------------------------------------*/
void CReactiveNavigationSystem3D::changeRobotShape( TRobotShape robotShape )
{
	m_PTGsMustBeReInitialized = true;

	for (unsigned int i=0; i<robotShape.size(); i++)
	{
		if ( robotShape.polygon(i).verticesCount() < 3 )
			THROW_EXCEPTION("The robot shape has less than 3 vertices!!")
	}

	m_robotShape = robotShape;
}



void CReactiveNavigationSystem3D::internal_loadConfigFile(const mrpt::utils::CConfigFileBase &ini, const std::string &section_prefix)
{
	MRPT_START

	m_PTGsMustBeReInitialized = true;

	const std::string sectRob = section_prefix + std::string("ROBOT_CONFIG");
	const std::string sectCfg = section_prefix + std::string("ReactiveParams"); // Was: "NAVIGATION_CONFIG". JLB changed this to make it consistent with 2D version and allow refactoring this method.

	// Load config from INI file:
	// ------------------------------------------------------------
	unsigned int num_levels;
	vector <float> xaux,yaux;

	//Read config params which describe the robot shape
	num_levels = ini.read_int(sectRob,"HEIGHT_LEVELS", 1, true);
	m_robotShape.resize(num_levels);
	for (unsigned int i=1;i<=num_levels;i++)
	{
		m_robotShape.setHeight(i-1, ini.read_float(sectRob,format("LEVEL%d_HEIGHT",i), 1.0, true) );
		m_robotShape.setRadius(i-1, ini.read_float(sectRob,format("LEVEL%d_RADIUS",i), 0.5, false) );
		ini.read_vector(sectRob,format("LEVEL%d_VECTORX",i), vector<float> (0), xaux, false);
		ini.read_vector(sectRob,format("LEVEL%d_VECTORY",i), vector<float> (0), yaux, false);
		ASSERT_(xaux.size() == yaux.size());
		for (unsigned int j=0;j<xaux.size();j++)
			m_robotShape.polygon(i-1).AddVertex(xaux[j], yaux[j]);
	}

	// Load PTGs from file:
	// ---------------------------------------------
	// levels = m_robotShape.heights.size()

	unsigned int num_ptgs = ini.read_int(sectCfg,"PTG_COUNT", 1, true);
	m_ptgmultilevel.resize(num_ptgs);

	// Read each PTG parameters, and generate K x N collisiongrids
	//	K - Number of PTGs
	//	N - Number of height sections
	for (unsigned int j=1; j<=num_ptgs; j++)
	{
		for (unsigned int i=1; i<=m_robotShape.size(); i++)
		{
			MRPT_LOG_INFO_FMT("[loadConfigFile] Generating PTG#%u at level %u...",j,i);
			const std::string sPTGName = ini.read_string(sectCfg,format("PTG%d_TYPE",j),"",true);
			CParameterizedTrajectoryGenerator *ptgaux = CParameterizedTrajectoryGenerator::CreatePTG(sPTGName,ini,sectCfg,format("PTG%d_",j));
			m_ptgmultilevel[j-1].PTGs.push_back(ptgaux);
		}
	}

	MRPT_LOG_DEBUG_FMT(" Robot height sections = %u\n", static_cast<unsigned int>(m_robotShape.size()) );

	//this->STEP1_InitPTGs();

	MRPT_END
}

void CReactiveNavigationSystem3D::STEP1_InitPTGs()
{
	if (m_PTGsMustBeReInitialized)
	{
		m_PTGsMustBeReInitialized = false;

		mrpt::utils::CTimeLoggerEntry tle(m_timelogger, "STEP1_InitPTGs");

		for (unsigned int j=0; j<m_ptgmultilevel.size(); j++)
		{
			for (unsigned int i=0; i<m_robotShape.size(); i++)
			{
				m_ptgmultilevel[j].PTGs[i]->deinitialize();

				MRPT_LOG_INFO_FMT("[loadConfigFile] Initializing PTG#%u.%u... (`%s`)", j,i,m_ptgmultilevel[j].PTGs[i]->getDescription().c_str());

				// Polygonal robot shape?
				{
					mrpt::nav::CPTG_RobotShape_Polygonal *ptg = dynamic_cast<mrpt::nav::CPTG_RobotShape_Polygonal *>(m_ptgmultilevel[j].PTGs[i]);
					if (ptg)
						ptg->setRobotShape(m_robotShape.polygon(i));
				}
				// Circular robot shape?
				{
					mrpt::nav::CPTG_RobotShape_Circular *ptg = dynamic_cast<mrpt::nav::CPTG_RobotShape_Circular*>(m_ptgmultilevel[j].PTGs[i]);
					if (ptg)
						ptg->setRobotShapeRadius(m_robotShape.getRadius(i));
				}

				m_ptgmultilevel[j].PTGs[i]->initialize(
					format("%s/ReacNavGrid_%s_%03u_L%02u.dat.gz", ptg_cache_files_directory.c_str(), robotName.c_str(), i, j),
					m_enableConsoleOutput /*verbose*/
				);
				MRPT_LOG_INFO("...Done.");
			}
		}
	}
}

/*************************************************************************

                         STEP2_SortObstacles

		Load the obstacles and sort them accorging to the height
				sections used to model the robot.

*************************************************************************/
bool CReactiveNavigationSystem3D::implementSenseObstacles(mrpt::system::TTimeStamp &obstacles_timestamp)
{
	//-------------------------------------------------------------------
	// The user must implement its own method to load the obstacles from
	// either sensor measurements or simulators (m_robot.senseObstacles(...))
	// Data have to be subsequently sorted in height bands according to the
	// height sections of the robot.
	//-------------------------------------------------------------------

	m_timelogger.enter("navigationStep.STEP2_LoadAndSortObstacle");

	{
		CTimeLoggerEntry tle(m_timlog_delays, "senseObstacles()");
		if (!m_robot.senseObstacles(m_WS_Obstacles_unsorted, obstacles_timestamp))
			return false;
	}

	// Empty slice maps:
	const size_t nSlices = m_robotShape.size();
	m_WS_Obstacles_inlevels.resize(nSlices);
	for (size_t i=0;i<nSlices;i++)
		m_WS_Obstacles_inlevels[i].clear();


	// Sort obstacles in "slices":
	size_t nPts;
	const float *xs,*ys,*zs;
	m_WS_Obstacles_unsorted.getPointsBuffer(nPts,xs,ys,zs);
	const float OBS_MAX_XY = this->refDistance*1.1f;

	for (size_t j=0; j<nPts; j++)
	{
		float h = 0;
		for (size_t idxH=0;idxH<nSlices;++idxH)
		{
			if (zs[j] < 0.01)
				break; // skip this points

			h += m_robotShape.getHeight(idxH);
			if (zs[j] < h)
			{
				// Speed-up: If the obstacle is, for sure, out of the collision grid,
				// just don't account for it, because we don't know its mapping into TP-Obstacles anyway...
				if (xs[j]>-OBS_MAX_XY && xs[j]<OBS_MAX_XY && ys[j]>-OBS_MAX_XY && ys[j]<OBS_MAX_XY)
					m_WS_Obstacles_inlevels[idxH].insertPoint(xs[j],ys[j],zs[j]);

				break; // stop searching for height slots.
			}
		}
	}

	m_timelogger.leave("navigationStep.STEP2_LoadAndSortObstacle");

	return true;
}

/*************************************************************************
		Transform the obstacle into TP-Obstacles in TP-Spaces
*************************************************************************/
void CReactiveNavigationSystem3D::STEP3_WSpaceToTPSpace(
	const size_t ptg_idx,
	std::vector<double> &out_TPObstacles,
	mrpt::nav::ClearanceDiagram &out_clearance,
	const mrpt::poses::CPose2D &rel_pose_PTG_origin_wrt_sense)
{
	ASSERT_EQUAL_(m_WS_Obstacles_inlevels.size(),m_robotShape.size())

	for (size_t j=0;j<m_robotShape.size();j++)
	{
		size_t nObs;
		const float *xs,*ys,*zs;
		m_WS_Obstacles_inlevels[j].getPointsBuffer(nObs,xs,ys,zs);

		for (size_t obs=0;obs<nObs;obs++)
		{
			double ox, oy;
			rel_pose_PTG_origin_wrt_sense.composePoint(xs[obs], ys[obs], ox, oy);
			m_ptgmultilevel[ptg_idx].PTGs[j]->updateTPObstacle(ox, oy, out_TPObstacles);
			m_ptgmultilevel[ptg_idx].PTGs[j]->updateClearance(ox, oy, out_clearance);

		}
	}

	// Distances in TP-Space are normalized to [0,1]
	// They'll be normalized in the base abstract class:
}


/** Generates a pointcloud of obstacles to be saved in the logging record for the current timestep */
void CReactiveNavigationSystem3D::loggingGetWSObstaclesAndShape(CLogFileRecord &out_log)
{

	out_log.WS_Obstacles.clear();
	//Include the points of all levels (this could be improved depending on STEP2)
	for (unsigned int i=0; i < m_WS_Obstacles_inlevels.size(); i++)
		out_log.WS_Obstacles.insertAnotherMap( &m_WS_Obstacles_inlevels[i], CPose3D(0,0,0));

	//Polygons of each height level are drawn (but they are all shown connected...)
	if (out_log.robotShape_x.size() == 0)
	{
		size_t nVerts = 0;
		TPoint2D paux;
		size_t cuenta = 0;
		for (unsigned int i=0; i < m_robotShape.size(); i++)
			nVerts += m_robotShape.polygon(i).size() + 1;
		if (size_t(out_log.robotShape_x.size()) != nVerts)
		{
			out_log.robotShape_x.resize(nVerts);
			out_log.robotShape_y.resize(nVerts);
		}
		for (unsigned int i=0; i<m_robotShape.size(); i++)
		{
			for (unsigned int j=0; j<m_robotShape.polygon(i).size(); j++)
			{
				paux = m_robotShape.polygon(i)[j];
				out_log.robotShape_x[cuenta]= paux.x;
				out_log.robotShape_y[cuenta]= paux.y;
				cuenta++;
			}
			paux = m_robotShape.polygon(i)[0];
			out_log.robotShape_x[cuenta]= paux.x;
			out_log.robotShape_y[cuenta]= paux.y;
			cuenta++;
		}
	}
	out_log.robotShape_radius = m_robotShape.getRadius(0);
}
