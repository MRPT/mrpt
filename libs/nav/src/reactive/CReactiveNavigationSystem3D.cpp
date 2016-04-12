/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2016, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include "nav-precomp.h" // Precomp header

#include <mrpt/nav/reactive/CReactiveNavigationSystem3D.h>
#include <mrpt/nav/tpspace/motion_planning_utils.h>
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
	CReactiveInterfaceImplementation   &react_iterf_impl,
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
	m_collisionGridsMustBeUpdated = true;

	for (unsigned int i=0; i<robotShape.polygons.size(); i++)
	{
		if ( robotShape.polygons[i].verticesCount() < 3 )
			THROW_EXCEPTION("The robot shape has less than 3 vertices!!")
	}

	m_robotShape = robotShape;
}



/*---------------------------------------------------------------
						loadConfigFile
  ---------------------------------------------------------------*/
void CReactiveNavigationSystem3D::loadConfigFile(const mrpt::utils::CConfigFileBase &ini)
{
	MRPT_START

	m_collisionGridsMustBeUpdated = true;

	// Load config from INI file:
	// ------------------------------------------------------------
	robotName = ini.read_string("ROBOT_CONFIG","Name", "MyRobot", false );

	unsigned int num_levels;
	vector <float> xaux,yaux;

	//Read config params which describe the robot shape
	num_levels = ini.read_int("ROBOT_CONFIG","HEIGHT_LEVELS", 1, true);
	m_robotShape.polygons.resize(num_levels);
	m_robotShape.heights.resize(num_levels);
	for (unsigned int i=1;i<=num_levels;i++)
	{
		m_robotShape.heights[i-1] = ini.read_float("ROBOT_CONFIG",format("LEVEL%d_HEIGHT",i), 1, true);
		ini.read_vector("ROBOT_CONFIG",format("LEVEL%d_VECTORX",i), vector<float> (0), xaux, false);
		ini.read_vector("ROBOT_CONFIG",format("LEVEL%d_VECTORY",i), vector<float> (0), yaux, false);
		ASSERT_(xaux.size() == yaux.size());
		for (unsigned int j=0;j<xaux.size();j++)
		{
			m_robotShape.polygons[i-1].AddVertex(xaux[j], yaux[j]);
		}
	}

	//Read navigation params
	refDistance = ini.read_float("NAVIGATION_CONFIG","MAX_DISTANCE_PTG", 1, true);
	robotMax_V_mps = ini.read_float("NAVIGATION_CONFIG","VMAX_MPS", 1, true);
	robotMax_W_degps = ini.read_float("NAVIGATION_CONFIG","WMAX_DEGPS", 60, true);
	SPEEDFILTER_TAU =  ini.read_float("NAVIGATION_CONFIG","SPEEDFILTER_TAU", 0, true);

	DIST_TO_TARGET_FOR_SENDING_EVENT = ini.read_float("NAVIGATION_CONFIG", "DIST_TO_TARGET_FOR_SENDING_EVENT", DIST_TO_TARGET_FOR_SENDING_EVENT, false);

	ini.read_vector("NAVIGATION_CONFIG", "weights", vector<float> (0), weights, 1);
	ASSERT_(weights.size()==6);

	badNavAlarm_AlarmTimeout = ini.read_float("NAVIGATION_CONFIG","ALARM_SEEMS_NOT_APPROACHING_TARGET_TIMEOUT", badNavAlarm_AlarmTimeout, false);

	//m_reactiveparam.m_reload_ptgfiles = ini.read_bool("NAVIGATION_CONFIG","RELOAD_PTGFILES", 1, true);


	// Load PTGs from file:
	// ---------------------------------------------

	unsigned int num_ptgs, num_alfas; // levels = m_robotShape.heights.size()
	TParameters<double> params;
	CParameterizedTrajectoryGenerator *ptgaux;


	num_ptgs = ini.read_int("NAVIGATION_CONFIG","PTG_COUNT", 1, true);
	params["ref_distance"] = ini.read_float("NAVIGATION_CONFIG","MAX_DISTANCE_PTG", 1, true);
	colGridRes = ini.read_float("NAVIGATION_CONFIG","GRID_RESOLUTION", 0.03, true);
	params["resolution"] = colGridRes;

	m_ptgmultilevel.resize(num_ptgs);


	// Read each PTG parameters, and generate K x N collisiongrids
	//	K - Number of PTGs
	//	N - Number of height sections

	for (unsigned int j=1; j<=num_ptgs; j++)
	{
		params["PTG_type"]	= ini.read_int("NAVIGATION_CONFIG",format("PTG%d_TYPE",j),1,true);
		params["v_max"] = ini.read_float("NAVIGATION_CONFIG",format("PTG%d_VMAX",j),1,true);
		params["w_max"] = DEG2RAD(ini.read_float("NAVIGATION_CONFIG",format("PTG%d_WMAX",j),1,true));
		params["K"] = ini.read_int("NAVIGATION_CONFIG",format("PTG%d_K",j),1,true);
		params["cte_a0v"] = DEG2RAD(ini.read_float("NAVIGATION_CONFIG",format("PTG%d_AV",j),1,true));
		params["cte_a0w"] = DEG2RAD(ini.read_float("NAVIGATION_CONFIG",format("PTG%d_AW",j),1,true));
		num_alfas = ini.read_int("NAVIGATION_CONFIG",format("PTG%d_NALFAS",j),30,true);

		for (unsigned int i=1; i<=m_robotShape.heights.size(); i++)
		{

			printf_debug("[loadConfigFile] Generating PTG#%u at level %u...",j,i);
			ptgaux = CParameterizedTrajectoryGenerator::CreatePTG(params);
			m_ptgmultilevel[j-1].PTGs.push_back(ptgaux);


			const float min_dist = 0.015f;
			m_timelogger.enter("PTG.simulateTrajectories");
			m_ptgmultilevel[j-1].PTGs[i-1]->simulateTrajectories(num_alfas,75, refDistance, 10*refDistance/min_dist, 0.0005f, min_dist);
			//Arguments -> n_alfas, max.tim, max.dist (ref_distance), max.n, diferencial_t, min_dist
			m_timelogger.leave("PTG.simulateTrajectories");

			// Just for debugging, etc.
			//m_ptgmultilevel[j-1].PTGs[i-1]->debugDumpInFiles(j);

			printf_debug("...OK!\n");

		}
	}

	//Load holonomic method params
	this->loadHolonomicMethodConfig(ini,"NAVIGATION_CONFIG");


	// Show configuration parameters:
	// -------------------------------------------------------------------
	printf_debug("\tLOADED CONFIGURATION:\n");
	printf_debug("-------------------------------------------------------------\n");

	ASSERT_(!m_holonomicMethod.empty())
	printf_debug("  Holonomic method \t\t= %s\n",typeid(m_holonomicMethod[0]).name());
	printf_debug("  PTG Count\t\t\t= %u\n", num_ptgs );
	printf_debug("  Max. ref. distance\t\t= %f\n", refDistance );
	printf_debug("  Cells resolution \t\t= %.04f\n", colGridRes );
	printf_debug("  Max. speed (v,w)\t\t= (%.04f m/sec, %.04f deg/sec)\n", robotMax_V_mps, robotMax_W_degps );
	printf_debug("  Robot Height Sections \t= %u\n", m_robotShape.heights.size() );
	printf_debug("\n\n");

	m_init_done = true;

	MRPT_END
}

/*************************************************************************
                         STEP1_CollisionGridsBuilder
     -> C-Paths generation.
     -> Build the collision grids
*************************************************************************/
void CReactiveNavigationSystem3D::STEP1_CollisionGridsBuilder()
{
	if (m_collisionGridsMustBeUpdated)
	{
		m_collisionGridsMustBeUpdated = false;

		m_timelogger.enter("build_PTG_collision_grids");

		for (unsigned int j=0; j<m_ptgmultilevel.size(); j++)
		{
			for (unsigned int i=0; i<m_robotShape.heights.size(); i++)
			{
				mrpt::nav::build_PTG_collision_grids(
					m_ptgmultilevel[j].PTGs[i],
					m_robotShape.polygons[i],
					format("%s/ReacNavGrid_%s_%03u_L%02u.dat.gz",ptg_cache_files_directory.c_str(),robotName.c_str(),i,j),
					m_enableConsoleOutput /*VERBOSE*/
					);
			}
		}

		m_timelogger.leave("build_PTG_collision_grids");
	}
}

/*************************************************************************

                         STEP2_SortObstacles

		Load the obstacles and sort them accorging to the height
				sections used to model the robot.

*************************************************************************/
bool CReactiveNavigationSystem3D::STEP2_SenseObstacles()
{
	//-------------------------------------------------------------------
	// The user must implement its own method to load the obstacles from
	// either sensor measurements or simulators (m_robot.senseObstacles(...))
	// Data have to be subsequently sorted in height bands according to the
	// height sections of the robot.
	//-------------------------------------------------------------------

	m_timelogger.enter("navigationStep.STEP2_LoadAndSortObstacle");

	m_robot.senseObstacles( m_WS_Obstacles_unsorted );

	// Empty slice maps:
	const size_t nSlices = m_robotShape.heights.size();
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

			h += m_robotShape.heights[idxH];
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
	std::vector<float> &out_TPObstacles )
{
	ASSERT_EQUAL_(m_WS_Obstacles_inlevels.size(),m_robotShape.heights.size())

	for (size_t j=0;j<m_robotShape.heights.size();j++)
	{
		size_t nObs;
		const float *xs,*ys,*zs;
		m_WS_Obstacles_inlevels[j].getPointsBuffer(nObs,xs,ys,zs);

		for (size_t obs=0;obs<nObs;obs++)
		{
			const float ox = xs[obs], oy = ys[obs];
			// Get TP-Obstacles:
			const CParameterizedTrajectoryGenerator::TCollisionCell &cell = m_ptgmultilevel[ptg_idx].PTGs[j]->m_collisionGrid.getTPObstacle(ox,oy);

			// Keep the minimum distance:
			for (CParameterizedTrajectoryGenerator::TCollisionCell::const_iterator i=cell.begin();i!=cell.end();++i)
				if ( i->second < out_TPObstacles[i->first] )
					out_TPObstacles[i->first] = i->second;
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
		for (unsigned int i=0; i < m_robotShape.heights.size(); i++)
			nVerts += m_robotShape.polygons[i].size() + 1;
		if (size_t(out_log.robotShape_x.size()) != nVerts)
		{
			out_log.robotShape_x.resize(nVerts);
			out_log.robotShape_y.resize(nVerts);
		}
		for (unsigned int i=0; i<m_robotShape.heights.size(); i++)
		{
			for (unsigned int j=0; j<m_robotShape.polygons[i].size(); j++)
			{
				paux = m_robotShape.polygons[i][j];
				out_log.robotShape_x[cuenta]= paux.x;
				out_log.robotShape_y[cuenta]= paux.y;
				cuenta++;
			}
			paux = m_robotShape.polygons[i][0];
			out_log.robotShape_x[cuenta]= paux.x;
			out_log.robotShape_y[cuenta]= paux.y;
			cuenta++;
		}
	}
}

