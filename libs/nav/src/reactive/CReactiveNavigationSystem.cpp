/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2016, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include "nav-precomp.h" // Precomp header

#include <mrpt/nav/reactive/CReactiveNavigationSystem.h>
#include <mrpt/nav/tpspace/CPTG_DiffDrive_CollisionGridBased.h>
#include <mrpt/system/filesystem.h>
#include <mrpt/utils/CConfigFileMemory.h>
#include <typeinfo>  // For typeid()

using namespace mrpt;
using namespace mrpt::poses;
using namespace mrpt::math;
using namespace mrpt::utils;
using namespace mrpt::nav;
using namespace std;


/*---------------------------------------------------------------
					Constructor
  ---------------------------------------------------------------*/
CReactiveNavigationSystem::CReactiveNavigationSystem(
	CRobot2NavInterface   &react_iterf_impl,
    bool					enableConsoleOutput,
    bool					enableLogToFile)
	:
	CAbstractPTGBasedReactive(react_iterf_impl,enableConsoleOutput,enableLogToFile),
	minObstaclesHeight           (-1.0),
	maxObstaclesHeight           (1e9)
{
}

// Dtor:
CReactiveNavigationSystem::~CReactiveNavigationSystem()
{
	this->preDestructor();

	// Free PTGs:
	for (size_t i=0;i<PTGs.size();i++)	delete PTGs[i];
	PTGs.clear();
}


/*---------------------------------------------------------------
						changeRobotShape
  ---------------------------------------------------------------*/
void CReactiveNavigationSystem::changeRobotShape( const math::CPolygon &shape )
{
	m_PTGsMustBeReInitialized = true;

	if ( shape.verticesCount()<3 )
		THROW_EXCEPTION("The robot shape has less than 3 vertices!!")

	m_robotShape = shape;
}


/** Reload the configuration from a file. See details in CReactiveNavigationSystem docs. */
void CReactiveNavigationSystem::loadConfigFile(const mrpt::utils::CConfigFileBase &ini)
{
	mrpt::utils::CConfigFileMemory dummyRobotParams;
	this->loadConfigFile(ini,dummyRobotParams);
}

/*---------------------------------------------------------------
						loadConfigFile
  ---------------------------------------------------------------*/
void CReactiveNavigationSystem::loadConfigFile(const mrpt::utils::CConfigFileBase &ini, const mrpt::utils::CConfigFileBase &robotIni )
{
	MRPT_START

	m_PTGsMustBeReInitialized = true;

	// Load config from INI file:
	// ------------------------------------------------------------
	robotName = robotIni.read_string("ROBOT_NAME","Name", "ReactiveParams" /* Default section for the rest of params */);

	unsigned int PTG_COUNT = ini.read_int(robotName,"PTG_COUNT",0, true );

	refDistance = ini.read_float(robotName,"MAX_REFERENCE_DISTANCE",5 );
	colGridRes = ini.read_float(robotName,"LUT_CELL_SIZE",0.0f );
	// backwards compt config file:
	if (!colGridRes) colGridRes = ini.read_float(robotName,"RESOLUCION_REJILLA_X",0.02f );

	MRPT_LOAD_CONFIG_VAR(SPEEDFILTER_TAU,float,  ini,robotName);

	ini.read_vector( robotName, "weights", vector<float>(0), weights, true );
	ASSERT_(weights.size()==6);

	MRPT_LOAD_CONFIG_VAR_NO_DEFAULT(minObstaclesHeight,float,  ini,robotName);
	MRPT_LOAD_CONFIG_VAR_NO_DEFAULT(maxObstaclesHeight,float,  ini,robotName);

	MRPT_LOAD_CONFIG_VAR_NO_DEFAULT(DIST_TO_TARGET_FOR_SENDING_EVENT,float,  ini,robotName);

	badNavAlarm_AlarmTimeout = ini.read_float("GLOBAL_CONFIG","ALARM_SEEMS_NOT_APPROACHING_TARGET_TIMEOUT", badNavAlarm_AlarmTimeout, true);

	// Load robot shape:
	// ---------------------------------------------
	math::CPolygon		shape;
	vector<float>        xs,ys;

	ini.read_vector(robotName,"RobotModel_shape2D_xs",vector<float>(0), xs, true );
	ini.read_vector(robotName,"RobotModel_shape2D_ys",vector<float>(0), ys, true );
	ASSERT_(xs.size()==ys.size());

	// Add to polygon
	for (size_t i=0;i<xs.size();i++)
		shape.AddVertex(xs[i],ys[i]);

	changeRobotShape( shape );

	// Load PTGs from file:
	// ---------------------------------------------
	// Free previous PTGs:
	for (size_t i=0;i<PTGs.size();i++)	delete PTGs[i];
	PTGs.assign(PTG_COUNT,NULL);

	printf_debug("\n");

	for ( unsigned int n=0;n<PTG_COUNT;n++ )
	{
		// load params of this PTG:

		TParameters<double> params;
		params["ref_distance"] = refDistance;
		params["resolution"]   = colGridRes;

		params["v_max"]		= ini.read_float(robotName,format("PTG%u_v_max_mps", n ), 5, true);
		params["w_max"]		= DEG2RAD(ini.read_float(robotName,format("PTG%u_w_max_gps", n ), 0, true));
		params["K"]			= ini.read_int(robotName,format("PTG%u_K", n ), 1, false);
		params["cte_a0v"]	= DEG2RAD( ini.read_float(robotName,format("PTG%u_cte_a0v_deg", n ), 0, false) );
		params["cte_a0w"]	= DEG2RAD( ini.read_float(robotName,format("PTG%u_cte_a0w_deg", n ), 0, false) );

		// For backwards compat with old config files:
		const int num_paths1  = ini.read_int(robotName,format("PTG%u_nAlfas", n ),-1, false);
		// New recommended param name:
		const int num_paths2 = ini.read_int(robotName,format("PTG%u_num_paths", n ),-1,false);
		if (num_paths1<=0 && num_paths2<=0)
			THROW_EXCEPTION_CUSTOM_MSG1("Missing configuration parameter: `PTG%u_num_paths`",n);

		params["num_paths"] = num_paths2 > 0 ? num_paths2 : num_paths1;

		// Factory:
		const std::string sPTGName = ini.read_string(robotName,format("PTG%u_Type", n ),"", true );
		PTGs[n] = CParameterizedTrajectoryGenerator::CreatePTG(sPTGName,params);
	}
	printf_debug("\n");

	this->STEP1_InitPTGs();


	this->loadHolonomicMethodConfig(ini,"GLOBAL_CONFIG");

	// Mostrar configuracion cargada de fichero:
	// --------------------------------------------------------
	printf_debug("\tLOADED CONFIGURATION:\n");
	printf_debug("-------------------------------------------------------------\n");

	ASSERT_(!m_holonomicMethod.empty())
	printf_debug("  Holonomic method \t\t= %s\n",typeid(m_holonomicMethod[0]).name());
	printf_debug("\n  GPT Count\t\t\t= %u\n", (int)PTG_COUNT );
	printf_debug("  Max. ref. distance\t\t= %f\n", refDistance );
	printf_debug("  Cells resolution \t= %.04f\n", colGridRes );
	printf_debug("  Robot Shape Points Count \t= %u\n", m_robotShape.verticesCount() );
	printf_debug("  Obstacles 'z' axis range \t= [%.03f,%.03f]\n", minObstaclesHeight, maxObstaclesHeight );
	printf_debug("\n\n");

	m_init_done = true;

	MRPT_END
}

void CReactiveNavigationSystem::STEP1_InitPTGs()
{
	if (m_PTGsMustBeReInitialized)
	{
		m_PTGsMustBeReInitialized = false;

		mrpt::utils::CTimeLoggerEntry tle(m_timelogger,"STEP1_InitPTGs");
		
		for (unsigned int i=0;i<PTGs.size();i++)
		{
			PTGs[i]->deinitialize();

			printf_debug("[loadConfigFile] Initializing PTG#%u...", i);
			printf_debug(PTGs[i]->getDescription().c_str());

			// Set robot shape:
			{
				mrpt::nav::CPTG_DiffDrive_CollisionGridBased *ptg = dynamic_cast<mrpt::nav::CPTG_DiffDrive_CollisionGridBased *>(PTGs[i]);
				if (ptg)
					ptg->setRobotShape(m_robotShape);
			}
			PTGs[i]->initialize(
				format("%s/ReacNavGrid_%s_%03u.dat.gz", ptg_cache_files_directory.c_str(), robotName.c_str(), i),
				m_enableConsoleOutput /*verbose*/
			);
			printf_debug("...Done!\n");
		}
	}
}

/*************************************************************************
		STEP2_SenseObstacles
*************************************************************************/
bool CReactiveNavigationSystem::STEP2_SenseObstacles()
{
	try
	{
		CTimeLoggerEntry tle(m_timelogger,"navigationStep.STEP2_Sense");

		// Return true on success:
		return m_robot.senseObstacles( m_WS_Obstacles );
		// Note: Clip obstacles by "z" axis coordinates is more efficiently done in STEP3_WSpaceToTPSpace()
	}
	catch (std::exception &e)
	{
		printf_debug("[CReactiveNavigationSystem::STEP2_Sense] Exception:");
		printf_debug((char*)(e.what()));
		return false;
	}
	catch (...)
	{
		printf_debug("[CReactiveNavigationSystem::STEP2_Sense] Unexpected exception!\n");
		return false;
	}

}

/*************************************************************************
				STEP3_WSpaceToTPSpace
*************************************************************************/
void CReactiveNavigationSystem::STEP3_WSpaceToTPSpace(const size_t ptg_idx,std::vector<double> &out_TPObstacles)
{
	CParameterizedTrajectoryGenerator	*ptg = this->PTGs[ptg_idx];

	const float OBS_MAX_XY = this->refDistance*1.1f;

	// Merge all the (k,d) for which the robot collides with each obstacle point:
	size_t nObs;
	const float *xs,*ys,*zs;
	m_WS_Obstacles.getPointsBuffer(nObs,xs,ys,zs);

	for (size_t obs=0;obs<nObs;obs++)
	{
		const float ox=xs[obs], oy = ys[obs], oz=zs[obs];

		if (ox>-OBS_MAX_XY && ox<OBS_MAX_XY &&
			oy>-OBS_MAX_XY && oy<OBS_MAX_XY &&
			oz>=minObstaclesHeight && oz<=maxObstaclesHeight)
		{
			ptg->updateTPObstacle(ox, oy, out_TPObstacles);
		}
	}
}


/** Generates a pointcloud of obstacles, and the robot shape, to be saved in the logging record for the current timestep */
void CReactiveNavigationSystem::loggingGetWSObstaclesAndShape(CLogFileRecord &out_log)
{
	out_log.WS_Obstacles = m_WS_Obstacles;

	const size_t nVerts = m_robotShape.size();
    out_log.robotShape_x.resize(nVerts);
    out_log.robotShape_y.resize(nVerts);

    for (size_t i=0;i<nVerts;i++)
    {
		out_log.robotShape_x[i]= m_robotShape.GetVertex_x(i);
		out_log.robotShape_y[i]= m_robotShape.GetVertex_y(i);
    }
}
