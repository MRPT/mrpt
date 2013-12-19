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

#include <mrpt/reactivenav.h>  // Precomp header

#include <mrpt/utils.h>
#include <mrpt/system/filesystem.h>

using namespace mrpt;
using namespace mrpt::poses;
using namespace mrpt::math;
using namespace mrpt::utils;
using namespace mrpt::reactivenav;
using namespace std;


/*---------------------------------------------------------------
					Constructor
  ---------------------------------------------------------------*/
CReactiveNavigationSystem::CReactiveNavigationSystem(
	CReactiveInterfaceImplementation   &react_iterf_impl,
    bool					enableConsoleOutput,
    bool					enableLogToFile)
	:
	CAbstractPTGBasedReactive(react_iterf_impl),
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
	m_collisionGridsMustBeUpdated = true;

	if ( shape.verticesCount()<3 )
		THROW_EXCEPTION("The robot shape has less than 3 vertices!!")

	m_robotShape = shape;
}


/*---------------------------------------------------------------
						loadConfigFile
  ---------------------------------------------------------------*/
void CReactiveNavigationSystem::loadConfigFile(const mrpt::utils::CConfigFileBase &ini, const mrpt::utils::CConfigFileBase &robotIni )
{
	MRPT_START

	m_collisionGridsMustBeUpdated = true;

	// Load config from INI file:
	// ------------------------------------------------------------
	robotName = robotIni.read_string("ROBOT_NAME","Name", "", true );

	unsigned int PTG_COUNT = ini.read_int(robotName,"PTG_COUNT",0, true );

	refDistance = ini.read_float(robotName,"MAX_REFERENCE_DISTANCE",5 );
	colGridRes = ini.read_float(robotName,"RESOLUCION_REJILLA_X",0.02f );

	MRPT_LOAD_CONFIG_VAR_NO_DEFAULT(robotMax_V_mps,float,  ini,robotName);
	MRPT_LOAD_CONFIG_VAR_NO_DEFAULT(robotMax_W_degps,float,  ini,robotName);


	ini.read_vector( robotName, "weights", vector<float>(0), weights, true );
	ASSERT_(weights.size()==6);

	MRPT_LOAD_CONFIG_VAR_NO_DEFAULT(minObstaclesHeight,float,  ini,robotName);
	MRPT_LOAD_CONFIG_VAR_NO_DEFAULT(maxObstaclesHeight,float,  ini,robotName);

	MRPT_LOAD_CONFIG_VAR_NO_DEFAULT(DIST_TO_TARGET_FOR_SENDING_EVENT,float,  ini,robotName);


	badNavAlarm_AlarmTimeout = ini.read_float("GLOBAL_CONFIG","ALARM_SEEMS_NOT_APPROACHING_TARGET_TIMEOUT", 30, true);

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

		params["PTG_type"]	= ini.read_int(robotName,format("PTG%u_Type", n ),1, true );
		params["v_max"]		= ini.read_float(robotName,format("PTG%u_v_max_mps", n ), 5, true);
		params["w_max"]		= DEG2RAD(ini.read_float(robotName,format("PTG%u_w_max_gps", n ), 0, true));
		params["K"]			= ini.read_int(robotName,format("PTG%u_K", n ), 1, false);
		params["cte_a0v"]	= DEG2RAD( ini.read_float(robotName,format("PTG%u_cte_a0v_deg", n ), 0, false) );
		params["cte_a0w"]	= DEG2RAD( ini.read_float(robotName,format("PTG%u_cte_a0w_deg", n ), 0, false) );

		const int nAlfas = ini.read_int(robotName,format("PTG%u_nAlfas", n ),100, true );

		// Generate it:
		printf_debug("[loadConfigFile] Generating PTG#%u...",n);

		PTGs[n] = CParameterizedTrajectoryGenerator::CreatePTG(params);

		printf_debug(PTGs[n]->getDescription().c_str());

		m_timelogger.enter("PTG.simulateTrajectories");
		PTGs[n]->simulateTrajectories(
		    nAlfas,					// alphas,
		    75,						// max.tim,
		    refDistance,			// max.dist,
		    600,					// max.n,
		    0.010f,					// diferencial_t
		    0.015f					// min_dist
			);
		m_timelogger.leave("PTG.simulateTrajectories");

		// Just for debugging, etc.
		//PTGs[n]->debugDumpInFiles(n);

		printf_debug("...OK!\n");
	}
	printf_debug("\n");

	this->loadHolonomicMethodConfig(ini,"GLOBAL_CONFIG");

	// Mostrar configuracion cargada de fichero:
	// --------------------------------------------------------
	printf_debug("\tLOADED CONFIGURATION:\n");
	printf_debug("-------------------------------------------------------------\n");

	printf_debug("  Robot name \t\t\t=%s\n",robotName.c_str());

	ASSERT_(!m_holonomicMethod.empty())
	printf_debug("  Holonomic method \t\t= %s\n",typeid(m_holonomicMethod[0]).name()); 
	printf_debug("\n  GPT Count\t\t\t= %u\n", (int)PTG_COUNT );
	printf_debug("  Max. ref. distance\t\t= %f\n", refDistance );
	printf_debug("  Cells resolution \t= %.04f\n", colGridRes );
	printf_debug("  Max. speed (v,w)\t\t= (%.04f m/sec, %.04f deg/sec)\n", robotMax_V_mps, robotMax_W_degps );
	printf_debug("  Robot Shape Points Count \t= %u\n", m_robotShape.verticesCount() );
	printf_debug("  Obstacles 'z' axis range \t= [%.03f,%.03f]\n", minObstaclesHeight, maxObstaclesHeight );
	printf_debug("\n\n");

	m_init_done = true;

	MRPT_END
}

/*************************************************************************

                         STEP1_CollisionGridsBuilder

     -> C-Paths generation.
     -> Check de colision entre cada celda de la rejilla y el contorno
          del robot

*************************************************************************/
void CReactiveNavigationSystem::STEP1_CollisionGridsBuilder()
{
	if (m_collisionGridsMustBeUpdated)
	{
		m_collisionGridsMustBeUpdated = false;

		m_timelogger.enter("build_PTG_collision_grids");

		mrpt::reactivenav::build_PTG_collision_grids(
			PTGs,
			m_robotShape,
			format("ReacNavGrid_%s",robotName.c_str())
			);

		m_timelogger.leave("build_PTG_collision_grids");
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
void CReactiveNavigationSystem::STEP3_WSpaceToTPSpace(const size_t ptg_idx,mrpt::vector_double &out_TPObstacles)
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
			const CParameterizedTrajectoryGenerator::TCollisionCell & cell = ptg->m_collisionGrid.getTPObstacle(ox,oy);

			// Keep the minimum distance:
			for (CParameterizedTrajectoryGenerator::TCollisionCell::const_iterator i=cell.begin();i!=cell.end();++i)
				if ( i->second < out_TPObstacles[ i->first ] )
					out_TPObstacles[i->first] = i->second;
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
