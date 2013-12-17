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
	nLastSelectedPTG             (-1),
	m_decimateHeadingEstimate    (0),
	minObstaclesHeight           (-1.0),
	maxObstaclesHeight           (1e9)
{
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


	badNavAlarm_AlarmTimeout = ini.read_float("GLOBAL_CONFIG","ALARM_SEEMS_NOT_APPROACHING_TARGET_TIMEOUT", 10, true);

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

/*---------------------------------------------------------------
	  				performNavigationStep

  Este metodo se ejecuta periodicamente solo si se está en el estado
   NAVIGATING. Aqui se implementa el algoritmo de navegacion reactiva
  ---------------------------------------------------------------*/
void  CReactiveNavigationSystem::performNavigationStep()
{
	
	CLogFileRecord							newLogRec;
	float										targetDist;
	poses::CPoint2D								relTarget;		// The target point, relative to current robot pose.
	poses::CPose2D								curPose;
	float										curVL;			// en metros/seg
	float										curW;			// en rad/segundo
	THolonomicMovement							selectedHolonomicMovement;
	float										cmd_v=0,cmd_w=0;	// The non-holonomic command
	std::vector<CHolonomicLogFileRecordPtr>		HLFRs;
	int											nSelectedPTG;

	float cur_approx_heading_dir = 0;

	// Already closing??
	if (m_closing_navigator) return;

	if (!m_init_done) THROW_EXCEPTION("Have you called loadConfigFile() before?")

	// Lock
	mrpt::synch::CCriticalSectionLocker lock( &m_critZoneNavigating );

	m_timelogger.enter("navigationStep");
	try
	{
		// Start timer
		totalExecutionTime.Tic();

		const mrpt::system::TTimeStamp tim_start_iteration = mrpt::system::now();

		/* ----------------------------------------------------------------
		 	  Request current robot pose and velocities
		   ---------------------------------------------------------------- */
		m_timelogger.enter("navigationStep.getCurrentPoseAndSpeeds");
		if ( !m_robot.getCurrentPoseAndSpeeds(curPose, curVL, curW ) )
		{
			m_timelogger.leave("navigationStep.getCurrentPoseAndSpeeds");
			m_timelogger.leave("navigationStep");
			doEmergencyStop("ERROR calling m_robot.getCurrentPoseAndSpeeds, stopping robot and finishing navigation");
			return;
		}
		m_timelogger.leave("navigationStep.getCurrentPoseAndSpeeds");

		/* ----------------------------------------------------------------
		 	  Have we reached the target location?
		   ---------------------------------------------------------------- */
		targetDist= curPose.distance2DTo( m_navigationParams.target.x, m_navigationParams.target.y );


		// Enviar evento de final de navegacion??
		if (!navigationEndEventSent && targetDist<DIST_TO_TARGET_FOR_SENDING_EVENT)
		{
			navigationEndEventSent = true;
			m_robot.sendNavigationEndEvent();
		}

		if ( targetDist < m_navigationParams.targetAllowedDistance )
		{
			m_robot.stop();
			m_navigationState = IDLE;
            if (m_enableConsoleOutput) printf_debug("Navigation target was reached!\n" );

			if (!navigationEndEventSent)
			{
				navigationEndEventSent = true;
				m_robot.sendNavigationEndEvent();
			}
			m_timelogger.leave("navigationStep");
			return;
		}

		// Check the "no approaching the target"-alarm:
		// -----------------------------------------------------------
		if (targetDist < badNavAlarm_minDistTarget )
		{
			badNavAlarm_minDistTarget = targetDist;
			badNavAlarm_lastMinDistTime =  system::getCurrentTime();
		}
		else
		{
			// Too much time have passed?
			if ( system::timeDifference( badNavAlarm_lastMinDistTime, system::getCurrentTime() ) > badNavAlarm_AlarmTimeout)
			{
				std::cout << "\n--------------------------------------------\nWARNING: Timeout for approaching toward the target expired!! Aborting navigation!! \n---------------------------------\n";

				m_navigationState = NAV_ERROR;
				m_timelogger.leave("navigationStep");
				return;
			}
		}


		// Compute target location relative to current robot pose:
		// ---------------------------------------------------------------------
		relTarget = CPoint2D(m_navigationParams.target) - curPose;

		// STEP1: Collision Grids Builder.
		// -----------------------------------------------------------------------------
		STEP1_CollisionGridsBuilder();

		// STEP2: Sense obstacles.
		// -----------------------------------------------------------------------------

		// Start timer
		executionTime.Tic();

		// For some behaviors:
		//  If set to true, "cmd_v" & "cmd_w" must be set to the desired values:
		bool		skipNormalReactiveNavigation = false;


		if (! skipNormalReactiveNavigation )
		{

			//  STEP3: Build TP-Obstacles and transform target location into TP-Space
			// -----------------------------------------------------------------------------
			// Vectors for each PTG:
			if (PTGs.size() != TP_Targets.size() )   TP_Targets.resize( PTGs.size() );
			if (PTGs.size() != valid_TP.size() )	valid_TP.resize( PTGs.size() );

			// Vectors for each PTG & Security Distance:
			const size_t n = PTGs.size();
			if (n != TP_Obstacles.size() )				TP_Obstacles.resize( n );
			if (n != times_TP_transformations.size() )	times_TP_transformations.resize( n );
			if (n != holonomicMovements.size() )		holonomicMovements.resize( n );
			if (n != HLFRs.size() )						HLFRs.resize(n);
			if (n != times_HoloNav.size() )				times_HoloNav.resize( n );
			newLogRec.infoPerPTG.resize( n );

			//// For each PTG:
			//for (size_t indexPTG=0;indexPTG<PTGs.size();indexPTG++)
			//{
			//	// Target location:
			//	float	alpha,dist;
			//	int		k;

			//	// Firstly, check if target falls into the PTG domain!!
			//	valid_TP[indexPTG] = true;
			//	//valid_TP[i] = PTGs[i]->PTG_IsIntoDomain( relTarget.x,relTarget.y );

			//	if (valid_TP[indexPTG])
			//	{
			//		PTGs[indexPTG]->lambdaFunction(
			//		    relTarget.x(),
			//		    relTarget.y(),
			//		    k,
			//		    dist );

			//		alpha = PTGs[indexPTG]->index2alpha(k);
			//		TP_Targets[indexPTG].x( cos(alpha) * dist );
			//		TP_Targets[indexPTG].y( sin(alpha) * dist );
			//	}

			//	// And for each security distance:
			//	tictac.Tic();

			//	// TP-Obstacles
			//	STEP3_SpaceTransformer(	WS_Obstacles,
			//	                        PTGs[indexPTG],
			//	                        TP_Obstacles[indexPTG] );

			//	times_TP_transformations[indexPTG] = tictac.Tac();

			//} // indexPTG

			//  STEP4: Holonomic navigation method
			// -----------------------------------------------------------------------------
			// For each PTG:
			for (size_t indexPTG=0;indexPTG<PTGs.size();indexPTG++)
			{
				tictac.Tic();

				holonomicMovements[indexPTG].PTG = PTGs[indexPTG];

				if (valid_TP[indexPTG])
				{
					STEP4_HolonomicMethod(	TP_Obstacles[indexPTG],
					                       TP_Targets[indexPTG],
					                       PTGs[indexPTG]->getMax_V_inTPSpace(),
					                       holonomicMovements[indexPTG],
					                       HLFRs[indexPTG]);
				}
				else
				{
					holonomicMovements[indexPTG].direction = 0;
					holonomicMovements[indexPTG].evaluation = 0;
					holonomicMovements[indexPTG].speed = 0;
					HLFRs[indexPTG] = CLogFileRecord_VFF::Create();
				}
				times_HoloNav[indexPTG] = (float)tictac.Tac();
			} // indexPTG

			// STEP5: Evaluate each movement to assign them a "evaluation" value.
			// ---------------------------------------------------------------------

			// For each PTG:
			for (size_t indexPTG=0;indexPTG<PTGs.size();indexPTG++)
			{
				if (valid_TP[indexPTG])
					STEP5_Evaluator(	holonomicMovements[indexPTG],
					                 TP_Obstacles[indexPTG],
					                 relTarget,
					                 TP_Targets[indexPTG],
					                 nLastSelectedPTG == (int)indexPTG,
					                 newLogRec.infoPerPTG[indexPTG] );

				if ( HLFRs[indexPTG].present()  )
				{
					if (IS_CLASS(HLFRs[indexPTG], CLogFileRecord_ND))
					{
						if (CLogFileRecord_NDPtr(HLFRs[indexPTG])->situation == CHolonomicND::SITUATION_TARGET_DIRECTLY)
						{
							holonomicMovements[indexPTG].evaluation += 1.0f;
						}
					}
				}
			} // indexPTG

			// STEP6: Selects the best movement
			// ---------------------------------------------------------------------
			STEP6_Selector( holonomicMovements, selectedHolonomicMovement, nSelectedPTG );
			nLastSelectedPTG = nSelectedPTG;


			// Compute the approximate heading direction
			// ----------------------------------------------------------------
			{
				if (++m_decimateHeadingEstimate>10)
				{
					m_decimateHeadingEstimate=0;

					float x=1;
					float y=0;
					float p,t;
					selectedHolonomicMovement.PTG->getCPointWhen_d_Is(2.0, selectedHolonomicMovement.PTG->alpha2index(selectedHolonomicMovement.direction),x,y,p,t);

					cur_approx_heading_dir = atan2(y,x);
					m_robot.notifyHeadingDirection(cur_approx_heading_dir);
				}
			}

			// STEP7: Get the non-holonomic movement command.
			// ---------------------------------------------------------------------
			STEP7_NonHolonomicMovement( selectedHolonomicMovement, cmd_v, cmd_w);

			last_cmd_v = cmd_v;
			last_cmd_w = cmd_w;

		} // end of "!skipNormalReactiveNavigation"

		// ---------------------------------------------------------------------
		//				SEND MOVEMENT COMMAND TO THE ROBOT
		// ---------------------------------------------------------------------
		if ( cmd_v == 0.0 && cmd_w == 0.0 )
		{
			m_robot.stop();
		}
		else
		{
			if ( !m_robot.changeSpeeds( cmd_v, cmd_w ) )
			{
				doEmergencyStop("\nERROR calling RobotMotionControl::changeSpeeds!! Stopping robot and finishing navigation\n");
				m_timelogger.leave("navigationStep");
				return;
			}
		}

		// Statistics:
		// ----------------------------------------------------
		float	executionTimeValue = (float) executionTime.Tac();
		meanExecutionTime=  0.3f * meanExecutionTime +
		                    0.7f * executionTimeValue;
		meanTotalExecutionTime=  0.3f * meanTotalExecutionTime +
		                         0.7f * ((float)totalExecutionTime.Tac() );
		meanExecutionPeriod = 0.3f * meanExecutionPeriod +
		                      0.7f * min(1.0f, (float)timerForExecutionPeriod.Tac());


		timerForExecutionPeriod.Tic();


        if (m_enableConsoleOutput)
        {
            printf_debug("CMD:%.02lfm/s,%.02lfd/s \t",
		           (double)cmd_v,
		           (double)RAD2DEG( cmd_w ) );

            printf_debug(" T=%.01lfms Exec:%.01lfms|%.01lfms \t",
		           1000.0*meanExecutionPeriod,
		           1000.0*meanExecutionTime,
		           1000.0*meanTotalExecutionTime );
        }

		if (!skipNormalReactiveNavigation)
		{
            if (m_enableConsoleOutput)
            {
                printf_debug("E=%.01lf ", (double)selectedHolonomicMovement.evaluation );
                printf_debug("PTG#%i ", nSelectedPTG);
            }
		}
		else
		{
			nSelectedPTG = 0;
		}

        if (m_enableConsoleOutput) printf_debug("\n");

		// ---------------------------------------
		// Generate log record
		// ---------------------------------------
		m_timelogger.enter("navigationStep.populate_log_info");

		newLogRec.WS_Obstacles			= WS_Obstacles;
		newLogRec.robotOdometryPose		= curPose;
		newLogRec.WS_target_relative	= relTarget;
		newLogRec.v						= cmd_v;
		newLogRec.w						= cmd_w;
		newLogRec.nSelectedPTG			= nSelectedPTG;
		newLogRec.executionTime			= executionTimeValue;
		newLogRec.actual_v				= curVL;
		newLogRec.actual_w				= curW;
		newLogRec.estimatedExecutionPeriod = meanExecutionPeriod;
		newLogRec.timestamp             = tim_start_iteration;
		newLogRec.nPTGs					= PTGs.size();
		newLogRec.navigatorBehavior		= 0;  // Not used now

		const size_t nVerts = m_robotShape.size();
		if (size_t(newLogRec.robotShape_x.size()) != nVerts)
		{
			newLogRec.robotShape_x.resize(nVerts);
			newLogRec.robotShape_y.resize(nVerts);
		}
		for (size_t i=0;i<nVerts;i++)
		{
			newLogRec.robotShape_x[i]= m_robotShape.GetVertex_x(i);
			newLogRec.robotShape_y[i]= m_robotShape.GetVertex_y(i);
		}

		// For each PTG:
		if (!skipNormalReactiveNavigation)
		{
			for (size_t indexPTG=0;indexPTG<PTGs.size();indexPTG++)
			{
				newLogRec.infoPerPTG[indexPTG].PTG_desc					= PTGs[indexPTG]->getDescription();
				mrpt::utils::metaprogramming::copy_container_typecasting(TP_Obstacles[indexPTG], newLogRec.infoPerPTG[indexPTG].TP_Obstacles);
				newLogRec.infoPerPTG[indexPTG].TP_Target					= TP_Targets[indexPTG];
				newLogRec.infoPerPTG[indexPTG].timeForTPObsTransformation	= times_TP_transformations[indexPTG];
				newLogRec.infoPerPTG[indexPTG].timeForHolonomicMethod		= times_HoloNav[indexPTG];
				newLogRec.infoPerPTG[indexPTG].HLFR = HLFRs[indexPTG];
				newLogRec.infoPerPTG[indexPTG].desiredDirection = holonomicMovements[indexPTG].direction;
				newLogRec.infoPerPTG[indexPTG].desiredSpeed = holonomicMovements[indexPTG].speed;
				newLogRec.infoPerPTG[indexPTG].evaluation = holonomicMovements[indexPTG].evaluation;
			}
		}
		else
		{
			newLogRec.infoPerPTG.clear();
		}

		m_timelogger.leave("navigationStep.populate_log_info");

		// --------------------------------------
		//  Save to log file:
		// --------------------------------------
		m_timelogger.enter("navigationStep.write_log_file");
		if (logFile) (*logFile) << newLogRec;
		m_timelogger.leave("navigationStep.write_log_file");

		// --------------------------------------
		// Set as last log record
		// --------------------------------------
		// Lock
		{
			mrpt::synch::CCriticalSectionLocker lock_log(&m_critZoneLastLog);
			// COPY
			lastLogRecord = newLogRec;
		}
	}
	catch (std::exception &e)
	{
		std::cout << e.what();
		std::cout << "[CReactiveNavigationSystem::performNavigationStep] Exceptions!!\n";
	}
	catch (...)
	{
		std::cout << "[CReactiveNavigationSystem::performNavigationStep] Unexpected exception!!:\n";
	}

	m_timelogger.leave("navigationStep");
}

/*************************************************************************

                         STEP1_CollisionGridsBuilder

     -> C-Paths generation.
     -> Check de colision entre cada celda de la rejilla y el contorno
          del robot

*************************************************************************/
void CReactiveNavigationSystem::STEP1_CollisionGridsBuilder()
{
	try
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
	catch (std::exception &e)
	{
		printf_debug("[CReactiveNavigationSystem::STEP1_CollisionGridsBuilder] Exception:");
		printf_debug(e.what());
	}
}

/*************************************************************************
		STEP2_SenseObstacles
*************************************************************************/
bool CReactiveNavigationSystem::STEP2_SenseObstacles()
{
	try
	{
		m_timelogger.enter("navigationStep.STEP2_Sense");

		const bool ret = m_robot.senseObstacles( m_WS_Obstacles );

		// Clip obstacles by "z" axis coordinates:
		m_WS_Obstacles.clipOutOfRangeInZ( minObstaclesHeight, maxObstaclesHeight );

		// Clip obstacles out of the reactive method range:
		CPoint2D    dumm(0,0);
		m_WS_Obstacles.clipOutOfRange( dumm, refDistance+1.5f );

		m_timelogger.leave("navigationStep.STEP2_Sense");

		return ret;
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
void CReactiveNavigationSystem::STEP3_WSpaceToTPSpace(const mrpt::poses::CPose2D &relTarget)
//void STEP3_SpaceTransformer(
//    mrpt::poses::CPointsMap					&in_obstacles,
//    CParameterizedTrajectoryGenerator	*in_PTG,
//    vector_double						&out_TPObstacles
)
{
	// For each PTG:
	for (size_t indexPTG=0;indexPTG<PTGs.size();indexPTG++)
	{
		//// TP-Obstacles
		//STEP3_SpaceTransformer(	WS_Obstacles,
		//	                    PTGs[indexPTG],
		//	                    TP_Obstacles[indexPTG] );

		CParameterizedTrajectoryGenerator	*in_PTG = this->PTGs[indexPTG];

		const size_t Ki = in_PTG->getAlfaValuesCount();

		// Ver si hay espacio ya reservado en los TP-Obstacles:
		if ( size_t(out_TPObstacles.size()) != Ki )
			out_TPObstacles.resize( Ki );

		// Coger "k"s y "distances" a las que choca cada punto de obstaculo
		//  de la "Rejilla" del PT dado.
		// --------------------------------------------------------------------
		const size_t nObs = in_obstacles.getPointsCount();

		for (size_t k=0;k<Ki;k++)
		{
			// Iniciar a max. distancia o hasta que abs(phi)=pi
			out_TPObstacles[k] = in_PTG->refDistance;

			// Si acaba girado 180deg, acabar ahi:
			float phi = in_PTG->GetCPathPoint_phi(k,in_PTG->getPointsCountInCPath_k(k)-1);

			if (fabs(phi) >= M_PI* 0.95 )
				out_TPObstacles[k]= in_PTG->GetCPathPoint_d(k,in_PTG->getPointsCountInCPath_k(k)-1);
		}

		for (size_t obs=0;obs<nObs;obs++)
		{
			float ox,oy;
			in_obstacles.getPoint(obs, ox,oy);

			const CParameterizedTrajectoryGenerator::TCollisionCell & cell = in_PTG->m_collisionGrid.getTPObstacle(ox,oy);

			// Keep the minimum distance:
			for (CParameterizedTrajectoryGenerator::TCollisionCell::const_iterator i=cell.begin();i!=cell.end();i++)
				if ( i->second < out_TPObstacles[ i->first ] )
					out_TPObstacles[i->first] = i->second;
		}

		// Distances in TP-Space are normalized to [0,1]:
		for (size_t i=0;i<Ki;i++)
			out_TPObstacles[i] /= in_PTG->refDistance;

	} // end foreach indexPTG

}

/*************************************************************************

                             STEP4_HolonomicMethod

*************************************************************************/
void CReactiveNavigationSystem::STEP4_HolonomicMethod(
	const vector_double         & in_Obstacles,
	const mrpt::math::TPoint2D  & in_Target,
    const float                   in_maxRobotSpeed,
    THolonomicMovement          & out_selectedMovement,
    CHolonomicLogFileRecordPtr  & in_HLFR )
{
	try
	{
		m_timelogger.enter("navigationStep.STEP4_HolonomicMethod");

		holonomicMethod->navigate(	in_Target,
		                           in_Obstacles,
		                           in_maxRobotSpeed,
		                           out_selectedMovement.direction,
		                           out_selectedMovement.speed,
		                           in_HLFR );

		m_timelogger.leave("navigationStep.STEP4_HolonomicMethod");
	}
	catch (std::exception &e)
	{
		m_timelogger.leave("navigationStep.STEP4_HolonomicMethod");
		printf_debug("[CReactiveNavigationSystem::STEP4_HolonomicMethod] Exception:");
		printf_debug((char*)(e.what()));
	}
	catch (...)
	{
		m_timelogger.leave("navigationStep.STEP4_HolonomicMethod");
		printf_debug("[CReactiveNavigationSystem::STEP4_HolonomicMethod] Unexpected exception!\n");
	}
}

/*************************************************************************

						 	STEP5_Evaluator

*************************************************************************/
void CReactiveNavigationSystem::STEP5_Evaluator(
    THolonomicMovement			&in_holonomicMovement,
    vector_double				&in_TPObstacles,
    mrpt::poses::CPoint2D		&WS_Target,
    mrpt::poses::CPoint2D		&TP_Target,
    bool						wasSelectedInLast,
    CLogFileRecord::TInfoPerPTG	&log)
{
	try
	{
		m_timelogger.enter("navigationStep.STEP5_Evaluator");

		float	a;
		float	factor1, factor2,factor3,factor4,factor5,factor6;

		if (TP_Target.x()!=0 || TP_Target.y()!=0)
			a = atan2( TP_Target.y(), TP_Target.x());
		else	a = 0;


		const int		TargetSector = in_holonomicMovement.PTG->alpha2index( a );
		const double	TargetDist = TP_Target.norm();
		const int		kDirection = in_holonomicMovement.PTG->alpha2index( in_holonomicMovement.direction );
		const double	refDist	   = in_holonomicMovement.PTG->refDistance;

		// Las coordenadas en C-Space representativas de la trajectoria seleccionada:
		float	x,y,phi,t,d;
		d = min( in_TPObstacles[ kDirection ], 0.90f*TargetDist);
		in_holonomicMovement.PTG->getCPointWhen_d_Is( d, kDirection,x,y,phi,t );

		// Factor 1: Distancia hasta donde llego por esta GPT:
		// -----------------------------------------------------
		factor1 = in_TPObstacles[kDirection];

		//	if (kDirection == TargetSector )	// Si es TARGET_DIRECTLY:
		//			factor1 = 1 - TargetDist;				// Llego todo lo lejos q quiero ir!! :-)
		//	else	factor1 = in_TPObstacles[kDirection];

		// Factor 2: Distancia en sectores:
		// -------------------------------------------
		float   dif = fabs(((float)( TargetSector - kDirection )));
		float	nSectors = (float)in_TPObstacles.size();
		if ( dif > (0.5f*nSectors)) dif = nSectors - dif;
		factor2= exp(-square( dif / (in_TPObstacles.size()/3.0f))) ;

		// Factor 3: Angulo que hará el robot con target en (x,y):
		// -----------------------------------------------------
		float   t_ang = atan2( WS_Target.y() - y, WS_Target.x() - x );
		t_ang -= phi;

		while (t_ang> M_PI)  t_ang-=(float)M_2PI;
		while (t_ang<-M_PI)  t_ang+=(float)M_2PI;

		factor3 = exp(-square( t_ang / (float)(0.5f*M_PI)) );

		// Factor4:		DECREMENTO de la distancia euclidea entre (x,y) y target:
		//  Se valora negativamente el alejarse del target
		// -----------------------------------------------------
		float dist_eucl_final = WS_Target.distance2DTo(x,y);
		float dist_eucl_now   = WS_Target.norm();

		//    float dist_eucl_final = sqrt( square( d*cos(in_holonomicMovement.direction)-TP_Target.x ) + square( d*sin(in_holonomicMovement.direction) -TP_Target.y ) );
		//	float dist_eucl_now   = sqrt( square( TP_Target.x ) + square( TP_Target.y ) );

		factor4 = min(2.0*refDist,max(0.0,((dist_eucl_now - dist_eucl_final)+refDist)))/(2*refDist);

		// ---------
		//	float decrementDistanc = dist_eucl_now - dist_eucl_final;
		//	if (dist_eucl_now>0)
		//			factor4 = min(1.0,min(refDist*2,max(0,decrementDistanc + refDist)) / dist_eucl_now);
		//	else	factor4 = 0;
		// ---------
		//	factor4 = min(2*refDist2,max(0,decrementDistanc + refDist2)) / (2*refDist2);
		//  factor4=  (refDist2 - min( refDist2, dist_eucl ) ) / refDist2;

		// Factor5: Histeresis:
		// -----------------------------------------------------
		float	want_v,want_w;
		in_holonomicMovement.PTG->directionToMotionCommand( kDirection, want_v,want_w);

		float	likely_v = exp( -fabs(want_v-last_cmd_v)/0.10f );
		float	likely_w = exp( -fabs(want_w-last_cmd_w)/0.40f );

		factor5 = min( likely_v,likely_w );
		//factor5 = wasSelectedInLast ? 1:0;

		// Factor6: Security distance !!
		// -----------------------------------------------------
		factor6 = 0;

		// --------------------
		//  SAVE LOG
		// --------------------
		log.evalFactors.resize(6);
		log.evalFactors[0] = factor1;
		log.evalFactors[1] = factor2;
		log.evalFactors[2] = factor3;
		log.evalFactors[3] = factor4;
		log.evalFactors[4] = factor5;
		log.evalFactors[5] = factor6;

		if (in_holonomicMovement.speed==0)
		{
			// If no movement has been found -> the worst evaluation:
			in_holonomicMovement.evaluation = 0;
		}
		else
		{
			// Sum: Dos casos:
			if (dif<2	&&										// Heading the target
			        in_TPObstacles[kDirection]*0.95f>TargetDist 	// and free space towards the target
			   )
			{
				// Caso de camino directo al target:
//				in_holonomicMovement.evaluation = 1.0f + (1 - TargetDist) + factor5 * weight5 + factor6*weight6;
				in_holonomicMovement.evaluation = 1.0f + (1 - t/15.0f) + factor5 * weights[4] + factor6*weights[5];
			}
			else
			{
				// Caso general:
				in_holonomicMovement.evaluation = (
				                                      factor1 * weights[0] +
				                                      factor2 * weights[1] +
				                                      factor3 * weights[2] +
				                                      factor4 * weights[3] +
				                                      factor5 * weights[4] +
				                                      factor6 * weights[5]
				                                  ) / ( math::sum(weights));
			}
		}

		m_timelogger.leave("navigationStep.STEP5_Evaluator");
	}
	catch (std::exception &e)
	{
		m_timelogger.leave("navigationStep.STEP5_Evaluator");
		THROW_STACKED_EXCEPTION(e);
	}
	catch (...)
	{
		m_timelogger.leave("navigationStep.STEP5_Evaluator");
		std::cout << "[CReactiveNavigationSystem::STEP5_Evaluator] Unexpected exception!:\n";
	}

}

/*************************************************************************

							STEP6_Selector

*************************************************************************/
void CReactiveNavigationSystem::STEP6_Selector(
    std::vector<THolonomicMovement>		&in_holonomicMovements,
    THolonomicMovement					&out_selected,
    int									&out_nSelectedPTG)
{
	m_timelogger.enter("navigationStep.STEP6_Selector");

	// Si no encontramos nada mejor, es que no hay movimiento posible:
	out_selected.direction= 0;
	out_selected.speed = 0;
	out_selected.PTG = NULL;
	out_selected.evaluation= 0;		// Anotacion pa mi: Don't modify this 0 and the ">" comparison
	out_nSelectedPTG = 0;

	// Coger la trayectoria con mejor evaluacion, mientras no produzca
	//  colision:
	for (unsigned int i=0;i<in_holonomicMovements.size();i++)
	{
		float ev = in_holonomicMovements[i].evaluation;
		if ( ev > out_selected.evaluation )
		{
			out_selected = in_holonomicMovements[i];
			out_selected.evaluation = ev;
			out_nSelectedPTG = i;
		}
	}

	m_timelogger.leave("navigationStep.STEP6_Selector");
}

/*************************************************************************

						STEP7_NonHolonomicMovement

*************************************************************************/
void CReactiveNavigationSystem::STEP7_NonHolonomicMovement(
    THolonomicMovement					&in_movement,
    float								&out_v,
    float								&out_w)
{
	try
	{
		m_timelogger.enter("navigationStep.STEP7_NonHolonomicMovement");

		if (in_movement.speed==0)
		{
			// The robot will stop:
			out_v = out_w = 0;
		}
		else
		{
			// Take the normalized movement command:
			in_movement.PTG->directionToMotionCommand(
			    in_movement.PTG->alpha2index( in_movement.direction ),
			    out_v,
			    out_w );

			// Scale holonomic speeds to real-world one:
			//const double reduction = min(1.0, in_movement.speed / in_movement.PTG->getMax_V_inTPSpace());
			double reduction = in_movement.speed;
			if (reduction < 0.5)
				reduction = 0.5;

			// To scale:
			out_v*=reduction;
			out_w*=reduction;

			// Assure maximum speeds:
			if (fabs(out_v)>robotMax_V_mps)
			{
				// Scale:
				float F = fabs(robotMax_V_mps / out_v);
				out_v *= F;
				out_w *= F;
			}

			if (fabs(out_w)>DEG2RAD(robotMax_W_degps))
			{
				// Scale:
				float F = fabs((float)DEG2RAD(robotMax_W_degps) / out_w);
				out_v *= F;
				out_w *= F;
			}
		}
		m_timelogger.leave("navigationStep.STEP7_NonHolonomicMovement");
	}
	catch (std::exception &e)
	{
		m_timelogger.leave("navigationStep.STEP7_NonHolonomicMovement");
		printf_debug("[CReactiveNavigationSystem::STEP7_NonHolonomicMovement] Exception:");
		printf_debug((char*)(e.what()));
	}
	catch (...)
	{
		m_timelogger.leave("navigationStep.STEP7_NonHolonomicMovement");
		printf_debug("[CReactiveNavigationSystem::STEP7_NonHolonomicMovement] Unexpected exception!\n");
	}
}


/*************************************************************************
	 		Destructor
*************************************************************************/
CReactiveNavigationSystem::~CReactiveNavigationSystem()
{
	m_closing_navigator = true;

	// Wait to end of navigation (multi-thread...)
	m_critZoneNavigating.enter();
	m_critZoneNavigating.leave();

	// Just in case.
	m_robot.stop();

	mrpt::utils::delete_safe(logFile);

	// Free PTGs:
	for (size_t i=0;i<PTGs.size();i++)	delete PTGs[i];
	PTGs.clear();

	// Free holonomic method:
	mrpt::utils::delete_safe(holonomicMethod);
}



