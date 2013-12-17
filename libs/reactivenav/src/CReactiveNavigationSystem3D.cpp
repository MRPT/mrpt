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


using namespace mrpt;
using namespace mrpt::poses;
using namespace mrpt::math;
using namespace mrpt::utils;
using namespace mrpt::reactivenav;
using namespace std;


/*---------------------------------------------------------------
					Constructor
  ---------------------------------------------------------------*/
CReactiveNavigationSystem3D::CReactiveNavigationSystem3D(
	CReactiveInterfaceImplementation   &react_iterf_impl,
    bool					enableConsoleOutput,
    bool					enableLogToFile)
	:
	CAbstractPTGBasedReactive(react_iterf_impl),
	navigationEndEventSent       (false),
	holonomicMethod              (0),
	logFile                      (NULL),
	m_enableConsoleOutput        (enableConsoleOutput),
	m_init_done                  (false),
	nIteration                   (0),
	meanExecutionPeriod          (0.1f),
	m_timelogger                 (false), // default: disabled
	m_collisionGridsMustBeUpdated(true),
	meanExecutionTime            (0.1f),
	meanTotalExecutionTime       (0.1f),
	m_decimateHeadingEstimate    (0),
	m_closing_navigator          (false)
{
	WS_Obstacles_unsorted = CSimplePointsMap::Create();
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

	robotName = ini.read_string("ROBOT_CONFIG","Name", "", true );

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

	DIST_TO_TARGET_FOR_SENDING_EVENT = ini.read_float("NAVIGATION_CONFIG", "DIST_TO_TARGET_FOR_SENDING_EVENT", 0.4, false);

	ini.read_vector("NAVIGATION_CONFIG", "weights", vector<float> (0), weights, 1);
	ASSERT_(weights.size()==6);

	badNavAlarm_AlarmTimeout = ini.read_float("NAVIGATION_CONFIG","ALARM_SEEMS_NOT_APPROACHING_TARGET_TIMEOUT", 10, false);

	//m_reactiveparam.m_reload_ptgfiles = ini.read_bool("NAVIGATION_CONFIG","RELOAD_PTGFILES", 1, true);


	// Load PTGs from file:
	// ---------------------------------------------

	unsigned int num_ptgs, num_alfas; // levels = m_robotShape.heights.size()
	vector <CPolygon> shape;
	vector <double> x,y;
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


			m_timelogger.enter("PTG.simulateTrajectories");
			m_ptgmultilevel[j-1].PTGs[i-1]->simulateTrajectories(num_alfas,75, refDistance, 600, 0.01f, 0.015f);
			//Arguments -> n_alfas, max.tim, max.dist (ref_distance), max.n, diferencial_t, min_dist
			m_timelogger.leave("PTG.simulateTrajectories");

			printf_debug("...OK!\n");

		}
	}

	//Load holonomic method params
	this->loadHolonomicMethodConfig(ini,"NAVIGATION_CONFIG");


	// Show configuration parameters:
	// -------------------------------------------------------------------
	printf_debug("\tLOADED CONFIGURATION:\n");
	printf_debug("-------------------------------------------------------------\n");
	
	printf_debug("  Robot name \t\t\t=%s\n",robotName.c_str());
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

/*---------------------------------------------------------------
	  				performNavigationStep

  This method implements the 3D reactive navigation algorithm.
  It is executed periodically only if the robot is in
  "NAVIGATING" state.
  ---------------------------------------------------------------*/
void  CReactiveNavigationSystem3D::performNavigationStep()
{
	vector <mrpt::slam::CSimplePointsMap>		WS_Obstacles_inlevels;
	CLogFileRecord								newLogRec;
	std::vector<CHolonomicLogFileRecordPtr>		HLFRs;
	poses::CPoint2D								relTarget;		// The target point, relative to current robot pose.
	poses::CPose2D								curPose;
	float										targetDist;
	float										curVL;			// in m/s
	float										curW;			// in rad/s
	int											nSelectedPTG;
	THolonomicMovement							selectedHolonomicMovement;

	HLFRs.resize(m_ptgmultilevel.size());
	newLogRec.infoPerPTG.resize(m_ptgmultilevel.size());

	//float cur_approx_heading_dir = 0;

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
		targetDist = curPose.distance2DTo( m_navigationParams.target.x, m_navigationParams.target.y );


		// Should "End of navigation" event be sent??
		if (!navigationEndEventSent && targetDist < DIST_TO_TARGET_FOR_SENDING_EVENT)
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
		//if (targetDist < badNavAlarm_minDistTarget )
		//{
		//	badNavAlarm_minDistTarget = targetDist;
		//	badNavAlarm_lastMinDistTime =  system::getCurrentTime();
		//}
		//else
		//{
		//	// Too much time have passed?
		//	if ( system::timeDifference( badNavAlarm_lastMinDistTime, system::getCurrentTime() ) > badNavAlarm_AlarmTimeout)
		//	{
		//		std::cout << "\n--------------------------------------------\nWARNING: Timeout for approaching toward the target expired!! Aborting navigation!! \n---------------------------------\n";

		//		m_navigationState = NAV_ERROR;
		//		m_timelogger.leave("navigationStep");
		//		return;
		//	}
		//}


		// Compute target location relative to current robot pose:
		// ---------------------------------------------------------------------
		relTarget = CPoint2D(m_navigationParams.target) - curPose;

		// STEP1: Collision Grids Builder. (Only once using the "initialize" method)
		// -----------------------------------------------------------------------------
		// STEP1_CollisionGridsBuilder();

		// STEP2: Load the obstacles and sort them in height bands.
		// -----------------------------------------------------------------------------
		if (! STEP2_LoadAndSortObstacles( 	WS_Obstacles_inlevels ) )
		{
			printf_debug("Warning: Error while loading and sorting the obstacles. Robot will be stopped.\n");
			m_robot.stop();
			m_navigationState = NAV_ERROR;
			m_timelogger.leave("navigationStep");
			return;
		}


		// Start timer
		executionTime.Tic();

		// For some behaviors:
		//  If set to true, "new_cmd_v" & "new_cmd_w" must be set to the desired values:
		bool skipNormalReactiveNavigation = false;


		if (! skipNormalReactiveNavigation )
		{
			// Clip obstacles out of the reactive method range:
			//CPoint2D    dumm(0,0);
			//WS_Obstacles.clipOutOfRange( dumm, refDistance+1.5f );


			//  STEP3: Build TP-Obstacles and transform target location into TP-Space
			// -----------------------------------------------------------------------------

			STEP3_WSpaceToTPSpace(WS_Obstacles_inlevels, relTarget);



			//  STEP4: Holonomic navigation method
			// -----------------------------------------------------------------------------

			STEP4_HolonomicMethod(HLFRs);



			// STEP5: Evaluate each movement to assign them a "evaluation" value.
			// ---------------------------------------------------------------------

			// For each PTG:
			for (unsigned int i=0; i<m_ptgmultilevel.size(); i++)
			{
				m_ptgmultilevel[i].holonomicmov.PTG = m_ptgmultilevel[i].PTGs[0];
                CPoint2D TP_Target_i = CPoint2D(m_ptgmultilevel[i].TP_Target);
				STEP5_PTGEvaluator(	m_ptgmultilevel[i].holonomicmov,
									m_ptgmultilevel[i].TPObstacles,
									relTarget,
									TP_Target_i,
									newLogRec.infoPerPTG[i]);
			}



			// STEP6: Selects the best movement
			// ---------------------------------------------------------------------

			STEP6_Selector( selectedHolonomicMovement, nSelectedPTG );



			// Compute the approximate heading direction
			// ----------------------------------------------------------------
			//{
			//	if (++m_decimateHeadingEstimate>10)
			//	{
			//		m_decimateHeadingEstimate=0;

			//		float x=1;
			//		float y=0;
			//		float p,t;
			//		selectedHolonomicMovement.PTG->getCPointWhen_d_Is(2.0, selectedHolonomicMovement.PTG->alpha2index(selectedHolonomicMovement.direction),x,y,p,t);

			//		cur_approx_heading_dir = atan2(y,x);
			//		m_robot.notifyHeadingDirection(cur_approx_heading_dir);
			//	}
			//}


			// STEP7: Get the non-holonomic movement command.
			// ---------------------------------------------------------------------

			STEP7_GenerateSpeedCommands( selectedHolonomicMovement );



		} // end of "!skipNormalReactiveNavigation"


		// ---------------------------------------------------------------------
		//				SEND MOVEMENT COMMAND TO THE ROBOT
		// ---------------------------------------------------------------------
		if ( new_cmd_v == 0.0 && new_cmd_w == 0.0 )
		{
			m_robot.stop();
		}
		else
		{
			if ( !m_robot.changeSpeeds( new_cmd_v, new_cmd_w ) )
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
		           (double)new_cmd_v,
		           (double)RAD2DEG( new_cmd_w ) );

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
		// STEP8: Generate log record
		// ---------------------------------------

		m_timelogger.enter("navigationStep.populate_log_info");

		CSimplePointsMap auxpointmap;

		//Include the points of all levels (this could be improved depending on STEP2)
		for (unsigned int i=0; i < WS_Obstacles_inlevels.size(); i++)
			auxpointmap += *WS_Obstacles_inlevels[i].getAsSimplePointsMap();

		newLogRec.WS_Obstacles				= auxpointmap;
		newLogRec.robotOdometryPose			= curPose;
		newLogRec.WS_target_relative		= relTarget;
		newLogRec.v							= new_cmd_v;
		newLogRec.w							= new_cmd_w;
		newLogRec.nSelectedPTG				= nSelectedPTG;
		newLogRec.executionTime				= executionTimeValue;
		newLogRec.actual_v					= curVL;
		newLogRec.actual_w					= curW;
		newLogRec.estimatedExecutionPeriod	= meanExecutionPeriod;
		newLogRec.timestamp					= tim_start_iteration;
		newLogRec.nPTGs						= m_ptgmultilevel.size();
		newLogRec.navigatorBehavior			= holonomicMethodSel;


		//Polygons of each height level are drawn (but they are all shown connected...)
		if (newLogRec.robotShape_x.size() == 0)
		{
			size_t nVerts = 0;
			TPoint2D paux;
			size_t cuenta = 0;
			for (unsigned int i=0; i < m_robotShape.heights.size(); i++)
				nVerts += m_robotShape.polygons[i].size() + 1;
			if (size_t(newLogRec.robotShape_x.size()) != nVerts)
			{
				newLogRec.robotShape_x.resize(nVerts);
				newLogRec.robotShape_y.resize(nVerts);
			}
			for (unsigned int i=0; i<m_robotShape.heights.size(); i++)
			{
				for (unsigned int j=0; j<m_robotShape.polygons[i].size(); j++)
				{
					paux = m_robotShape.polygons[i][j];
					newLogRec.robotShape_x[cuenta]= paux.x;
					newLogRec.robotShape_y[cuenta]= paux.y;
					cuenta++;
				}
				paux = m_robotShape.polygons[i][0];
				newLogRec.robotShape_x[cuenta]= paux.x;
				newLogRec.robotShape_y[cuenta]= paux.y;
				cuenta++;
			}
		}


		// For each PTG:
		if (!skipNormalReactiveNavigation)
		{
			for (size_t i = 0; i< m_ptgmultilevel.size(); i++)
			{
				//sizeobs = m_ptgmultilevel[i].TPObstacles.size();
				metaprogramming::copy_container_typecasting(m_ptgmultilevel[i].TPObstacles, newLogRec.infoPerPTG[i].TP_Obstacles);
				newLogRec.infoPerPTG[i].PTG_desc					= m_ptgmultilevel[i].PTGs[0]->getDescription();
				newLogRec.infoPerPTG[i].TP_Target					= m_ptgmultilevel[i].TP_Target;
				newLogRec.infoPerPTG[i].timeForTPObsTransformation	= 0;
				newLogRec.infoPerPTG[i].timeForHolonomicMethod		= 0;
				newLogRec.infoPerPTG[i].HLFR						= HLFRs[i];
				newLogRec.infoPerPTG[i].desiredDirection			= m_ptgmultilevel[i].holonomicmov.direction;
				newLogRec.infoPerPTG[i].desiredSpeed				= m_ptgmultilevel[i].holonomicmov.speed;
				newLogRec.infoPerPTG[i].evaluation					= m_ptgmultilevel[i].holonomicmov.evaluation;
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
     -> Build the collision grids
*************************************************************************/
void CReactiveNavigationSystem3D::STEP1_CollisionGridsBuilder()
{
	try
	{
		if (m_collisionGridsMustBeUpdated)
		{
			m_collisionGridsMustBeUpdated = false;

			m_timelogger.enter("build_PTG_collision_grids");

			for (unsigned int j=0; j<m_ptgmultilevel.size(); j++)
			{
				for (unsigned int i=0; i<m_robotShape.heights.size(); i++)
				{
					build_PTG_collision_grid3D(m_ptgmultilevel[j].PTGs[i], m_robotShape.polygons[i], i+1, j+1, m_enableConsoleOutput /*VERBOSE*/ );
				}
			}

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

                         STEP2_SortObstacles

		Load the obstacles and sort them accorging to the height
				sections used to model the robot.

*************************************************************************/
bool CReactiveNavigationSystem3D::STEP2_LoadAndSortObstacles(
	vector <mrpt::slam::CSimplePointsMap>		&out_obstacles)
{
	try
	{
		//-------------------------------------------------------------------
		// The user must implement its own method to load the obstacles from
		// either sensor measurements or simulators (m_robot.senseObstacles(...))
		// Data have to be subsequently sorted in height bands according to the
		// height sections of the robot.
		//-------------------------------------------------------------------

		m_timelogger.enter("navigationStep.STEP2_LoadAndSortObstacle");

		m_robot.senseObstacles( *WS_Obstacles_unsorted );

		unsigned int cont;
		bool classified;
		float h;
		TPoint3D paux;
		out_obstacles.resize(m_robotShape.heights.size());

		for (unsigned int j=0; j<WS_Obstacles_unsorted->getPointsCount(); j++)
		{
			WS_Obstacles_unsorted->getPoint(j,paux.x,paux.y,paux.z);
			classified  = 0;
			cont  = 0;
			h = 0;
			while (classified == 0)
			{
				if (paux.z < 0.01)
					classified = 1;
				else
				{
					h += m_robotShape.heights[cont];
					if (paux.z < h)
					{
						out_obstacles[cont].insertPoint(paux.x,paux.y,paux.z);
						classified = 1;
					}
					cont++;
					if ((cont == m_robotShape.heights.size())&&(classified == 0))
						classified = 1;
				}
			}
		}


		m_timelogger.leave("navigationStep.STEP2_LoadAndSortObstacle");

		return	1;
	}
	catch (std::exception &e)
	{
		printf_debug("[CReactiveNavigationSystem::STEP2_Load_Obstacles] Exception:");
		printf_debug((char*)(e.what()));
		return false;
	}
	catch (...)
	{
		printf_debug("[CReactiveNavigationSystem::STEP2_Load_Obstacles] Unexpected exception!\n");
		return false;
	}

}

/*************************************************************************

                      STEP3_ObstacleToTPObstacle

		Transform the obstacle into TP-Obstacles in TP-Spaces

*************************************************************************/
void CReactiveNavigationSystem3D::STEP3_WSpaceToTPSpace(
				vector <mrpt::slam::CSimplePointsMap>	&in_obstacles,
				CPoint2D								&relTarget)
{
	try
	{
		m_timelogger.enter("navigationStep.STEP3_WSpaceToTPSpace");

		size_t Ki,nObs;
		float invoperation;

		for (unsigned int a=0; a<m_ptgmultilevel.size(); a++)
		{
			//				Transform the obstacles
			//========================================================

			Ki = m_ptgmultilevel[a].PTGs[0]->getAlfaValuesCount();

			if ( static_cast<size_t>(m_ptgmultilevel[a].TPObstacles.size()) != Ki )
				m_ptgmultilevel[a].TPObstacles.resize(Ki);


			for (unsigned int k=0; k<Ki; k++)
			{
				// Initialized at max. distance
				m_ptgmultilevel[a].TPObstacles[k] = m_ptgmultilevel[a].PTGs[0]->refDistance;

				// If it turns more than 180deg, stop it.  (Better without this)
				//float phi = m_ptgmultilevel[a].PTGs[0]->GetCPathPoint_phi(k,m_ptgmultilevel[a].PTGs[0]->getPointsCountInCPath_k(k)-1);  //Last point orientation
				//if (fabs(phi) >= M_PI* 0.95 )
				//	m_ptgmultilevel[a].TPObstacles[k]= m_ptgmultilevel[a].PTGs[0]->GetCPathPoint_d(k,m_ptgmultilevel[a].PTGs[0]->getPointsCountInCPath_k(k)-1);
			}

			for (unsigned int j=0;j<m_robotShape.heights.size();j++)
			{
				nObs = in_obstacles[j].getPointsCount();

				for (unsigned int obs=0;obs<nObs;obs++)
				{
					float ox,oy;
					in_obstacles[j].getPoint(obs,ox,oy);

					const CParameterizedTrajectoryGenerator::TCollisionCell &cell = m_ptgmultilevel[a].PTGs[j]->m_collisionGrid.getTPObstacle(ox,oy);

					// Keep the minimum distance:
					for (CParameterizedTrajectoryGenerator::TCollisionCell::const_iterator i=cell.begin();i!=cell.end();i++)
						if ( i->second < m_ptgmultilevel[a].TPObstacles[i->first] )
							m_ptgmultilevel[a].TPObstacles[i->first] = i->second;
				}
			}

			// Distances in TP-Space are normalized to [0,1]
			invoperation = 1.0f/m_ptgmultilevel[a].PTGs[0]->refDistance;
			for (unsigned int k=0; k<Ki; k++)
				m_ptgmultilevel[a].TPObstacles[k] *= invoperation;

			//				Transform the target
			//========================================================

			int k;
			float d, alfa;
			for (unsigned int i=0; i<m_ptgmultilevel.size(); i++)
			{
				m_ptgmultilevel[i].PTGs[0]->lambdaFunction(relTarget[0], relTarget[1], k, d);
				alfa = m_ptgmultilevel[i].PTGs[0]->index2alpha(k);
				m_ptgmultilevel[i].TP_Target[0] = cos(alfa)*d;
				m_ptgmultilevel[i].TP_Target[1] = sin(alfa)*d;
			}

			m_timelogger.leave("navigationStep.STEP3_WSpaceToTPSpace");
		}
	}
	catch (std::exception &e)
	{
		printf("[CReactiveNavigationSystem::STEP3_ObstaclesToTPObstacles] Exception:");
		printf("%s", e.what());
	}
	catch (...)
	{
		cout << "\n[CReactiveNavigationSystem::STEP3_ObstaclesToTPObstacles] Unexpected exception!:\n" << endl;
	}
}


/*************************************************************************

                             STEP4_HolonomicMethod

*************************************************************************/
void CReactiveNavigationSystem3D::STEP4_HolonomicMethod(vector <CHolonomicLogFileRecordPtr>  &in_HLFR )
{
	try
	{
		m_timelogger.enter("navigationStep.STEP4_HolonomicMethod");

		for (unsigned int i=0; i<m_ptgmultilevel.size(); i++)
		{
			holonomicMethod[i]->navigate(	m_ptgmultilevel[i].TP_Target,
										m_ptgmultilevel[i].TPObstacles,
										m_ptgmultilevel[i].PTGs[0]->getMax_V_inTPSpace(),
										m_ptgmultilevel[i].holonomicmov.direction,
										m_ptgmultilevel[i].holonomicmov.speed,
										in_HLFR[i]);
		}

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
void CReactiveNavigationSystem3D::STEP5_PTGEvaluator(
    THolonomicMovement			&in_holonomicMovement,
    vector_double				&in_TPObstacles,
    mrpt::poses::CPoint2D		&WS_Target,
    mrpt::poses::CPoint2D		&TP_Target,
    CLogFileRecord::TInfoPerPTG	&log)
{
	try
	{
		m_timelogger.enter("navigationStep.STEP5_Evaluator");

		float	a;
		float	factor1,factor2,factor3,factor4,factor5,factor6;

		if (TP_Target.x()!=0 || TP_Target.y()!=0)
			a = atan2( TP_Target.y(), TP_Target.x());
		else	a = 0;


		const int		TargetSector = in_holonomicMovement.PTG->alpha2index( a );
		const double	TargetDist = TP_Target.norm();
		const int		kDirection = in_holonomicMovement.PTG->alpha2index( in_holonomicMovement.direction );
		const double	refDist	   = in_holonomicMovement.PTG->refDistance;

		// Coordinates of the trajectory end for the given PTG and "alpha":
		float	x,y,phi,t,d;
		d = min( in_TPObstacles[ kDirection ], 0.90f*TargetDist);
		in_holonomicMovement.PTG->getCPointWhen_d_Is( d, kDirection,x,y,phi,t );

		// Factor 1: Free distance for the chosen PTG and "alpha" in the TP-Space:
		// ----------------------------------------------------------------------
		factor1 = in_TPObstacles[kDirection];


		// Factor 2: Distance in sectors:
		// -------------------------------------------
		float   dif = fabs(((float)( TargetSector - kDirection )));
		float	nSectors = (float)in_TPObstacles.size();
		if ( dif > (0.5f*nSectors)) dif = nSectors - dif;
		factor2 = exp(-square( dif / (in_TPObstacles.size()/3.0f))) ;

		// Factor 3: Angle between the robot at the end of the chosen trajectory and the target
		// -------------------------------------------------------------------------------------
		float   t_ang = atan2( WS_Target.y() - y, WS_Target.x() - x );
		t_ang -= phi;

		while (t_ang> M_PI)  t_ang-=(float)M_2PI;
		while (t_ang<-M_PI)  t_ang+=(float)M_2PI;

		factor3 = exp(-square( t_ang / (float)(0.5f*M_PI)) );

		// Factor4:		Decrease in euclidean distance between (x,y) and the target:
		//  Moving away of the target is negatively valued
		// ---------------------------------------------------------------------------
		float dist_eucl_final = WS_Target.distance2DTo(x,y);
		float dist_eucl_now   = WS_Target.norm();

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


		// Factor6: Fast when free space
		// -----------------------------------------------------
		float aver_obs = 0;
		for (int i=0; i<in_TPObstacles.size(); i++)
			aver_obs += in_TPObstacles[i];

		aver_obs = aver_obs/in_TPObstacles.size();

		factor6 = aver_obs*want_v;


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

		if (in_holonomicMovement.speed == 0)
		{
			// If no movement has been found -> the worst evaluation:
			in_holonomicMovement.evaluation = 0;
		}
		else
		{
			// Sum: two cases: *************************************I'm not sure about this...
			if (dif<2	&&											// Heading the target
			        in_TPObstacles[kDirection]*0.95f>TargetDist 	// and free space towards the target
			   )
			{
				//	Direct path to target:
				//	in_holonomicMovement.evaluation = 1.0f + (1 - TargetDist) + factor5 * weight5 + factor6*weight6;
				in_holonomicMovement.evaluation = 1.0f + (1 - t/15.0f) + factor5 * weights[4] + factor6*weights[5];
			}
			else
			{
				// General case:
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
void CReactiveNavigationSystem3D::STEP6_Selector(
    THolonomicMovement					&out_selected,
    int									&out_nSelectedPTG)
{
	m_timelogger.enter("navigationStep.STEP6_Selector");

	// If nothing better is found, there isn't any possible movement:
	out_selected.direction= 0;
	out_selected.speed = 0;
	out_selected.PTG = NULL;
	out_selected.evaluation= 0;
	out_nSelectedPTG = 0;

	// Choose the best evaluated trajectory:
	for (unsigned int i=0; i<m_ptgmultilevel.size();i++)
	{
		float ev = m_ptgmultilevel[i].holonomicmov.evaluation;
		if ( ev > out_selected.evaluation )
		{
			out_selected = m_ptgmultilevel[i].holonomicmov;
			out_selected.evaluation = ev;
			out_nSelectedPTG = i;		//It starts at 0
		}
	}

	m_timelogger.leave("navigationStep.STEP6_Selector");
}

/*************************************************************************

						STEP7_NonHolonomicMovement

*************************************************************************/
void CReactiveNavigationSystem3D::STEP7_GenerateSpeedCommands( THolonomicMovement	&in_movement)
{
	try
	{
		m_timelogger.enter("navigationStep.STEP7_NonHolonomicMovement");

		last_cmd_v = new_cmd_v;
		last_cmd_w = new_cmd_w;

		if (in_movement.speed == 0)
		{
			// The robot will stop:
			new_cmd_v = new_cmd_w = 0;
		}
		else
		{
			// Take the normalized movement command:
			in_movement.PTG->directionToMotionCommand(
			    in_movement.PTG->alpha2index( in_movement.direction ),
			    new_cmd_v,
			    new_cmd_w );

			// Scale holonomic speeds to real-world one:
			//const double reduction = min(1.0, in_movement.speed / sqrt( square(new_cmd_v) + square(r*new_cmd_w)));
			double reduction = in_movement.speed;
			if (reduction < 0.5)
				reduction = 0.5;

			// To scale:
			new_cmd_v*=reduction;
			new_cmd_w*=reduction;

			// Assure maximum speeds:
			if (fabs(new_cmd_v) > robotMax_V_mps)
			{
				// Scale:
				float F = fabs(robotMax_V_mps / new_cmd_v);
				new_cmd_v *= F;
				new_cmd_w *= F;
			}

			if (fabs(new_cmd_w) > DEG2RAD(robotMax_W_degps))
			{
				// Scale:
				float F = fabs((float)DEG2RAD(robotMax_W_degps) / new_cmd_w);
				new_cmd_v *= F;
				new_cmd_w *= F;
			}

			//First order low-pass filter
			float alfa = meanExecutionPeriod/( meanExecutionPeriod + SPEEDFILTER_TAU);
			new_cmd_v = alfa*new_cmd_v + (1-alfa)*last_cmd_v;
			new_cmd_w = alfa*new_cmd_w + (1-alfa)*last_cmd_w;

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
CReactiveNavigationSystem3D::~CReactiveNavigationSystem3D()
{
	this->preDestructor();

	// Free PTGs:
	m_ptgmultilevel.clear();
}


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
