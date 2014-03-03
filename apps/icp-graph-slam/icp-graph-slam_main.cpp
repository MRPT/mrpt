/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2014, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

/*---------------------------------------------------------------
	APPLICATION: ICP-based Graph SLAM
	AUTHOR: Jose Luis Blanco Claraco <jlblanco@ctima.uma.es>
  ---------------------------------------------------------------*/

#include <mrpt/graphslam.h>
#include <mrpt/graphslam/GraphSlamEngine.h>

#include <mrpt/opengl.h>
#include <mrpt/obs.h>
#include <mrpt/utils.h>
#include <mrpt/system/filesystem.h>
#include <mrpt/system/threads.h> // for sleep()
#include <mrpt/opengl/CPlanarLaserScan.h>  // This class lives in the lib [mrpt-maps] and must be included by hand
#include <mrpt/gui.h>

using namespace mrpt;
using namespace mrpt::slam;
using namespace mrpt::opengl;
using namespace mrpt::gui;
using namespace mrpt::system;
using namespace mrpt::math;
using namespace mrpt::utils;
using namespace mrpt::graphs;
using namespace mrpt::graphslam;
using namespace std;

// The 2D-ICP graph-slam engine class: 
typedef GraphSlamEngine< 
	CNetworkOfPoses2D, // Graph type
	f2f_match::GS_F2F_ICP_2D  // Match finder 
	> 
	my_graphslam_engine_t;

// Instance of the GS Engine:
my_graphslam_engine_t  graphslam_engine;


// ------------------------------------------------------
//				icp_graphslam_2D
//  override_rawlog_file: If not empty, use that rawlog
//  instead of that in the config file.
// ------------------------------------------------------
void icp_graphslam_2D(const string &INI_FILENAME, const string &override_rawlog_file)
{
	MRPT_START

	CConfigFile				iniFile(INI_FILENAME);

	// ------------------------------------------
	//			Load config from file:
	// ------------------------------------------
	const string RAWLOG_FILE			 = !override_rawlog_file.empty() ? override_rawlog_file : iniFile.read_string("MappingApplication","rawlog_file","",  /*Force existence:*/ true);
	const unsigned int rawlog_offset		 = iniFile.read_int("MappingApplication","rawlog_offset",0,  /*Force existence:*/ true);
	const string OUT_DIR_STD			 = iniFile.read_string("MappingApplication","logOutput_dir","log_out",  /*Force existence:*/ true);
	const int LOG_FREQUENCY		 = iniFile.read_int("MappingApplication","LOG_FREQUENCY",5,  /*Force existence:*/ true);
	const bool  SAVE_POSE_LOG		 = iniFile.read_bool("MappingApplication","SAVE_POSE_LOG", false,  /*Force existence:*/ true);
	const bool  SAVE_3D_SCENE        = iniFile.read_bool("MappingApplication","SAVE_3D_SCENE", false,  /*Force existence:*/ true);
	const bool  CAMERA_3DSCENE_FOLLOWS_ROBOT = iniFile.read_bool("MappingApplication","CAMERA_3DSCENE_FOLLOWS_ROBOT", true,  /*Force existence:*/ true);

	bool 	SHOW_PROGRESS_3D_REAL_TIME = false;
	int		SHOW_PROGRESS_3D_REAL_TIME_DELAY_MS = 0;
	bool 	SHOW_LASER_SCANS_3D = true;

	MRPT_LOAD_CONFIG_VAR( SHOW_PROGRESS_3D_REAL_TIME, bool,  iniFile, "MappingApplication");
	MRPT_LOAD_CONFIG_VAR( SHOW_LASER_SCANS_3D , bool,  iniFile, "MappingApplication");
	MRPT_LOAD_CONFIG_VAR( SHOW_PROGRESS_3D_REAL_TIME_DELAY_MS, int, iniFile, "MappingApplication");

	const char* OUT_DIR = OUT_DIR_STD.c_str();

	// ------------------------------------
	//		Constructor of ICP-SLAM object
	// ------------------------------------
//	CMetricMapBuilderICP mapBuilder;

//	mapBuilder.ICP_options.loadFromConfigFile( iniFile, "MappingApplication");
//	mapBuilder.ICP_params.loadFromConfigFile ( iniFile, "ICP");
	//mapBuilder.ICP_params.dumpToConsole();
	//mapBuilder.ICP_options.dumpToConsole();

	// Construct the maps with the loaded configuration.
//	mapBuilder.initialize();

	// ---------------------------------
	//   CMetricMapBuilder::TOptions
	// ---------------------------------
//	mapBuilder.options.verbose = true;
    

	// Checks:
	ASSERT_(!RAWLOG_FILE.empty())
	ASSERT_FILE_EXISTS_(RAWLOG_FILE)

	CTicTac								tictac,tictacGlobal,tictac_JH;
	int									step = 0;
	float								t_exec;


	size_t						rawlogEntry = 0;
	CFileGZInputStream					rawlogFile( RAWLOG_FILE.c_str() );


	// Prepare output directory:
	// --------------------------------
	mrpt::system::deleteFilesInDirectory(OUT_DIR);
	mrpt::system::createDirectory(OUT_DIR);

	// Open log files:
	// ----------------------------------
	CFileOutputStream  f_log(format("%s/log_times.txt",OUT_DIR));
	CFileOutputStream  f_path(format("%s/log_estimated_path.txt",OUT_DIR));
	CFileOutputStream  f_pathOdo(format("%s/log_odometry_path.txt",OUT_DIR));


	// Create 3D window if requested:
	CDisplayWindow3DPtr	win3D;
#if MRPT_HAS_WXWIDGETS
	if (SHOW_PROGRESS_3D_REAL_TIME)
	{
		win3D = CDisplayWindow3D::Create("ICP-SLAM @ MRPT C++ Library", 600, 500);
		win3D->setCameraZoom(20);
		win3D->setCameraAzimuthDeg(-45);
	}
#endif

	// ----------------------------------------------------------
	//						Map Building
	// ----------------------------------------------------------
	CPose2D					odoPose(0,0,0);

	tictacGlobal.Tic();
	for (;;)
	{
		CActionCollectionPtr	action;
		CSensoryFramePtr		observations;
		CObservationPtr			observation;

		if (os::kbhit())
		{
			char c = os::getch();
			if (c==27)
				break;
		}

		// Load action/observation pair from the rawlog:
		// --------------------------------------------------
		if (! CRawlog::getActionObservationPairOrObservation( rawlogFile, action, observations, observation, rawlogEntry) )
			break; // file EOF

		const bool isObsBasedRawlog = observation.present();
		std::vector<mrpt::slam::CObservation2DRangeScanPtr> lst_current_laser_scans;   // Just for drawing in 3D views

		if (rawlogEntry>=rawlog_offset)
		{
			// Update odometry:
			if (isObsBasedRawlog)
			{
				static CPose2D lastOdo;
				static bool firstOdo = true;
				if (IS_CLASS(observation,CObservationOdometry))
				{
					CObservationOdometryPtr o = CObservationOdometryPtr(observation);
					if (!firstOdo)
						odoPose = odoPose + (o->odometry - lastOdo);

					lastOdo=o->odometry;
					firstOdo=false;
				}
			}
			else
			{
				CActionRobotMovement2DPtr act= action->getBestMovementEstimation();
				if (act)
					odoPose = odoPose + act->poseChange->getMeanVal();
			}

			// Build list of scans:
			if (SHOW_LASER_SCANS_3D)
			{
				// Rawlog in "Observation-only" format:
				if (isObsBasedRawlog)
				{
					if (IS_CLASS(observation,CObservation2DRangeScan))
					{
						lst_current_laser_scans.push_back( CObservation2DRangeScanPtr(observation) );
					}
				}
				else
				{
					// Rawlog in the Actions-SF format:
					for (size_t i=0; ; i++)
					{
						CObservation2DRangeScanPtr new_obs = observations->getObservationByClass<CObservation2DRangeScan>(i);
						if (!new_obs)
						     break; // There're no more scans
						else lst_current_laser_scans.push_back( new_obs );
					}
				}
			}


			// Execute:
			// ----------------------------------------
			tictac.Tic();
			//if (isObsBasedRawlog)
			//		mapBuilder.processObservation( observation );
			//else	mapBuilder.processActionObservation( *action, *observations );
			t_exec = tictac.Tac();
			printf("Map building executed in %.03fms\n", 1000.0f*t_exec );

			// Get current robot pose:
            CPose3D robotPose;
			//mapBuilder.getCurrentPoseEstimation()->getMean(robotPose);

			// Save a 3D scene view of the mapping process:
			if (0==(step % LOG_FREQUENCY) || (SAVE_3D_SCENE || win3D.present()))
			{

				COpenGLScenePtr		scene = COpenGLScene::Create();

                COpenGLViewportPtr view=scene->getViewport("main");
                ASSERT_(view);

                COpenGLViewportPtr view_map = scene->createViewport("mini-map");
                view_map->setBorderSize(2);
                view_map->setViewportPosition(0.01,0.01,0.35,0.35);
                view_map->setTransparent(false);

				{
					mrpt::opengl::CCamera &cam = view_map->getCamera();
					cam.setAzimuthDegrees(-90);
					cam.setElevationDegrees(90);
					cam.setPointingAt(robotPose);
					cam.setZoomDistance(20);
					cam.setOrthogonal();
				}

				// The ground:
				mrpt::opengl::CGridPlaneXYPtr groundPlane = mrpt::opengl::CGridPlaneXY::Create(-200,200,-200,200,0,5);
				groundPlane->setColor(0.4,0.4,0.4);
				view->insert( groundPlane );
				view_map->insert( CRenderizablePtr( groundPlane) ); // A copy

				// The camera pointing to the current robot pose:
				if (CAMERA_3DSCENE_FOLLOWS_ROBOT)
				{
				    scene->enableFollowCamera(true);

					mrpt::opengl::CCamera &cam = view_map->getCamera();
					cam.setAzimuthDegrees(-45);
					cam.setElevationDegrees(45);
					cam.setPointingAt(robotPose);
				}

				// The maps:
				{
					//opengl::CSetOfObjectsPtr obj = opengl::CSetOfObjects::Create();
					//mostLikMap->getAs3DObject( obj );
					//view->insert(obj);

					//// Only the point map:
					//opengl::CSetOfObjectsPtr ptsMap = opengl::CSetOfObjects::Create();
					//if (mostLikMap->m_pointsMaps.size())
					//{
     //                   mostLikMap->m_pointsMaps[0]->getAs3DObject(ptsMap);
     //                   view_map->insert( ptsMap );
					//}
				}

				// Draw the robot path:
				//CPose3DPDFPtr posePDF =  mapBuilder.getCurrentPoseEstimation();
				CPose3D  curRobotPose;
				//posePDF->getMean(curRobotPose);
				//{
				//	opengl::CSetOfObjectsPtr obj = opengl::stock_objects::RobotPioneer();
				//	obj->setPose( curRobotPose );
				//	view->insert(obj);
				//}
				//{
				//	opengl::CSetOfObjectsPtr obj = opengl::stock_objects::RobotPioneer();
				//	obj->setPose( curRobotPose );
				//	view_map->insert( obj );
				//}

				// Draw laser scanners in 3D:
				if (SHOW_LASER_SCANS_3D)
				{
					for (size_t i=0;i<lst_current_laser_scans.size();i++)
					{
						// Create opengl object and load scan data from the scan observation:
						opengl::CPlanarLaserScanPtr obj = opengl::CPlanarLaserScan::Create();
						obj->setScan(*lst_current_laser_scans[i]);
						obj->setPose( curRobotPose );
						obj->setSurfaceColor(1.0f,0.0f,0.0f, 0.5f);
						// inser into the scene:
						view->insert(obj);
					}
				}

				// Save as file:
				if (0==(step % LOG_FREQUENCY) && SAVE_3D_SCENE)
				{
					CFileGZOutputStream	f( format( "%s/buildingmap_%05u.3Dscene",OUT_DIR,step ));
					f << *scene;
				}

				// Show 3D?
				if (win3D)
				{
					opengl::COpenGLScenePtr &ptrScene = win3D->get3DSceneAndLock();
					ptrScene = scene;

					win3D->unlockAccess3DScene();

					// Move camera:
					win3D->setCameraPointingToPoint( curRobotPose.x(),curRobotPose.y(),curRobotPose.z() );

					// Update:
					win3D->forceRepaint();

					mrpt::system::sleep( SHOW_PROGRESS_3D_REAL_TIME_DELAY_MS );
				}
			}


			// Save the robot estimated pose for each step:
			//f_path.printf("%i %f %f %f\n",
			//	step,
			//	mapBuilder.getCurrentPoseEstimation()->getMeanVal().x(),
			//	mapBuilder.getCurrentPoseEstimation()->getMeanVal().y(),
			//	mapBuilder.getCurrentPoseEstimation()->getMeanVal().yaw() );

			//f_pathOdo.printf("%i %f %f %f\n",step,odoPose.x(),odoPose.y(),odoPose.phi());

		} // end of if "rawlog_offset"...

		step++;
		printf("\n---------------- STEP %u | RAWLOG ENTRY %u ----------------\n",step, (unsigned)rawlogEntry);
	};

	printf("\n---------------- END!! (total time: %.03f sec) ----------------\n",tictacGlobal.Tac());

	// Save map:
	//CSimpleMap finalMap;
	//mapBuilder.getCurrentlyBuiltMap(finalMap);

	//str = format("%s/_finalmap_.simplemap",OUT_DIR);
	//printf("Dumping final map in binary format to: %s\n", str.c_str() );
	//mapBuilder.saveCurrentMapToFile(str);

	//CMultiMetricMap  *finalPointsMap = mapBuilder.getCurrentlyBuiltMetricMap();
	//str = format("%s/_finalmaps_.txt",OUT_DIR);
	//printf("Dumping final metric maps to %s_XXX\n", str.c_str() );
	//finalPointsMap->saveMetricMapRepresentationToFile( str );

	if (win3D)
		win3D->waitForKey();

	MRPT_END
}


// ------------------------------------------------------
//						MAIN
// ------------------------------------------------------
int main(int argc, char **argv)
{
	try
	{
		bool showHelp    = argc>1 && !os::_strcmp(argv[1],"--help");
		bool showVersion = argc>1 && !os::_strcmp(argv[1],"--version");

		printf(" icp-graph-slam - Part of the MRPT\n");
		printf(" MRPT C++ Library: %s - BUILD DATE %s\n", MRPT_getVersion().c_str(), MRPT_getCompilationDate().c_str());

		if (showVersion)
			return 0;	// Program end

		printf("-------------------------------------------------------------------\n");

		// Process arguments:
		if (argc<2 || showHelp )
		{
			printf("Usage: %s <config_file.ini> [<dataset.rawlog>]\n\n",argv[0]);
			if (!showHelp)
			{
				mrpt::system::pause();
				return -1;
			}
			else	return 0;
		}

		const string INI_FILENAME = string( argv[1] );
		ASSERT_FILE_EXISTS_(INI_FILENAME)

		string override_rawlog_file;
		if (argc>=3)
			override_rawlog_file = string(argv[2]);

		// Run:
		icp_graphslam_2D(INI_FILENAME,override_rawlog_file);

		//pause();
		return 0;
	}
	catch (exception &e)
	{
		setConsoleColor(CONCOL_RED,true);
		cerr << "Program finished for an exception!!" << endl;
		setConsoleColor(CONCOL_NORMAL,true);

		cerr << e.what() << endl;

		mrpt::system::pause();
		return -1;
	}
	catch (...)
	{
		setConsoleColor(CONCOL_RED,true);
		cerr << "Program finished for an untyped exception!!" << endl;
		setConsoleColor(CONCOL_NORMAL,true);

		mrpt::system::pause();
		return -1;
	}
}
