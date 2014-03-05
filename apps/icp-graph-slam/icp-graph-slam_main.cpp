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
	See: http://www.mrpt.org/list-of-mrpt-apps/application-icp-graph-slam/
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

// ------------------------------------------------------
//				icp_graphslam_2D
//  rawlogFilename: If not empty, use that rawlog
//  instead of that in the config file.
// ------------------------------------------------------
void icp_graphslam_2D(const string & cfgFilename, const string & rawlogFilename)
{
	MRPT_START

	CConfigFile cfgFile(cfgFilename);

	// ------------------------------------------
	//			Load config from file:
	// ------------------------------------------
	const unsigned int rawlog_offset = cfgFile.read_int("icp_graphslam","rawlog_offset",0,  /*mandatory?*/ false);
	const string OUT_DIR             = cfgFile.read_string("icp_graphslam","logOutput_dir","icpgraphslam-log",  /*mandatory?*/ false);
	const int LOG_FREQUENCY          = cfgFile.read_int("icp_graphslam","LOG_FREQUENCY",5,  /*mandatory?*/ true);
	const bool  SAVE_POSE_LOG        = cfgFile.read_bool("icp_graphslam","SAVE_POSE_LOG", false,  /*mandatory?*/ true);
	const bool  SAVE_3D_SCENE        = cfgFile.read_bool("icp_graphslam","SAVE_3D_SCENE", false,  /*mandatory?*/ true);
	const bool  CAMERA_3DSCENE_FOLLOWS_ROBOT = cfgFile.read_bool("icp_graphslam","CAMERA_3DSCENE_FOLLOWS_ROBOT", true,  /*mandatory?*/ true);

	bool 	SHOW_PROGRESS_3D_REAL_TIME = false;
	int		SHOW_PROGRESS_3D_REAL_TIME_DELAY_MS = 0;
	bool 	SHOW_LASER_SCANS_3D = true;

	MRPT_LOAD_CONFIG_VAR( SHOW_PROGRESS_3D_REAL_TIME, bool,  cfgFile, "icp_graphslam");
	MRPT_LOAD_CONFIG_VAR( SHOW_LASER_SCANS_3D , bool,  cfgFile, "icp_graphslam");
	MRPT_LOAD_CONFIG_VAR( SHOW_PROGRESS_3D_REAL_TIME_DELAY_MS, int, cfgFile, "icp_graphslam");

	// ------------------------------------
	//		Constructor of SLAM object
	// ------------------------------------
	// The 2D-ICP graph-slam engine class: 
	typedef GraphSlamEngine< 
		CNetworkOfPoses2D, // Graph type
		f2f_match::GS_F2F_ICP_2D  // Match finder 
		> 
		my_graphslam_engine_t;

	// Instance of the GS Engine:
	my_graphslam_engine_t  graphslam_engine;

	MRPT_TODO("Load each module options")

//	mapBuilder.ICP_options.loadFromConfigFile( cfgFile, "icp_graphslam");
//	mapBuilder.ICP_options.dumpToConsole();

	// Prepare output directory:
	// --------------------------------
	mrpt::system::deleteFilesInDirectory(OUT_DIR);
	mrpt::system::createDirectory(OUT_DIR);

	// Create 3D window if requested:
	CDisplayWindow3DPtr	win3D;
#if MRPT_HAS_WXWIDGETS
	if (SHOW_PROGRESS_3D_REAL_TIME)
	{
		win3D = CDisplayWindow3D::Create("ICP-GRAPH-SLAM @ The MRPT project", 600, 500);
		win3D->setCameraZoom(20);
		win3D->setCameraAzimuthDeg(-45);
	}
#endif

	// ----------------------------------------------------------
	//						Map Building
	// ----------------------------------------------------------
	CPose2D odoPose(0,0,0);
	mrpt::utils::CTimeLogger timelog;

	CFileGZInputStream rawlogFile( rawlogFilename.c_str() );
	size_t rawlogEntry = 0;
	int    step = 0;

	for (;;)
	{
		// Exit? 
		if (os::kbhit())
		{
			char c = os::getch();
			if (c==27)
				break;
		}

		// Load action/observation pair from the rawlog:
		// --------------------------------------------------
		CActionCollectionPtr	action;
		CSensoryFramePtr		observations;
		CObservationPtr			observation;

		{
			mrpt::utils::CTimeLoggerEntry tle(timelog, "load_from_rawlog");
		
			if (! CRawlog::getActionObservationPairOrObservation( rawlogFile, action, observations, observation, rawlogEntry) )
				break; // file EOF
		}

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
			{
				mrpt::utils::CTimeLoggerEntry tle(timelog, "run_graphslam");
				if (isObsBasedRawlog)
						graphslam_engine.processObservation( observation );
				else	graphslam_engine.processActionObservation( *action, *observations );
			}

			// Get current robot pose:
            CPose2D robotPose;
			graphslam_engine.getCurrentPose(robotPose);
			cout << "Current global pose: " << robotPose << endl;

			const CPose3D curRobotPose = CPose3D(robotPose);

			// Save a 3D scene view of the mapping process:
			if (0==(step % LOG_FREQUENCY) || SAVE_3D_SCENE || win3D.present())
			{
				mrpt::utils::CTimeLoggerEntry tle(timelog, "create_3d_views");

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

				// The camera pointing to the current robot pose:
				if (CAMERA_3DSCENE_FOLLOWS_ROBOT)
				{
				    scene->enableFollowCamera(true);

					mrpt::opengl::CCamera &cam = view_map->getCamera();
					cam.setAzimuthDegrees(-45);
					cam.setElevationDegrees(45);
					cam.setPointingAt(robotPose);
				}

				// The topology of the graph:
				{
					TParametersDouble graphRenderParams;

					// Docs on these params: http://reference.mrpt.org/svn/group__mrpt__opengl__grp.html#ga30efc9f6fcb49801e989d174e0f65a61
					graphRenderParams["show_ID_labels"] = 1;
					graphRenderParams["show_ground_grid"] = 1;
					graphRenderParams["show_edges"] = 1;

					opengl::CSetOfObjectsPtr gl_graph = mrpt::opengl::graph_tools::graph_visualize( graphslam_engine.getGraph(), graphRenderParams);
					view->insert(gl_graph);
				}

				// Draw the robot path:
				//CPose3DPDFPtr posePDF =  mapBuilder.getCurrentPoseEstimation();
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
					CFileGZOutputStream	f( format( "%s/buildingmap_%05u.3Dscene",OUT_DIR.c_str(),step ));
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
		printf("\n--- Step: %u | Rawlog entry: %u | Map nodes: %u, edges: %u --------\n",
			step, static_cast<unsigned int>(rawlogEntry), 
			static_cast<unsigned int>(graphslam_engine.getGraph().nodeCount()), 
			static_cast<unsigned int>(graphslam_engine.getGraph().edgeCount()) );
	};

	// Save map:
	//CSimpleMap finalMap;
	//mapBuilder.getCurrentlyBuiltMap(finalMap);

	//str = format("%s/_finalmap_.simplemap",OUT_DIR.c_str());
	//printf("Dumping final map in binary format to: %s\n", str.c_str() );
	//mapBuilder.saveCurrentMapToFile(str);

	//CMultiMetricMap  *finalPointsMap = mapBuilder.getCurrentlyBuiltMetricMap();
	//str = format("%s/_finalmaps_.txt",OUT_DIR.c_str());
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
		if (argc<3 || showHelp )
		{
			printf("Usage: %s <config_file.ini> <dataset.rawlog>\n\n",argv[0]);
			if (!showHelp)
			{
				mrpt::system::pause();
				return -1;
			}
			else	return 0;
		}

		const string cfgFilename = string( argv[1] );
		ASSERT_FILE_EXISTS_(cfgFilename)

		string rawlogFilename = string(argv[2]);
		ASSERT_FILE_EXISTS_(rawlogFilename)

		// Run:
		icp_graphslam_2D(cfgFilename,rawlogFilename);

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
