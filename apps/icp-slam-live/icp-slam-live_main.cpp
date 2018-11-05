/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */

/*---------------------------------------------------------------
	APPLICATION: ICP-based SLAM, live version
	FILE: icp-slam-live_main.cpp
	AUTHOR: Jose Luis Blanco Claraco <joseluisblancoc@gmail.com>
	See example config files in
	 https://github.com/MRPT/mrpt/tree/master/share/mrpt/config_files/icp-slam-live/
	or docs in
	 http://www.mrpt.org/list-of-mrpt-apps/application-icp-slam-live/
  ---------------------------------------------------------------*/

#include <mrpt/hwdrivers/CGenericSensor.h>
#include <mrpt/obs/CObservation2DRangeScan.h>
#include <mrpt/slam/CMetricMapBuilderICP.h>
#include <mrpt/config/CConfigFile.h>
#include <mrpt/io/CFileGZInputStream.h>
#include <mrpt/io/CFileGZOutputStream.h>
#include <mrpt/io/CFileOutputStream.h>
#include <mrpt/system/os.h>
#include <mrpt/system/filesystem.h>
#include <mrpt/opengl/COpenGLScene.h>
#include <mrpt/opengl/CGridPlaneXY.h>
#include <mrpt/opengl/stock_objects.h>
#include <mrpt/opengl/CPlanarLaserScan.h>
#include <mrpt/gui/CDisplayWindow3D.h>

using namespace mrpt;
using namespace mrpt::opengl;
using namespace mrpt::poses;
using namespace mrpt::io;
using namespace std;

// Forward declaration.
void MapBuilding_ICP_Live(const string& INI_FILENAME);

// ------------------------------------------------------
//						MAIN
// ------------------------------------------------------
int main(int argc, char** argv)
{
	using namespace mrpt::system;

	try
	{
		bool showHelp = argc > 1 && !os::_strcmp(argv[1], "--help");
		bool showVersion = argc > 1 && !os::_strcmp(argv[1], "--version");

		printf(" icp-slam-live - Part of the MRPT\n");
		printf(
			" MRPT C++ Library: %s - Sources timestamp: %s\n",
			MRPT_getVersion().c_str(), MRPT_getCompilationDate().c_str());
		if (showVersion) return 0;  // Program end
		printf(
			"------------------------------------------------------------------"
			"-\n");
		// Process arguments:
		if (argc < 2 || showHelp)
		{
			printf("Usage: %s <config_file.ini>\n\n", argv[0]);
			if (!showHelp)
			{
				mrpt::system::pause();
				return -1;
			}
			else
				return 0;
		}

		const string INI_FILENAME = string(argv[1]);
		ASSERT_FILE_EXISTS_(INI_FILENAME);

		// Run:
		MapBuilding_ICP_Live(INI_FILENAME);

		// pause();
		return 0;
	}
	catch (exception& e)
	{
		setConsoleColor(CONCOL_RED, true);
		cerr << "Program finished for an exception!!" << endl;
		setConsoleColor(CONCOL_NORMAL, true);

		cerr << mrpt::exception_to_str(e) << endl;

		mrpt::system::pause();
		return -1;
	}
	catch (...)
	{
		setConsoleColor(CONCOL_RED, true);
		cerr << "Program finished for an untyped exception!!" << endl;
		setConsoleColor(CONCOL_NORMAL, true);

		mrpt::system::pause();
		return -1;
	}
}

// Sensor thread -------------------------
MRPT_TODO("Should these be global?")
mrpt::hwdrivers::CGenericSensor::TListObservations global_list_obs;
std::mutex cs_global_list_obs;

bool allThreadsMustExit = false;
struct TThreadParams
{
	mrpt::config::CConfigFile* cfgFile;
	string section_name;
};

void SensorThread(TThreadParams params)
{
	using namespace mrpt::hwdrivers;
	using namespace mrpt::system;
	try
	{
		string driver_name = params.cfgFile->read_string(
			params.section_name, "driver", "", true);
		CGenericSensor::Ptr sensor =
			CGenericSensor::createSensorPtr(driver_name);
		if (!sensor)
			throw std::runtime_error(
				string("***ERROR***: Class name not recognized: ") +
				driver_name);

		// Load common & sensor specific parameters:
		sensor->loadConfig(*params.cfgFile, params.section_name);
		cout << format("[thread_%s] Starting...", params.section_name.c_str())
			 << " at " << sensor->getProcessRate() << " Hz" << endl;

		ASSERTMSG_(
			sensor->getProcessRate() > 0,
			"process_rate must be set to a valid value (>0 Hz).");
		const int process_period_ms =
			mrpt::round(1000.0 / sensor->getProcessRate());

		sensor->initialize();  // Init device:
		while (!allThreadsMustExit)
		{
			const TTimeStamp t0 = now();
			sensor->doProcess();  // Process
			// Get new observations
			CGenericSensor::TListObservations lstObjs;
			sensor->getObservations(lstObjs);
			{
				std::lock_guard<std::mutex> lock(cs_global_list_obs);
				global_list_obs.insert(lstObjs.begin(), lstObjs.end());
			}
			lstObjs.clear();
			// wait for the process period:
			TTimeStamp t1 = now();
			double At = timeDifference(t0, t1);
			int At_rem_ms = process_period_ms - At * 1000;
			if (At_rem_ms > 0)
				std::this_thread::sleep_for(
					std::chrono::milliseconds(At_rem_ms));
		}
		sensor.reset();
		cout << format("[thread_%s] Closing...", params.section_name.c_str())
			 << endl;
	}
	catch (const std::exception& e)
	{
		cerr << "[SensorThread]  Closing due to exception:\n"
			 << mrpt::exception_to_str(e) << endl;
		allThreadsMustExit = true;
	}
	catch (...)
	{
		cerr << "[SensorThread] Untyped exception! Closing." << endl;
		allThreadsMustExit = true;
	}
}

void MapBuilding_ICP_Live(const string& INI_FILENAME)
{
	MRPT_START

	using namespace mrpt::slam;
	using namespace mrpt::obs;
	using namespace mrpt::opengl;
	using namespace mrpt::poses;
	using namespace mrpt::maps;

	mrpt::config::CConfigFile iniFile(INI_FILENAME);

	// Load sensor params from section: "LIDAR_SENSOR"
	std::thread hSensorThread;
	{
		TThreadParams threParms;
		threParms.cfgFile = &iniFile;
		threParms.section_name = "LIDAR_SENSOR";
		std::cout << "\n\n==== Launching LIDAR grabbing thread ==== \n";
		hSensorThread = std::thread(SensorThread, threParms);
	}
	// Wait and check if the sensor is ready:
	std::this_thread::sleep_for(2000ms);
	if (allThreadsMustExit)
		throw std::runtime_error(
			"\n\n==== ABORTING: It seems that we could not connect to the "
			"LIDAR. See reported errors. ==== \n");

	// ------------------------------------------
	//			Load config from file:
	// ------------------------------------------
	const string OUT_DIR_STD = iniFile.read_string(
		"MappingApplication", "logOutput_dir", "log_out",
		/*Force existence:*/ true);
	const int LOG_FREQUENCY = iniFile.read_int(
		"MappingApplication", "LOG_FREQUENCY", 5, /*Force existence:*/ true);
	const bool SAVE_POSE_LOG = iniFile.read_bool(
		"MappingApplication", "SAVE_POSE_LOG", false,
		/*Force existence:*/ true);
	const bool SAVE_3D_SCENE = iniFile.read_bool(
		"MappingApplication", "SAVE_3D_SCENE", false,
		/*Force existence:*/ true);
	const bool CAMERA_3DSCENE_FOLLOWS_ROBOT = iniFile.read_bool(
		"MappingApplication", "CAMERA_3DSCENE_FOLLOWS_ROBOT", true,
		/*Force existence:*/ true);
	const bool SAVE_RAWLOG = iniFile.read_bool(
		"MappingApplication", "SAVE_RAWLOG", true, /*Force existence:*/ false);

	bool SHOW_PROGRESS_3D_REAL_TIME = false;
	int SHOW_PROGRESS_3D_REAL_TIME_DELAY_MS = 0;
	bool SHOW_LASER_SCANS_3D = true;

	MRPT_LOAD_CONFIG_VAR(
		SHOW_PROGRESS_3D_REAL_TIME, bool, iniFile, "MappingApplication");
	MRPT_LOAD_CONFIG_VAR(
		SHOW_LASER_SCANS_3D, bool, iniFile, "MappingApplication");
	MRPT_LOAD_CONFIG_VAR(
		SHOW_PROGRESS_3D_REAL_TIME_DELAY_MS, int, iniFile,
		"MappingApplication");
	const char* OUT_DIR = OUT_DIR_STD.c_str();

	// Create output rawlog file?
	CFileGZOutputStream out_rawlog;
	if (SAVE_RAWLOG)
	{
		mrpt::system::TTimeParts parts;
		mrpt::system::timestampToParts(mrpt::system::now(), parts, true);
		string rawlog_postfix = "_";
		rawlog_postfix += format(
			"%04u-%02u-%02u_%02uh%02um%02us", (unsigned int)parts.year,
			(unsigned int)parts.month, (unsigned int)parts.day,
			(unsigned int)parts.hour, (unsigned int)parts.minute,
			(unsigned int)parts.second);

		rawlog_postfix += string(".rawlog");
		rawlog_postfix =
			mrpt::system::fileNameStripInvalidChars(rawlog_postfix);
		const string rawlog_filename = "icpslamlive_dataset_" + rawlog_postfix;

		cout << endl;
		cout << "Output rawlog filename: " << rawlog_filename << endl;

		out_rawlog.open(rawlog_filename);
	}

	// ------------------------------------
	//		Constructor of ICP-SLAM object
	// ------------------------------------
	CMetricMapBuilderICP mapBuilder;

	mapBuilder.ICP_options.loadFromConfigFile(iniFile, "MappingApplication");
	mapBuilder.ICP_params.loadFromConfigFile(iniFile, "ICP");

	// Construct the maps with the loaded configuration.
	mapBuilder.initialize();

	// ---------------------------------
	//   CMetricMapBuilder::TOptions
	// ---------------------------------
	mapBuilder.setVerbosityLevel(mrpt::system::LVL_DEBUG);

	mapBuilder.ICP_params.dumpToConsole();
	mapBuilder.ICP_options.dumpToConsole();

	// Checks:
	mrpt::system::CTicTac tictac, tictacGlobal, tictac_JH;
	int step = 0;
	string str;
	CSimpleMap finalMap;
	float t_exec;
	COccupancyGridMap2D::TEntropyInfo entropy;

	size_t rawlogEntry = 0;

	// Prepare output directory:
	mrpt::system::deleteFilesInDirectory(OUT_DIR);
	mrpt::system::createDirectory(OUT_DIR);

	// Open log files:
	CFileOutputStream f_path(format("%s/log_estimated_path.txt", OUT_DIR));

	// Create 3D window if requested:
	mrpt::gui::CDisplayWindow3D::Ptr win3D;
#if MRPT_HAS_WXWIDGETS
	if (SHOW_PROGRESS_3D_REAL_TIME)
	{
		win3D = mrpt::make_aligned_shared<mrpt::gui::CDisplayWindow3D>(
			"icp-slam-live | Part of the MRPT project", 800, 600);
		win3D->setCameraZoom(20);
		win3D->setCameraAzimuthDeg(-45);
	}
#endif
	cout
		<< "\n"
		   " =============================================================\n"
		   "                  Running icp-slam-live\n"
		   "  To EXIT: press ESC or close the 3D view window \n"
		   " =============================================================\n\n";

	// ----------------------------------------------------------
	//						Map Building
	// ----------------------------------------------------------
	tictacGlobal.Tic();
	mrpt::system::CTicTac timeout_read_scans;
	while (!allThreadsMustExit)
	{
		// Check for exit app:
		if (mrpt::system::os::kbhit())
		{
			const char c = mrpt::system::os::getch();
			if (c == 27) break;
		}
		if (win3D && !win3D->isOpen()) break;

		// Load sensor LIDAR data from live capture:
		// --------------------------------------------------
		CObservation2DRangeScan::Ptr observation;
		{
			mrpt::hwdrivers::CGenericSensor::TListObservations obs_copy;
			{
				std::lock_guard<std::mutex> csl(cs_global_list_obs);
				obs_copy = global_list_obs;
				global_list_obs.clear();
			}
			// Keep the most recent laser scan:
			for (auto it = obs_copy.rbegin();
				 !observation && it != obs_copy.rend(); ++it)
				if (it->second && IS_CLASS(it->second, CObservation2DRangeScan))
					observation =
						std::dynamic_pointer_cast<CObservation2DRangeScan>(
							it->second);

			// Save all of them to rawlog for optional post-processing:
			if (out_rawlog.fileOpenCorrectly())
			{
				for (auto it = obs_copy.begin(); it != obs_copy.end(); ++it)
					if (it->second &&
						IS_CLASS(it->second, CObservation2DRangeScan))
						mrpt::serialization::archiveFrom(out_rawlog)
							<< *it->second;
			}
		}

		// If we don't have a laser scan, wait for it:
		if (!observation)
		{
			if (timeout_read_scans.Tac() > 1.0)
			{
				timeout_read_scans.Tic();
				cout << "[Warning] *** Waiting for laser scans from the Device "
						"***\n";
			}
			std::this_thread::sleep_for(1ms);
			continue;
		}
		else
		{
			timeout_read_scans.Tic();  // Reset timeout
		}

		// Build list of scans:
		std::vector<mrpt::obs::CObservation2DRangeScan::Ptr>
			lst_current_laser_scans;  // Just for drawing in 3D views
		if (SHOW_LASER_SCANS_3D)
		{
			if (observation)
			{
				lst_current_laser_scans.push_back(observation);
			}
		}

		// Execute ICP-SLAM:
		// ----------------------------------------
		tictac.Tic();
		mapBuilder.processObservation(observation);
		t_exec = tictac.Tac();
		printf("Map building executed in %.03fms\n", 1000.0f * t_exec);

		// Info log:
		// -----------
		const CMultiMetricMap* mostLikMap =
			mapBuilder.getCurrentlyBuiltMetricMap();

		if (0 == (step % LOG_FREQUENCY))
		{
			// Pose log:
			// -------------
			if (SAVE_POSE_LOG)
			{
				printf("Saving pose log information...");
				mapBuilder.getCurrentPoseEstimation()->saveToTextFile(
					format("%s/mapbuild_posepdf_%03u.txt", OUT_DIR, step));
				printf("Ok\n");
			}
		}

		// Save a 3D scene view of the mapping process:
		if (0 == (step % LOG_FREQUENCY) || (SAVE_3D_SCENE || win3D))
		{
			CPose3D robotPose;
			mapBuilder.getCurrentPoseEstimation()->getMean(robotPose);

			COpenGLScene::Ptr scene = mrpt::make_aligned_shared<COpenGLScene>();

			COpenGLViewport::Ptr view = scene->getViewport("main");
			ASSERT_(view);

			COpenGLViewport::Ptr view_map = scene->createViewport("mini-map");
			view_map->setBorderSize(2);
			view_map->setViewportPosition(0.01, 0.01, 0.35, 0.35);
			view_map->setTransparent(false);

			{
				mrpt::opengl::CCamera& cam = view_map->getCamera();
				cam.setAzimuthDegrees(-90);
				cam.setElevationDegrees(90);
				cam.setPointingAt(robotPose);
				cam.setZoomDistance(20);
				cam.setOrthogonal();
			}

			// The ground:
			mrpt::opengl::CGridPlaneXY::Ptr groundPlane =
				mrpt::make_aligned_shared<mrpt::opengl::CGridPlaneXY>(
					-200, 200, -200, 200, 0, 5);
			groundPlane->setColor(0.4, 0.4, 0.4);
			view->insert(groundPlane);
			view_map->insert(CRenderizable::Ptr(groundPlane));  // A copy

			// The camera pointing to the current robot pose:
			if (CAMERA_3DSCENE_FOLLOWS_ROBOT)
			{
				scene->enableFollowCamera(true);

				mrpt::opengl::CCamera& cam = view_map->getCamera();
				cam.setAzimuthDegrees(-45);
				cam.setElevationDegrees(45);
				cam.setPointingAt(robotPose);
			}

			// The maps:
			{
				opengl::CSetOfObjects::Ptr obj =
					mrpt::make_aligned_shared<opengl::CSetOfObjects>();
				mostLikMap->getAs3DObject(obj);
				view->insert(obj);

				// Only the point map:
				opengl::CSetOfObjects::Ptr ptsMap =
					mrpt::make_aligned_shared<opengl::CSetOfObjects>();
				if (mostLikMap->m_pointsMaps.size())
				{
					mostLikMap->m_pointsMaps[0]->getAs3DObject(ptsMap);
					view_map->insert(ptsMap);
				}
			}

			// Draw the robot path:
			CPose3DPDF::Ptr posePDF = mapBuilder.getCurrentPoseEstimation();
			CPose3D curRobotPose;
			posePDF->getMean(curRobotPose);
			{
				opengl::CSetOfObjects::Ptr obj =
					opengl::stock_objects::RobotPioneer();
				obj->setPose(curRobotPose);
				view->insert(obj);
			}
			{
				opengl::CSetOfObjects::Ptr obj =
					opengl::stock_objects::RobotPioneer();
				obj->setPose(curRobotPose);
				view_map->insert(obj);
			}

			// Draw laser scanners in 3D:
			if (SHOW_LASER_SCANS_3D)
			{
				for (auto& lst_current_laser_scan : lst_current_laser_scans)
				{
					// Create opengl object and load scan data from the scan
					// observation:
					opengl::CPlanarLaserScan::Ptr obj =
						mrpt::make_aligned_shared<opengl::CPlanarLaserScan>();
					obj->setScan(*lst_current_laser_scan);
					obj->setPose(curRobotPose);
					obj->setSurfaceColor(1.0f, 0.0f, 0.0f, 0.5f);
					// inser into the scene:
					view->insert(obj);
				}
			}

			// Save as file:
			if (0 == (step % LOG_FREQUENCY) && SAVE_3D_SCENE)
			{
				CFileGZOutputStream f(
					format("%s/buildingmap_%05u.3Dscene", OUT_DIR, step));
				mrpt::serialization::archiveFrom(f) << *scene;
			}

			// Show 3D?
			if (win3D)
			{
				opengl::COpenGLScene::Ptr& ptrScene =
					win3D->get3DSceneAndLock();
				ptrScene = scene;

				win3D->unlockAccess3DScene();

				// Move camera:
				win3D->setCameraPointingToPoint(
					curRobotPose.x(), curRobotPose.y(), curRobotPose.z());

				// Update:
				win3D->forceRepaint();

				std::this_thread::sleep_for(std::chrono::milliseconds(
					SHOW_PROGRESS_3D_REAL_TIME_DELAY_MS));
			}
		}

		// Save the robot estimated pose for each step:
		f_path.printf(
			"%i %f %f %f\n", step,
			mapBuilder.getCurrentPoseEstimation()->getMeanVal().x(),
			mapBuilder.getCurrentPoseEstimation()->getMeanVal().y(),
			mapBuilder.getCurrentPoseEstimation()->getMeanVal().yaw());

		step++;
		printf(
			"\n---------------- STEP %u | RAWLOG ENTRY %u ----------------\n",
			step, (unsigned)rawlogEntry);
	};

	printf(
		"\n---------------- END!! (total time: %.03f sec) ----------------\n",
		tictacGlobal.Tac());

	// Save map:
	mapBuilder.getCurrentlyBuiltMap(finalMap);

	str = format("%s/_finalmap_.simplemap", OUT_DIR);
	printf("Dumping final map in binary format to: %s\n", str.c_str());
	mapBuilder.saveCurrentMapToFile(str);

	const CMultiMetricMap* finalPointsMap =
		mapBuilder.getCurrentlyBuiltMetricMap();
	str = format("%s/_finalmaps_.txt", OUT_DIR);
	printf("Dumping final metric maps to %s_XXX\n", str.c_str());
	finalPointsMap->saveMetricMapRepresentationToFile(str);

	cout << "Waiting for sensor thread to exit...\n";
	allThreadsMustExit = true;
	hSensorThread.join();
	cout << "Sensor thread is closed. Bye bye!\n";

	if (win3D && win3D->isOpen()) win3D->waitForKey();

	MRPT_END
}
