/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2019, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#include "apps-precomp.h"  // Precompiled headers

#include <mrpt/apps/ICP_SLAM_App.h>
#include <mrpt/gui/CDisplayWindow3D.h>
#include <mrpt/io/CFileGZOutputStream.h>
#include <mrpt/io/CFileOutputStream.h>
#include <mrpt/io/vector_loadsave.h>
#include <mrpt/maps/COccupancyGridMap2D.h>
#include <mrpt/obs/CObservationOdometry.h>
#include <mrpt/opengl/CGridPlaneXY.h>
#include <mrpt/opengl/COpenGLScene.h>
#include <mrpt/opengl/CPlanarLaserScan.h>  // from lib [mrpt-maps]
#include <mrpt/opengl/stock_objects.h>
#include <mrpt/serialization/CArchive.h>
#include <mrpt/slam/CMetricMapBuilderICP.h>
#include <mrpt/system/CRateTimer.h>
#include <mrpt/system/filesystem.h>
#include <mrpt/system/memory.h>
#include <mrpt/system/os.h>

using namespace mrpt::apps;

constexpr auto sect = "MappingApplication";

// ---------------------------------------
//   ICP_SLAM_App_Base
// ---------------------------------------
ICP_SLAM_App_Base::ICP_SLAM_App_Base()
{
	// Set logger display name:
	this->setLoggerName("ICP_SLAM_App");
}

void ICP_SLAM_App_Base::initialize(int argc, const char** argv)
{
	MRPT_START

	MRPT_LOG_INFO_FMT(
		" icp-slam - Part of the MRPT\n"
		" MRPT C++ Library: %s - Sources timestamp: %s\n\n",
		mrpt::system::MRPT_getVersion().c_str(),
		mrpt::system::MRPT_getCompilationDate().c_str());

	// Process arguments:
	if (argc < 2)
	{
		THROW_EXCEPTION_FMT("Usage: %s", impl_get_usage().c_str());
	}

	// Config file:
	const std::string configFile = std::string(argv[1]);

	ASSERT_FILE_EXISTS_(configFile);
	params.setContent(mrpt::io::file_get_contents(configFile));

	impl_initialize(argc, argv);

	MRPT_END
}

void ICP_SLAM_App_Base::run()
{
	MRPT_START

	using namespace mrpt;
	using namespace mrpt::slam;
	using namespace mrpt::obs;
	using namespace mrpt::maps;
	using namespace mrpt::opengl;
	using namespace mrpt::gui;
	using namespace mrpt::io;
	using namespace mrpt::gui;
	using namespace mrpt::config;
	using namespace mrpt::system;
	using namespace mrpt::math;
	using namespace mrpt::poses;
	using namespace std;

	// ------------------------------------------
	//			Load config from file:
	// ------------------------------------------

	const string OUT_DIR_STD =
		params.read_string(sect, "logOutput_dir", "log_out", true);
	const char* OUT_DIR = OUT_DIR_STD.c_str();

	const int LOG_FREQUENCY = params.read_int(sect, "LOG_FREQUENCY", 5, true);
	const bool SAVE_POSE_LOG =
		params.read_bool(sect, "SAVE_POSE_LOG", false, true);
	const bool SAVE_3D_SCENE =
		params.read_bool(sect, "SAVE_3D_SCENE", false, true);
	const bool CAMERA_3DSCENE_FOLLOWS_ROBOT =
		params.read_bool(sect, "CAMERA_3DSCENE_FOLLOWS_ROBOT", true, true);

	bool SHOW_PROGRESS_3D_REAL_TIME = false;
	int SHOW_PROGRESS_3D_REAL_TIME_DELAY_MS = 0;
	bool SHOW_LASER_SCANS_3D = true;

	MRPT_LOAD_CONFIG_VAR(SHOW_PROGRESS_3D_REAL_TIME, bool, params, sect);
	MRPT_LOAD_CONFIG_VAR(SHOW_LASER_SCANS_3D, bool, params, sect);
	MRPT_LOAD_CONFIG_VAR(
		SHOW_PROGRESS_3D_REAL_TIME_DELAY_MS, int, params, sect);

	if (params.keyExists(sect, "verbosity"))
		this->setMinLoggingLevel(params.read_enum<mrpt::system::VerbosityLevel>(
			sect, "verbosity", mrpt::system::VerbosityLevel::LVL_INFO));

	// ------------------------------------
	//		Constructor of ICP-SLAM object
	// ------------------------------------
	CMetricMapBuilderICP mapBuilder;
	mapBuilder.setVerbosityLevel(this->getMinLoggingLevel());

	mapBuilder.ICP_options.loadFromConfigFile(params, sect);
	mapBuilder.ICP_params.loadFromConfigFile(params, "ICP");

	// Construct the maps with the loaded configuration.
	mapBuilder.initialize();

	// ---------------------------------
	//   CMetricMapBuilder::TOptions
	// ---------------------------------
	mapBuilder.options.alwaysInsertByClass.fromString(
		params.read_string(sect, "alwaysInsertByClass", ""));

	// Print params:
	MRPT_LOG_INFO_FMT("Output directory: `%s`", OUT_DIR);

	{
		std::stringstream ss;
		mapBuilder.ICP_params.dumpToTextStream(ss);
		mapBuilder.ICP_options.dumpToTextStream(ss);
		MRPT_LOG_INFO(ss.str());
	}

	CTicTac tictac, tictacGlobal;
	int step = 0;
	CSimpleMap finalMap;
	float t_exec;

	// Prepare output directory:
	// --------------------------------
	deleteFilesInDirectory(OUT_DIR);
	createDirectory(OUT_DIR);

	// Open log files:
	// ----------------------------------
	CFileOutputStream f_log(format("%s/log_times.txt", OUT_DIR));
	CFileOutputStream f_path(format("%s/log_estimated_path.txt", OUT_DIR));
	CFileOutputStream f_pathOdo(format("%s/log_odometry_path.txt", OUT_DIR));

	// Create 3D window if requested:
	CDisplayWindow3D::Ptr win3D;
#if MRPT_HAS_WXWIDGETS
	if (SHOW_PROGRESS_3D_REAL_TIME)
	{
		win3D = std::make_shared<CDisplayWindow3D>(
			"ICP-SLAM @ MRPT C++ Library", 600, 500);
		win3D->setCameraZoom(20);
		win3D->setCameraAzimuthDeg(-45);
	}
#endif

	// ----------------------------------------------------------
	//						Map Building
	// ----------------------------------------------------------
	CPose2D odoPose(0, 0, 0);

	tictacGlobal.Tic();
	for (;;)
	{
		CActionCollection::Ptr action;
		CSensoryFrame::Ptr observations;
		CObservation::Ptr observation;

		if (quits_with_esc_key && os::kbhit())
		{
			char c = os::getch();
			if (c == 27) break;
		}

		// Load action/observation pair from the rawlog:
		// --------------------------------------------------
		if (!impl_get_next_observations(action, observations, observation))
			break;  // EOF

		const bool isObsBasedRawlog = observation ? true : false;

		ASSERT_(
			(isObsBasedRawlog && observation->timestamp != INVALID_TIMESTAMP) ||
			(!isObsBasedRawlog && !observations->empty() &&
			 *observations->begin() &&
			 (*observations->begin())->timestamp != INVALID_TIMESTAMP));

		const mrpt::system::TTimeStamp cur_timestamp =
			isObsBasedRawlog ? observation->timestamp
							 : (*observations->begin())->timestamp;

		// For drawing in 3D views:
		std::vector<mrpt::obs::CObservation2DRangeScan::Ptr> lst_lidars;

		// Update odometry:
		if (isObsBasedRawlog)
		{
			static CPose2D lastOdo;
			static bool firstOdo = true;
			if (IS_CLASS(*observation, CObservationOdometry))
			{
				auto o = std::dynamic_pointer_cast<CObservationOdometry>(
					observation);
				if (!firstOdo) odoPose = odoPose + (o->odometry - lastOdo);

				lastOdo = o->odometry;
				firstOdo = false;
			}
		}
		else
		{
			CActionRobotMovement2D::Ptr act =
				action->getBestMovementEstimation();
			if (act) odoPose = odoPose + act->poseChange->getMeanVal();
		}

		// Build list of scans:
		if (SHOW_LASER_SCANS_3D)
		{
			// Rawlog in "Observation-only" format:
			if (isObsBasedRawlog)
			{
				if (IS_CLASS(*observation, CObservation2DRangeScan))
				{
					lst_lidars.push_back(
						std::dynamic_pointer_cast<CObservation2DRangeScan>(
							observation));
				}
			}
			else
			{
				// Rawlog in the Actions-SF format:
				for (size_t i = 0;; i++)
				{
					CObservation2DRangeScan::Ptr new_obs =
						observations
							->getObservationByClass<CObservation2DRangeScan>(i);
					if (!new_obs)
						break;  // There're no more scans
					else
						lst_lidars.push_back(new_obs);
				}
			}
		}

		// Execute:
		// ----------------------------------------
		tictac.Tic();
		if (isObsBasedRawlog)
			mapBuilder.processObservation(observation);
		else
			mapBuilder.processActionObservation(*action, *observations);
		t_exec = tictac.Tac();
		MRPT_LOG_INFO_FMT("Map building executed in %.03fms", 1000.0f * t_exec);

		// Info log:
		// -----------
		f_log.printf(
			"%f %i\n", 1000.0f * t_exec, mapBuilder.getCurrentlyBuiltMapSize());

		const CMultiMetricMap* mostLikMap =
			mapBuilder.getCurrentlyBuiltMetricMap();

		if (LOG_FREQUENCY > 0 && 0 == (step % LOG_FREQUENCY))
		{
			// Pose log:
			// -------------
			if (SAVE_POSE_LOG)
			{
				auto str =
					mrpt::format("%s/mapbuild_posepdf_%03u.txt", OUT_DIR, step);
				MRPT_LOG_INFO_FMT(
					"Saving pose log information to `%s`", str.c_str());
				mapBuilder.getCurrentPoseEstimation()->saveToTextFile(str);
			}
		}

		const CPose3D robotPose =
			mapBuilder.getCurrentPoseEstimation()->getMeanVal();

		// Save a 3D scene view of the mapping process:
		if ((LOG_FREQUENCY > 0 && 0 == (step % LOG_FREQUENCY)) ||
			(SAVE_3D_SCENE || win3D))
		{
			auto scene = mrpt::opengl::COpenGLScene::Create();

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
				mrpt::opengl::CGridPlaneXY::Create(-200, 200, -200, 200, 0, 5);
			groundPlane->setColor(0.4f, 0.4f, 0.4f);
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
				auto obj = opengl::CSetOfObjects::Create();
				mostLikMap->getAs3DObject(obj);
				view->insert(obj);

				// Only the point map:
				auto ptsMap = opengl::CSetOfObjects::Create();
				if (auto pMap = mostLikMap->mapByClass<CPointsMap>(); pMap)
				{
					pMap->getAs3DObject(ptsMap);
					view_map->insert(ptsMap);
				}
			}

			// Draw the robot path:
			{
				auto obj = opengl::stock_objects::RobotPioneer();
				obj->setPose(robotPose);
				view->insert(obj);
			}
			{
				auto obj = opengl::stock_objects::RobotPioneer();
				obj->setPose(robotPose);
				view_map->insert(obj);
			}

			// Draw laser scanners in 3D:
			if (SHOW_LASER_SCANS_3D)
			{
				for (auto& lst_current_laser_scan : lst_lidars)
				{
					// Create opengl object and load scan data from the scan
					// observation:
					auto obj = opengl::CPlanarLaserScan::Create();
					obj->setScan(*lst_current_laser_scan);
					obj->setPose(robotPose);
					obj->setSurfaceColor(1.0f, 0.0f, 0.0f, 0.5f);
					// inser into the scene:
					view->insert(obj);
				}
			}

			// Save as file:
			if (LOG_FREQUENCY > 0 && 0 == (step % LOG_FREQUENCY) &&
				SAVE_3D_SCENE)
			{
				CFileGZOutputStream f(
					mrpt::format("%s/buildingmap_%05u.3Dscene", OUT_DIR, step));
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
					robotPose.x(), robotPose.y(), robotPose.z());

				// Update:
				win3D->forceRepaint();

				std::this_thread::sleep_for(std::chrono::milliseconds(
					SHOW_PROGRESS_3D_REAL_TIME_DELAY_MS));
			}
		}

		// Save the memory usage:
		// ------------------------------------------------------------------
		{
			auto str = mrpt::format("%s/log_MemoryUsage.txt", OUT_DIR);
			unsigned long memUsage = getMemoryUsage();
			FILE* f = os::fopen(str.c_str(), "at");
			if (f)
			{
				os::fprintf(f, "%u\t%lu\n", step, memUsage);
				os::fclose(f);
			}
			MRPT_LOG_INFO_FMT(
				"Memory usage:%.04f MiB", memUsage / (1024.0 * 1024.0));
		}

		// Save the robot estimated pose for each step:
		f_path.printf(
			"%i %f %f %f\n", step, robotPose.x(), robotPose.y(),
			robotPose.yaw());

		// Also keep the robot path as a vector, for the convenience of the app
		// user:
		out_estimated_path[cur_timestamp] = robotPose.asTPose();

		f_pathOdo.printf(
			"%i %f %f %f\n", step, odoPose.x(), odoPose.y(), odoPose.phi());

		step++;
		MRPT_LOG_INFO_FMT("------------- STEP %u ----------------", step);
	};

	MRPT_LOG_INFO_FMT(
		"----------- **END** (total time: %.03f sec) ---------",
		tictacGlobal.Tac());

	// Save map:
	mapBuilder.getCurrentlyBuiltMap(finalMap);

	{
		auto str = format("%s/_finalmap_.simplemap", OUT_DIR);
		MRPT_LOG_INFO_FMT(
			"Dumping final map in binary format to: %s\n", str.c_str());
		mapBuilder.saveCurrentMapToFile(str);
	}

	{
		const CMultiMetricMap* finalPointsMap =
			mapBuilder.getCurrentlyBuiltMetricMap();
		auto str = format("%s/_finalmaps_.txt", OUT_DIR);
		MRPT_LOG_INFO_FMT("Dumping final metric maps to %s_XXX\n", str.c_str());
		finalPointsMap->saveMetricMapRepresentationToFile(str);
	}

	if (win3D) win3D->waitForKey();

	MRPT_END
}

// ---------------------------------------
//   ICP_SLAM_App_Rawlog
// ---------------------------------------
ICP_SLAM_App_Rawlog::ICP_SLAM_App_Rawlog()
{
	setLoggerName("ICP_SLAM_App_Rawlog");
}

void ICP_SLAM_App_Rawlog::impl_initialize(int argc, const char** argv)
{
	MRPT_START
	// Rawlog file: from args. line or from config file:
	if (argc == 3)
		m_rawlogFileName = std::string(argv[2]);
	else
		m_rawlogFileName = params.read_string(
			sect, "rawlog_file", std::string("log.rawlog"), true);

	m_rawlog_offset = params.read_int(sect, "rawlog_offset", 0, true);

	ASSERT_FILE_EXISTS_(m_rawlogFileName);

	MRPT_END
}

// ---------------------------------------
//   ICP_SLAM_App_Live
// ---------------------------------------
ICP_SLAM_App_Live::ICP_SLAM_App_Live()
{
	this->setLoggerName("ICP_SLAM_App_Live");
}

ICP_SLAM_App_Live::~ICP_SLAM_App_Live() = default;

void ICP_SLAM_App_Live::impl_initialize(int argc, const char** argv)
{
	MRPT_START

	if (argc != 2)
	{
		THROW_EXCEPTION_FMT("Usage: %s", impl_get_usage().c_str());
	}

	// Config file already loaded into "params".

	// Load sensor params from section: "LIDAR_SENSOR"
	std::thread hSensorThread;
	{
		TThreadParams threParms;
		threParms.cfgFile = &params;
		threParms.section_name = "LIDAR_SENSOR";
		MRPT_LOG_INFO("Launching LIDAR grabbing thread...");
		hSensorThread =
			std::thread(&ICP_SLAM_App_Live::SensorThread, this, threParms);
	}
	// Wait and check if the sensor is ready:
	using namespace std::chrono_literals;
	std::this_thread::sleep_for(2000ms);

	if (m_allThreadsMustExit)
		throw std::runtime_error(
			"\n\n==== ABORTING: It seems that we could not connect to the "
			"LIDAR. See reported errors. ==== \n");

	MRPT_END
}

bool ICP_SLAM_App_Live::impl_get_next_observations(
	[[maybe_unused]] mrpt::obs::CActionCollection::Ptr& action,
	[[maybe_unused]] mrpt::obs::CSensoryFrame::Ptr& observations,
	mrpt::obs::CObservation::Ptr& observation)
{
	MRPT_START

	using mrpt::obs::CObservation2DRangeScan;

	// Check if we had any hardware failure:
	if (m_allThreadsMustExit) return false;

	const auto t0 = mrpt::Clock::now();

	// Load sensor LIDAR data from live capture:
	while (mrpt::system::timeDifference(t0, mrpt::Clock::now()) < 2.0)
	{
		CObservation2DRangeScan::Ptr new_obs;
		{
			mrpt::hwdrivers::CGenericSensor::TListObservations obs_copy;
			{
				std::lock_guard<std::mutex> csl(m_cs_global_list_obs);
				obs_copy = m_global_list_obs;
				m_global_list_obs.clear();
			}
			// Keep the most recent laser scan:
			for (auto it = obs_copy.rbegin(); !new_obs && it != obs_copy.rend();
				 ++it)
				if (it->second &&
					IS_CLASS(*it->second, CObservation2DRangeScan))
					new_obs =
						std::dynamic_pointer_cast<CObservation2DRangeScan>(
							it->second);
		}

		if (new_obs)
		{
			observation = std::move(new_obs);
			return true;
		}
		else
		{
			using namespace std::chrono_literals;
			std::this_thread::sleep_for(10ms);
		}
	}

	// timeout:
	MRPT_LOG_ERROR("Timeout waiting for next lidar scan.");
	return false;

	MRPT_END
}

void ICP_SLAM_App_Live::SensorThread(ICP_SLAM_App_Live::TThreadParams tp)
{
	using namespace mrpt::hwdrivers;
	using namespace mrpt::system;
	try
	{
		std::string driver_name =
			tp.cfgFile->read_string(tp.section_name, "driver", "", true);
		CGenericSensor::Ptr sensor =
			CGenericSensor::createSensorPtr(driver_name);
		if (!sensor)
			throw std::runtime_error(
				std::string("***ERROR***: Class name not recognized: ") +
				driver_name);

		// Load common & sensor specific parameters:
		sensor->loadConfig(*tp.cfgFile, tp.section_name);
		std::cout << mrpt::format(
						 "[thread_%s] Starting...", tp.section_name.c_str())
				  << " at " << sensor->getProcessRate() << " Hz" << std::endl;

		ASSERTMSG_(
			sensor->getProcessRate() > 0,
			"process_rate must be set to a valid value (>0 Hz).");

		mrpt::system::CRateTimer rate(sensor->getProcessRate());

		sensor->initialize();  // Init device:
		while (!m_allThreadsMustExit)
		{
			sensor->doProcess();  // Process
			// Get new observations
			CGenericSensor::TListObservations lstObjs;
			sensor->getObservations(lstObjs);
			{
				std::lock_guard<std::mutex> lock(m_cs_global_list_obs);
				m_global_list_obs.insert(lstObjs.begin(), lstObjs.end());
			}
			lstObjs.clear();

			// wait for the process period:
			rate.sleep();
		}
		sensor.reset();
		printf("[thread_%s] Closing...", tp.section_name.c_str());
	}
	catch (const std::exception& e)
	{
		std::cerr << "[SensorThread]  Closing due to exception:\n"
				  << mrpt::exception_to_str(e) << std::endl;
		m_allThreadsMustExit = true;
	}
	catch (...)
	{
		std::cerr << "[SensorThread] Untyped exception! Closing." << std::endl;
		m_allThreadsMustExit = true;
	}
}
