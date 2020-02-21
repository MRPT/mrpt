/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2019, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#include "apps-precomp.h"  // Precompiled headers

#include <mrpt/apps/RBPF_SLAM_App.h>
#include <mrpt/gui/CDisplayWindow3D.h>
#include <mrpt/io/CFileGZOutputStream.h>
#include <mrpt/io/CFileOutputStream.h>
#include <mrpt/io/vector_loadsave.h>
#include <mrpt/maps/COccupancyGridMap2D.h>
#include <mrpt/obs/CActionRobotMovement2D.h>
#include <mrpt/obs/CActionRobotMovement3D.h>
#include <mrpt/obs/CObservationGasSensors.h>
#include <mrpt/obs/CObservationWirelessPower.h>
#include <mrpt/obs/CRawlog.h>
#include <mrpt/opengl/CEllipsoid3D.h>
#include <mrpt/opengl/CGridPlaneXY.h>
#include <mrpt/opengl/CSetOfLines.h>
#include <mrpt/opengl/stock_objects.h>
#include <mrpt/poses/CPosePDFGaussian.h>
#include <mrpt/random.h>
#include <mrpt/system/filesystem.h>  // ASSERT_FILE_EXISTS_()
#include <mrpt/system/memory.h>  // getMemoryUsage()

using namespace mrpt::apps;

constexpr auto sect = "MappingApplication";

// ---------------------------------------
//   RBPF_SLAM_App_Base
// ---------------------------------------
RBPF_SLAM_App_Base::RBPF_SLAM_App_Base()
{
	// Set logger display name:
	this->setLoggerName("RBPF_SLAM_App");
}

void RBPF_SLAM_App_Base::initialize(int argc, const char** argv)
{
	MRPT_START

	MRPT_LOG_INFO_FMT(
		" rbpf-slam - Part of the MRPT\n"
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

void RBPF_SLAM_App_Base::run()
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

	int LOG_FREQUENCY = 5;
	bool GENERATE_LOG_JOINT_H = false;
	bool GENERATE_LOG_INFO = false;
	bool SAVE_POSE_LOG = false;
	bool SAVE_MAP_IMAGES = false;
	bool SAVE_3D_SCENE = false;
	bool CAMERA_3DSCENE_FOLLOWS_ROBOT = true;
	bool SHOW_PROGRESS_IN_WINDOW = false;
	int SHOW_PROGRESS_IN_WINDOW_DELAY_MS = 1;
	std::string METRIC_MAP_CONTINUATION_GRIDMAP_FILE;
	std::string SIMPLEMAP_CONTINUATION;
	mrpt::math::TPose2D METRIC_MAP_CONTINUATION_START_POSE;
	int PROGRESS_WINDOW_WIDTH = 600, PROGRESS_WINDOW_HEIGHT = 500;
	int RANDOM_SEED = -1;  // <0: randomize
	const string OUT_DIR_STD =
		params.read_string(sect, "logOutput_dir", "log_out", true);

	if (params.keyExists(sect, "verbosity"))
		this->setMinLoggingLevel(params.read_enum<mrpt::system::VerbosityLevel>(
			sect, "verbosity", mrpt::system::VerbosityLevel::LVL_INFO));

	// This to allow using the shorter XXX_CS() macros to read params:
	{
		const auto& c = params;
		const auto& s = sect;

		MRPT_LOAD_CONFIG_VAR_CS(LOG_FREQUENCY, int);
		MRPT_LOAD_CONFIG_VAR_CS(GENERATE_LOG_JOINT_H, bool);
		MRPT_LOAD_CONFIG_VAR_CS(GENERATE_LOG_INFO, bool);
		MRPT_LOAD_CONFIG_VAR_CS(SAVE_POSE_LOG, bool);
		MRPT_LOAD_CONFIG_VAR_CS(SAVE_MAP_IMAGES, bool);
		MRPT_LOAD_CONFIG_VAR_CS(SAVE_3D_SCENE, bool);
		MRPT_LOAD_CONFIG_VAR_CS(CAMERA_3DSCENE_FOLLOWS_ROBOT, bool);
		MRPT_LOAD_CONFIG_VAR_CS(SHOW_PROGRESS_IN_WINDOW, bool);
		MRPT_LOAD_CONFIG_VAR_CS(SHOW_PROGRESS_IN_WINDOW_DELAY_MS, int);
		MRPT_LOAD_CONFIG_VAR_CS(METRIC_MAP_CONTINUATION_GRIDMAP_FILE, string);
		MRPT_LOAD_CONFIG_VAR_CS(SIMPLEMAP_CONTINUATION, string);
		METRIC_MAP_CONTINUATION_START_POSE.x =
			c.read_double(s, "METRIC_MAP_CONTINUATION_START_POSE_X", .0);
		METRIC_MAP_CONTINUATION_START_POSE.y =
			c.read_double(s, "METRIC_MAP_CONTINUATION_START_POSE_Y", .0);
		METRIC_MAP_CONTINUATION_START_POSE.phi = DEG2RAD(
			c.read_double(s, "METRIC_MAP_CONTINUATION_START_POSE_PHI_DEG", .0));
		MRPT_LOAD_CONFIG_VAR_CS(PROGRESS_WINDOW_WIDTH, int);
		MRPT_LOAD_CONFIG_VAR_CS(PROGRESS_WINDOW_HEIGHT, int);
		MRPT_LOAD_CONFIG_VAR_CS(RANDOM_SEED, int);
	}
	const char* OUT_DIR = OUT_DIR_STD.c_str();

	// Print params:
	MRPT_LOG_INFO_FMT("Output directory: `%s`", OUT_DIR);

	CTicTac tictac, tictacGlobal, tictac_JH;
	int step = 0;
	CSimpleMap finalMap;
	float t_exec;
	COccupancyGridMap2D::TEntropyInfo entropy;

	char strFil[1000];

	// ---------------------------------
	//		MapPDF opts
	// ---------------------------------
	CMetricMapBuilderRBPF::TConstructionOptions rbpfMappingOptions;
	rbpfMappingOptions.loadFromConfigFile(params, sect);

	// ---------------------------------
	//		Constructor
	// ---------------------------------
	mapBuilder = std::make_shared<CMetricMapBuilderRBPF>(rbpfMappingOptions);
	mapBuilder->setVerbosityLevel(this->getMinLoggingLevel());

	{
		std::stringstream ss;
		rbpfMappingOptions.dumpToTextStream(ss);
		MRPT_LOG_INFO(ss.str());
	}

	// handle the case of metric map continuation
	if (!METRIC_MAP_CONTINUATION_GRIDMAP_FILE.empty())
	{
		CSimpleMap dummySimpleMap;
		CPosePDFGaussian startPose;

		startPose.mean.x(METRIC_MAP_CONTINUATION_START_POSE.x);
		startPose.mean.y(METRIC_MAP_CONTINUATION_START_POSE.y);
		startPose.mean.phi(METRIC_MAP_CONTINUATION_START_POSE.phi);
		startPose.cov.setZero();

		mrpt::maps::COccupancyGridMap2D gridmap;
		{
			mrpt::io::CFileGZInputStream f(
				METRIC_MAP_CONTINUATION_GRIDMAP_FILE);
			mrpt::serialization::archiveFrom(f) >> gridmap;
		}

		mapBuilder->initialize(dummySimpleMap, &startPose);

		for (auto& m_particle : mapBuilder->mapPDF.m_particles)
		{
			CRBPFParticleData* part_d = m_particle.d.get();
			CMultiMetricMap& mmap = part_d->mapTillNow;
			mrpt::maps::COccupancyGridMap2D::Ptr it_grid =
				mmap.mapByClass<mrpt::maps::COccupancyGridMap2D>();
			ASSERTMSG_(
				it_grid,
				"No gridmap in multimetric map definition, but metric map "
				"continuation was set (!)");
			it_grid->copyMapContentFrom(gridmap);
		}
	}
	if (!SIMPLEMAP_CONTINUATION.empty())
	{
		mrpt::maps::CSimpleMap init_map;
		mrpt::io::CFileGZInputStream f(SIMPLEMAP_CONTINUATION);
		mrpt::serialization::archiveFrom(f) >> init_map;
		mapBuilder->initialize(init_map);
	}

	// ---------------------------------
	//   CMetricMapBuilder::TOptions
	// ---------------------------------
	// mapBuilder->setVerbosityLevel(  mrpt::system::LVL_DEBUG );  // default
	// value: as loaded from config file
	mapBuilder->options.enableMapUpdating = true;
	mapBuilder->options.debugForceInsertion = false;

	auto& rng = mrpt::random::getRandomGenerator();
	if (RANDOM_SEED >= 0)
		rng.randomize(RANDOM_SEED);
	else
		rng.randomize();

	// Prepare output directory:
	// --------------------------------
	deleteFilesInDirectory(OUT_DIR);
	createDirectory(OUT_DIR);

	string OUT_DIR_MAPS = format("%s/maps", OUT_DIR);
	string OUT_DIR_3D = format("%s/3D", OUT_DIR);

	deleteFilesInDirectory(OUT_DIR_MAPS);
	createDirectory(OUT_DIR_MAPS);

	deleteFilesInDirectory(OUT_DIR_3D);
	createDirectory(OUT_DIR_3D);

	// Open log files:
	// ----------------------------------
	CFileOutputStream f_log(format("%s/log_times.txt", OUT_DIR));
	CFileOutputStream f_info(format("%s/log_info.txt", OUT_DIR));
	CFileOutputStream f_jinfo(format("%s/log_jinfo.txt", OUT_DIR));
	CFileOutputStream f_path(format("%s/log_estimated_path.txt", OUT_DIR));
	CFileOutputStream f_pathOdo(format("%s/log_odometry_path.txt", OUT_DIR));
	CFileOutputStream f_partStats(format("%s/log_ParticlesStats.txt", OUT_DIR));

	f_log.printf(
		"%% time_step  execution_time(ms)  map_size(#frames)  frame_inserted? "
		"\n"
		"%%-------------------------------------------------------------------"
		"\n");

	f_info.printf(
		"%% EMI    H    EMMI    effecMappedArea  effecMappedCells \n"
		"%%-------------------------------------------------------\n");

	f_pathOdo.printf(
		"%% time_step  x  y z yaw pitch roll timestamp \n"
		"%%--------------------------------------------\n");

	f_pathOdo.printf(
		"%% time_step  x  y z yaw pitch roll \n"
		"%%----------------------------------\n");

	f_partStats.printf(
		"%% time_step   #particles   ESS \n"
		"%%------------------------------\n");

	// ----------------------------------------------------------
	//						Map Building
	// ----------------------------------------------------------
	CPose3D odoPose(0, 0, 0);

	CDisplayWindow3D::Ptr win3D;

	if (SHOW_PROGRESS_IN_WINDOW)
	{
		win3D = CDisplayWindow3D::Create(
			"RBPF-SLAM @ MRPT C++ Library", PROGRESS_WINDOW_WIDTH,
			PROGRESS_WINDOW_HEIGHT);
		win3D->setCameraZoom(40);
		win3D->setCameraAzimuthDeg(-50);
		win3D->setCameraElevationDeg(70);
	}

	tictacGlobal.Tic();
	for (;;)
	{
		if (quits_with_esc_key && os::kbhit())
		{
			char c = os::getch();
			if (c == 27) break;
		}

		CActionCollection::Ptr action;
		CSensoryFrame::Ptr observations;
		CObservation::Ptr observation;

		// Load action/observation pair from the rawlog:
		// --------------------------------------------------
		if (!impl_get_next_observations(action, observations, observation))
			break;  // EOF

		// Update odometry:
		{
			CActionRobotMovement2D::Ptr act =
				action->getBestMovementEstimation();
			if (act)
				odoPose = odoPose + CPose3D(act->poseChange->getMeanVal());
			else
			{
				CActionRobotMovement3D::Ptr act3D =
					action->getActionByClass<CActionRobotMovement3D>();
				if (act3D) odoPose = odoPose + act3D->poseChange.mean;
			}
		}

		mrpt::system::TTimeStamp observations_timestamp;
		if (observations && !observations->empty())
			observations_timestamp = (*observations->begin())->timestamp;

		// Execute:
		// ----------------------------------------
		tictac.Tic();
		mapBuilder->processActionObservation(*action, *observations);
		t_exec = tictac.Tac();
		MRPT_LOG_INFO_FMT("Map building executed in %.03fms", 1000.0f * t_exec);

		// Info log:
		// -----------
		f_log.printf(
			"%u %f %i %i\n", static_cast<unsigned int>(step), 1000.0f * t_exec,
			mapBuilder->getCurrentlyBuiltMapSize(),
			mapBuilder->m_statsLastIteration.observationsInserted ? int(1)
																  : int(0));

		CPose3DPDF::Ptr curPDFptr = mapBuilder->getCurrentPoseEstimation();
		CPose3DPDFParticles curPDF;

		if (IS_CLASS(*curPDFptr, CPose3DPDFParticles))
		{
			CPose3DPDFParticles::Ptr pp =
				std::dynamic_pointer_cast<CPose3DPDFParticles>(curPDFptr);
			curPDF = *pp;
		}

		if (LOG_FREQUENCY > 0 && 0 == (step % LOG_FREQUENCY))
		{
			const CMultiMetricMap* mostLikMap =
				mapBuilder->mapPDF.getCurrentMostLikelyMetricMap();

			if (GENERATE_LOG_INFO)
			{
				tictac_JH.Tic();

				const CMultiMetricMap* avrMap =
					mapBuilder->mapPDF.getAveragedMetricMapEstimation();
				COccupancyGridMap2D::Ptr grid =
					avrMap->mapByClass<COccupancyGridMap2D>();
				ASSERT_(grid);
				grid->computeEntropy(entropy);

				grid->saveAsBitmapFile(
					format("%s/EMMI_gridmap_%03u.png", OUT_DIR, step));

				f_info.printf(
					"%f %f %f %f %lu\n", entropy.I, entropy.H, entropy.mean_I,
					entropy.effectiveMappedArea, entropy.effectiveMappedCells);
				MRPT_LOG_INFO_FMT(
					"Log information saved. EMI = %.04f EMMI=%.04f (in "
					"%.03fms)\n",
					entropy.I, entropy.mean_I, 1000.0f * tictac_JH.Tac());
			}

			// Pose log:
			// -------------
			if (SAVE_POSE_LOG)
			{
				curPDF.saveToTextFile(
					format("%s/mapbuild_posepdf_%03u.txt", OUT_DIR, step));
			}

			// Map images:
			// -------------
			if (SAVE_MAP_IMAGES)
			{
				MRPT_LOG_DEBUG("Saving map images to files...");

				//  Most likely maps:
				// ----------------------------------------
				mostLikMap->saveMetricMapRepresentationToFile(
					format("%s/mapbuilt_%05u_", OUT_DIR_MAPS.c_str(), step));

				if (mostLikMap->countMapsByClass<COccupancyGridMap2D>() > 0)
				{
					mrpt::img::CImage img;
					mapBuilder->drawCurrentEstimationToImage(&img);
					img.saveToFile(
						format("%s/mapping_%05u.png", OUT_DIR, step));
				}
			}

			// Save a 3D scene view of the mapping process:
			COpenGLScene::Ptr scene;
			if (SAVE_3D_SCENE || SHOW_PROGRESS_IN_WINDOW)
			{
				scene = std::make_shared<COpenGLScene>();

				// The ground:
				mrpt::opengl::CGridPlaneXY::Ptr groundPlane =
					mrpt::opengl::CGridPlaneXY::Create(
						-200, 200, -200, 200, 0, 5);
				groundPlane->setColor(0.4, 0.4, 0.4);
				scene->insert(groundPlane);

				// The camera pointing to the current robot pose:
				if (CAMERA_3DSCENE_FOLLOWS_ROBOT)
				{
					mrpt::opengl::CCamera::Ptr objCam =
						mrpt::opengl::CCamera::Create();
					CPose3D robotPose;
					curPDF.getMean(robotPose);

					objCam->setPointingAt(robotPose);
					objCam->setAzimuthDegrees(-30);
					objCam->setElevationDegrees(30);
					scene->insert(objCam);
				}
				// Draw the map(s):
				mrpt::opengl::CSetOfObjects::Ptr objs =
					mrpt::opengl::CSetOfObjects::Create();
				mostLikMap->getAs3DObject(objs);
				scene->insert(objs);

				// Draw the robot particles:
				size_t M = mapBuilder->mapPDF.particlesCount();
				mrpt::opengl::CSetOfLines::Ptr objLines =
					mrpt::opengl::CSetOfLines::Create();
				objLines->setColor(0, 1, 1);
				for (size_t i = 0; i < M; i++)
				{
					std::deque<TPose3D> path;
					mapBuilder->mapPDF.getPath(i, path);

					float x0 = 0, y0 = 0, z0 = 0;
					for (auto& k : path)
					{
						objLines->appendLine(
							x0, y0, z0 + 0.001, k.x, k.y, k.z + 0.001);
						x0 = k.x;
						y0 = k.y;
						z0 = k.z;
					}
				}
				scene->insert(objLines);

				// An ellipsoid:
				CPose3D lastMeanPose;
				float minDistBtwPoses = -1;
				std::deque<TPose3D> dummyPath;
				mapBuilder->mapPDF.getPath(0, dummyPath);
				for (int k = (int)dummyPath.size() - 1; k >= 0; k--)
				{
					CPose3DPDFParticles poseParts;
					mapBuilder->mapPDF.getEstimatedPosePDFAtTime(k, poseParts);

					const auto [COV, meanPose] =
						poseParts.getCovarianceAndMean();

					if (meanPose.distanceTo(lastMeanPose) > minDistBtwPoses)
					{
						CMatrixDouble33 COV3 = COV.blockCopy<3, 3>(0, 0);

						minDistBtwPoses = 6 * sqrt(COV3(0, 0) + COV3(1, 1));

						opengl::CEllipsoid3D::Ptr objEllip =
							std::make_shared<opengl::CEllipsoid3D>();
						objEllip->setLocation(
							meanPose.x(), meanPose.y(), meanPose.z() + 0.001);
						objEllip->setCovMatrix(COV3, COV3(2, 2) == 0 ? 2 : 3);

						objEllip->setColor(0, 0, 1);
						objEllip->enableDrawSolid3D(false);
						scene->insert(objEllip);

						lastMeanPose = meanPose;
					}
				}
			}  // end if show or save 3D scene->

			if (SAVE_3D_SCENE)
			{  // Save as file:
				CFileGZOutputStream f(format(
					"%s/buildingmap_%05u.3Dscene", OUT_DIR_3D.c_str(), step));
				mrpt::serialization::archiveFrom(f) << *scene;
			}

			if (SHOW_PROGRESS_IN_WINDOW)
			{
				COpenGLScene::Ptr& scenePtr = win3D->get3DSceneAndLock();
				scenePtr = scene;
				win3D->unlockAccess3DScene();

				win3D->forceRepaint();
				int add_delay =
					SHOW_PROGRESS_IN_WINDOW_DELAY_MS - t_exec * 1000;
				if (add_delay > 0)
					std::this_thread::sleep_for(
						std::chrono::milliseconds(add_delay));
			}

			// Save the weighted entropy of each map:
			// ----------------------------------------
			if (GENERATE_LOG_JOINT_H)
			{
				tictac_JH.Tic();

				double H_joint = mapBuilder->getCurrentJointEntropy();
				double H_path = mapBuilder->mapPDF.getCurrentEntropyOfPaths();
				f_jinfo.printf("%e %e\n", H_joint, H_path);
				MRPT_LOG_INFO_FMT(
					"Saving joing H info. joint-H=%f\t(in %.03fms)", H_joint,
					1000.0f * tictac_JH.Tac());
			}

		}  // end of LOG_FREQ

		// Save the memory usage:
		// ------------------------------------------------------------------
		{
			unsigned long memUsage = mrpt::system::getMemoryUsage();
			FILE* f = os::fopen(
				format("%s/log_MemoryUsage.txt", OUT_DIR).c_str(), "at");
			if (f)
			{
				os::fprintf(f, "%u\t%lu\n", step, memUsage);
				os::fclose(f);
			}
			MRPT_LOG_INFO_FMT(
				"Saving memory usage: %.04f MiB", memUsage / (1024.0 * 1024.0));
		}

		// Save the parts stats:
		f_partStats.printf(
			"%u %u %f\n", (unsigned int)step, (unsigned int)curPDF.size(),
			curPDF.ESS());

		// Save the robot estimated pose for each step:
		CPose3D meanPose;
		mapBuilder->getCurrentPoseEstimation()->getMean(meanPose);

		f_path.printf(
			"%u %f %f %f %f %f %f %f\n", (unsigned int)step, meanPose.x(),
			meanPose.y(), meanPose.z(), meanPose.yaw(), meanPose.pitch(),
			meanPose.roll(), mrpt::Clock::toDouble(observations_timestamp));

		// Also keep the robot path as a vector, for the convenience of the app
		// user:
		out_estimated_path[observations_timestamp] = meanPose.asTPose();

		f_pathOdo.printf(
			"%i\t%f\t%f\t%f\t%f\t%f\t%f\n", step, odoPose.x(), odoPose.y(),
			odoPose.z(), odoPose.yaw(), odoPose.pitch(), odoPose.roll());

		step++;
		MRPT_LOG_INFO_FMT("------------- STEP %u ----------------", step);

	};  // end while

	MRPT_LOG_INFO_FMT(
		"----------- **END** (total time: %.03f sec) ---------",
		tictacGlobal.Tac());

	// Save map:
	mapBuilder->getCurrentlyBuiltMap(finalMap);

	CFileOutputStream filOut(format("%s/_finalmap_.simplemap", OUT_DIR));
	mrpt::serialization::archiveFrom(filOut) << finalMap;

	// Save gridmap extend (if exists):
	const CMultiMetricMap* mostLikMap =
		mapBuilder->mapPDF.getCurrentMostLikelyMetricMap();
	mostLikMap->saveMetricMapRepresentationToFile(
		format("%s/finalMap", OUT_DIR));

	// Save the most likely path of the particle set
	FILE* f_pathPart;

	os::sprintf(strFil, 1000, "%s/most_likely_path.txt", OUT_DIR);
	f_pathPart = os::fopen(strFil, "wt");

	ASSERT_(f_pathPart != nullptr);

	std::deque<TPose3D> outPath;
	std::deque<TPose3D>::iterator itPath;

	mapBuilder->getCurrentMostLikelyPath(outPath);

	for (itPath = outPath.begin(); itPath != outPath.end(); itPath++)
		os::fprintf(
			f_pathPart, "%.3f %.3f %.3f\n", itPath->x, itPath->y, itPath->yaw);

	os::fclose(f_pathPart);

	// Close 3D window, if any:
	if (win3D) win3D->waitForKey();

	MRPT_END
}

// ---------------------------------------
//   RBPF_SLAM_App_Rawlog
// ---------------------------------------
RBPF_SLAM_App_Rawlog::RBPF_SLAM_App_Rawlog()
{
	setLoggerName("RBPF_SLAM_App_Rawlog");
}

void RBPF_SLAM_App_Rawlog::impl_initialize(int argc, const char** argv)
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

	// Set relative path for externally-stored images in rawlogs:
	std::string rawlog_images_path =
		mrpt::system::extractFileDirectory(m_rawlogFileName);
	rawlog_images_path += "/Images";
	mrpt::img::CImage::setImagesPathBase(rawlog_images_path);

	MRPT_END
}
