/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */

/*---------------------------------------------------------------
	APPLICATION: Particle Filter (Global) Localization Demo
	FILE: pf_localization_main.cpp
	AUTHOR: Jose Luis Blanco Claraco <joseluisblancoc@gmail.com>

	For instructions and more:
	 http://www.mrpt.org/list-of-mrpt-apps/application-pf-localization
  ---------------------------------------------------------------*/

#include <mrpt/slam/CMonteCarloLocalization2D.h>

#include <mrpt/config/CConfigFile.h>
#include <mrpt/math/ops_vectors.h>  // << for vector<>
#include <mrpt/system/filesystem.h>
#include <mrpt/gui/CDisplayWindow3D.h>
#include <mrpt/poses/CPose2D.h>
#include <mrpt/poses/CPose2DInterpolator.h>
#include <mrpt/bayes/CParticleFilter.h>
#include <mrpt/random.h>

#include <mrpt/obs/CActionRobotMovement2D.h>
#include <mrpt/obs/CActionCollection.h>
#include <mrpt/obs/CObservationOdometry.h>
#include <mrpt/obs/CRawlog.h>
#include <mrpt/maps/CSimpleMap.h>
#include <mrpt/maps/COccupancyGridMap2D.h>
#include <mrpt/maps/CMultiMetricMap.h>

#include <mrpt/system/os.h>
#include <mrpt/system/vector_loadsave.h>
#include <mrpt/math/distributions.h>
#include <mrpt/math/utils.h>
#include <mrpt/system/CTicTac.h>
#include <mrpt/io/CFileOutputStream.h>
#include <mrpt/io/CFileGZOutputStream.h>
#include <mrpt/io/CFileGZInputStream.h>

#include <mrpt/opengl/CGridPlaneXY.h>
#include <mrpt/opengl/CPointCloud.h>
#include <mrpt/opengl/CEllipsoid.h>
#include <mrpt/opengl/CDisk.h>
#include <mrpt/opengl/stock_objects.h>

#include <mrpt/gui/CDisplayWindowPlots.h>
#include <mrpt/math/data_utils.h>

using namespace mrpt;
using namespace mrpt::slam;
using namespace mrpt::maps;
using namespace mrpt::opengl;
using namespace mrpt::gui;
using namespace mrpt::math;
using namespace mrpt::system;
using namespace mrpt::random;
using namespace mrpt::poses;
using namespace mrpt::bayes;
using namespace mrpt::obs;
using namespace mrpt::io;
using namespace mrpt::img;
using namespace mrpt::config;
using namespace mrpt::serialization;
using namespace std;

// Forward declaration:
void do_pf_localization(
	const std::string& iniFilename, const std::string& cmdline_rawlog_file);
void getGroundTruth(
	CPose2D& expectedPose, size_t rawlogEntry, const CMatrixDouble& GT,
	const Clock::time_point& cur_time);

// ------------------------------------------------------
//						MAIN
// ------------------------------------------------------
int main(int argc, char** argv)
{
	try
	{
		printf(" pf-localization\n");
		printf(
			" MRPT C++ Library: %s - Sources timestamp: %s\n",
			MRPT_getVersion().c_str(), MRPT_getCompilationDate().c_str());
		printf(
			"------------------------------------------------------------------"
			"-\n");

		// Process arguments:
		if (argc != 2 && argc != 3)
		{
			printf("Usage: %s <config.ini> [rawlog_file.rawlog]\n\n", argv[0]);
			return 1;
		}

		// Optional 2º param:
		std::string cmdline_rawlog_file;
		if (argc == 3) cmdline_rawlog_file = std::string(argv[2]);

		do_pf_localization(argv[1], cmdline_rawlog_file);

		return 0;
	}
	catch (exception& e)
	{
		cout << "Caught MRPT exception:\n" << mrpt::exception_to_str(e) << endl;

		// pause();
		return -1;
	}
	catch (...)
	{
		printf("Untyped exception!!");
		// pause();
		return -1;
	}
}

// ------------------------------------------------------
//				do_pf_localization
// ------------------------------------------------------
void do_pf_localization(
	const std::string& ini_fil, const std::string& cmdline_rawlog_file)
{
	ASSERT_FILE_EXISTS_(ini_fil);

	CConfigFile cfg(ini_fil);

	std::vector<int>
		particles_count;  // Number of initial particles (if size>1, run
	// the experiments N times)

	// Load configuration:
	// -----------------------------------------
	const string sect("LocalizationExperiment");

	// Mandatory entries:
	cfg.read_vector(
		sect, "particles_count", std::vector<int>(1, 0), particles_count,
		/*Fail if not found*/ true);
	string OUT_DIR_PREFIX =
		cfg.read_string(sect, "logOutput_dir", "", /*Fail if not found*/ true);

	string RAWLOG_FILE;
	if (cmdline_rawlog_file.empty())
		RAWLOG_FILE = cfg.read_string(
			sect, "rawlog_file", "", /*Fail if not found*/ true);
	else
		RAWLOG_FILE = cmdline_rawlog_file;

	// Non-mandatory entries:
	string MAP_FILE = cfg.read_string(sect, "map_file", "");
	size_t rawlog_offset = cfg.read_int(sect, "rawlog_offset", 0);
	string GT_FILE = cfg.read_string(sect, "ground_truth_path_file", "");
	int NUM_REPS = cfg.read_int(sect, "experimentRepetitions", 1);
	int SCENE3D_FREQ = cfg.read_int(sect, "3DSceneFrequency", 10);
	bool SCENE3D_FOLLOW = cfg.read_bool(sect, "3DSceneFollowRobot", true);
	unsigned int testConvergenceAt =
		cfg.read_int(sect, "experimentTestConvergenceAtStep", -1);

	bool SAVE_STATS_ONLY = cfg.read_bool(sect, "SAVE_STATS_ONLY", false);
	bool DO_RELIABILITY_ESTIMATE = false;
	bool DO_SCAN_LIKELIHOOD_DEBUG = false;
	MRPT_LOAD_CONFIG_VAR(DO_RELIABILITY_ESTIMATE, bool, cfg, sect);
	MRPT_LOAD_CONFIG_VAR(DO_SCAN_LIKELIHOOD_DEBUG, bool, cfg, sect);

	bool SHOW_PROGRESS_3D_REAL_TIME =
		cfg.read_bool(sect, "SHOW_PROGRESS_3D_REAL_TIME", false);
	int SHOW_PROGRESS_3D_REAL_TIME_DELAY_MS =
		cfg.read_int(sect, "SHOW_PROGRESS_3D_REAL_TIME_DELAY_MS", 1);
	double STATS_CONF_INTERVAL =
		cfg.read_double(sect, "STATS_CONF_INTERVAL", 0.2);

	CPose2D initial_odo;
	initial_odo.x(cfg.read_double(sect, "initial_odo_x", 0));
	initial_odo.y(cfg.read_double(sect, "initial_odo_y", 0));
	initial_odo.phi(cfg.read_double(sect, "initial_odo_phi", 0));

#if !MRPT_HAS_WXWIDGETS
	SHOW_PROGRESS_3D_REAL_TIME = false;
#endif

	// Default odometry uncertainty parameters in "dummy_odom_params" depending
	// on how fast the robot moves, etc...
	//  Only used for observations-only rawlogs:
	CActionRobotMovement2D::TMotionModelOptions dummy_odom_params;
	dummy_odom_params.modelSelection = CActionRobotMovement2D::mmGaussian;
	dummy_odom_params.gaussianModel.minStdXY =
		cfg.read_double("DummyOdometryParams", "minStdXY", 0.04);
	dummy_odom_params.gaussianModel.minStdPHI =
		DEG2RAD(cfg.read_double("DummyOdometryParams", "minStdPHI", 2.0));

	// PF-algorithm Options:
	// ---------------------------
	CParticleFilter::TParticleFilterOptions pfOptions;
	pfOptions.loadFromConfigFile(cfg, "PF_options");

	// PDF Options:
	// ------------------
	TMonteCarloLocalizationParams pdfPredictionOptions;
	pdfPredictionOptions.KLD_params.loadFromConfigFile(cfg, "KLD_options");

	// Metric map options:
	// -----------------------------
	TSetOfMetricMapInitializers mapList;
	mapList.loadFromConfigFile(cfg, "MetricMap");

	cout << "-------------------------------------------------------------\n"
		 << "\t RAWLOG_FILE = \t " << RAWLOG_FILE << endl
		 << "\t MAP_FILE = \t " << MAP_FILE << endl
		 << "\t GT_FILE = \t " << GT_FILE << endl
		 << "\t OUT_DIR_PREFIX = \t " << OUT_DIR_PREFIX << endl
		 << "\t #particles = \t " << particles_count << endl
		 << "-------------------------------------------------------------\n";
	pfOptions.dumpToConsole();
	mapList.dumpToConsole();

	// --------------------------------------------------------------------
	//						EXPERIMENT PREPARATION
	// --------------------------------------------------------------------
	CTicTac tictac, tictacGlobal;
	CSimpleMap simpleMap;
	CParticleFilter::TParticleFilterStats PF_stats;

	// Load the set of metric maps to consider in the experiments:
	CMultiMetricMap metricMap;
	metricMap.setListOfMaps(&mapList);
	mapList.dumpToConsole();

	getRandomGenerator().randomize();

	// Load the map (if any):
	// -------------------------
	if (MAP_FILE.size())
	{
		ASSERT_(fileExists(MAP_FILE));

		// Detect file extension:
		// -----------------------------
		string mapExt = lowerCase(extractFileExtension(
			MAP_FILE, true));  // Ignore possible .gz extensions

		if (!mapExt.compare("simplemap"))
		{
			// It's a ".simplemap":
			// -------------------------
			printf("Loading '.simplemap' file...");
			{
				CFileGZInputStream f(MAP_FILE);
				archiveFrom(f) >> simpleMap;
			}
			printf("Ok\n");

			ASSERT_(simpleMap.size() > 0);

			// Build metric map:
			// ------------------------------
			printf("Building metric map(s) from '.simplemap'...");
			metricMap.loadFromProbabilisticPosesAndObservations(simpleMap);
			printf("Ok\n");
		}
		else if (!mapExt.compare("gridmap"))
		{
			// It's a ".gridmap":
			// -------------------------
			printf("Loading gridmap from '.gridmap'...");
			ASSERT_(metricMap.m_gridMaps.size() == 1);
			{
				CFileGZInputStream f(MAP_FILE);
				archiveFrom(f) >> (*metricMap.m_gridMaps[0]);
			}
			printf("Ok\n");
		}
		else
		{
			THROW_EXCEPTION_FMT(
				"Map file has unknown extension: '%s'", mapExt.c_str());
		}
	}

	// Load the Ground Truth:
	CMatrixDouble GT(0, 0);
	if (fileExists(GT_FILE))
	{
		printf("Loading ground truth file...");
		GT.loadFromTextFile(GT_FILE);
		printf("OK\n");
	}
	else
		printf("Ground truth file: NO\n");

	// Create 3D window if requested:
	CDisplayWindow3D::Ptr win3D;
	if (SHOW_PROGRESS_3D_REAL_TIME)
	{
		win3D = mrpt::make_aligned_shared<CDisplayWindow3D>(
			"pf-localization - The MRPT project", 1000, 600);
		win3D->setCameraZoom(20);
		win3D->setCameraAzimuthDeg(-45);
		// win3D->waitForKey();
	}

	// Create the 3D scene and get the map only once, later we'll modify only
	// the particles, etc..
	COpenGLScene scene;

	float init_PDF_min_x = 0, init_PDF_min_y = 0, init_PDF_max_x = 0,
		  init_PDF_max_y = 0;
	MRPT_LOAD_CONFIG_VAR(init_PDF_min_x, float, cfg, sect)
	MRPT_LOAD_CONFIG_VAR(init_PDF_min_y, float, cfg, sect)
	MRPT_LOAD_CONFIG_VAR(init_PDF_max_x, float, cfg, sect)
	MRPT_LOAD_CONFIG_VAR(init_PDF_max_y, float, cfg, sect)

	// Gridmap / area of initial uncertainty:
	COccupancyGridMap2D::TEntropyInfo gridInfo;
	if (metricMap.m_gridMaps.size())
	{
		metricMap.m_gridMaps[0]->computeEntropy(gridInfo);
		printf(
			"The gridmap has %.04fm2 observed area, %u observed cells\n",
			gridInfo.effectiveMappedArea,
			(unsigned)gridInfo.effectiveMappedCells);
	}
	else
	{
		gridInfo.effectiveMappedArea = (init_PDF_max_x - init_PDF_min_x) *
									   (init_PDF_max_y - init_PDF_min_y);
	}

	{
		scene.insert(mrpt::make_aligned_shared<mrpt::opengl::CGridPlaneXY>(
			-50, 50, -50, 50, 0, 5));

		CSetOfObjects::Ptr gl_obj = mrpt::make_aligned_shared<CSetOfObjects>();
		metricMap.getAs3DObject(gl_obj);
		scene.insert(gl_obj);

		if (SHOW_PROGRESS_3D_REAL_TIME)
		{
			COpenGLScene::Ptr ptrScene = win3D->get3DSceneAndLock();

			ptrScene->insert(gl_obj);
			ptrScene->enableFollowCamera(true);

			win3D->unlockAccess3DScene();
		}
	}

	for (int PARTICLE_COUNT : particles_count)
	{
		printf(
			"Initial PDF: %f particles/m2\n",
			PARTICLE_COUNT / gridInfo.effectiveMappedArea);

		// Global stats for all the experiment loops:
		int nConvergenceTests = 0, nConvergenceOK = 0;
		CVectorDouble covergenceErrors;
		// --------------------------------------------------------------------
		//					EXPERIMENT REPETITIONS LOOP
		// --------------------------------------------------------------------
		tictacGlobal.Tic();
		for (int repetition = 0; repetition < NUM_REPS; repetition++)
		{
			CVectorDouble indivConvergenceErrors, executionTimes, odoError;
			cout << "\n--------------------------------------------------------"
					"-----\n"
				 << "      RUNNING FOR " << PARTICLE_COUNT
				 << " INITIAL PARTICLES  - Repetition " << 1 + repetition
				 << " / " << NUM_REPS << "\n"
				 << "----------------------------------------------------------"
					"---\n\n";

			// --------------------------
			// Load the rawlog:
			// --------------------------
			printf("Opening the rawlog file...");
			CFileGZInputStream rawlog_in_stream(RAWLOG_FILE);
			printf("OK\n");

			// The experiment directory is:
			string sOUT_DIR, sOUT_DIR_PARTS, sOUT_DIR_3D;

			if (!SAVE_STATS_ONLY)
			{
				sOUT_DIR =
					format("%s_%03u", OUT_DIR_PREFIX.c_str(), repetition);
				sOUT_DIR_PARTS = format("%s/particles", sOUT_DIR.c_str());
				sOUT_DIR_3D = format("%s/3D", sOUT_DIR.c_str());

				printf("Creating directory: %s\n", sOUT_DIR.c_str());
				createDirectory(sOUT_DIR);
				ASSERT_(fileExists(sOUT_DIR));
				deleteFiles(format("%s/*.*", sOUT_DIR.c_str()));

				printf("Creating directory: %s\n", sOUT_DIR_PARTS.c_str());
				createDirectory(sOUT_DIR_PARTS);
				ASSERT_(fileExists(sOUT_DIR_PARTS));
				deleteFiles(format("%s/*.*", sOUT_DIR_PARTS.c_str()));

				printf("Creating directory: %s\n", sOUT_DIR_3D.c_str());
				createDirectory(sOUT_DIR_3D);
				ASSERT_(fileExists(sOUT_DIR_3D));
				deleteFiles(format("%s/*.*", sOUT_DIR_3D.c_str()));

				if (!metricMap.m_gridMaps.empty())
				{
					metricMap.m_gridMaps[0]->saveAsBitmapFile(
						format("%s/gridmap.png", sOUT_DIR.c_str()));
					CFileOutputStream(
						format("%s/gridmap_limits.txt", sOUT_DIR.c_str()))
						.printf(
							"%f %f %f %f", metricMap.m_gridMaps[0]->getXMin(),
							metricMap.m_gridMaps[0]->getXMax(),
							metricMap.m_gridMaps[0]->getYMin(),
							metricMap.m_gridMaps[0]->getYMax());
				}

				// Save the landmarks for plot in matlab:
				if (metricMap.m_landmarksMap)
					metricMap.m_landmarksMap->saveToMATLABScript2D(
						format("%s/plot_landmarks_map.m", sOUT_DIR.c_str()));
			}

			int M = PARTICLE_COUNT;
			CMonteCarloLocalization2D pdf;

			// PDF Options:
			pdf.options = pdfPredictionOptions;

			pdf.options.metricMap = &metricMap;

			// Create the PF object:
			CParticleFilter PF;
			PF.m_options = pfOptions;

			size_t step = 0;
			size_t rawlogEntry = 0;

			// Initialize the PDF:
			// -----------------------------
			tictac.Tic();
			if (!cfg.read_bool(
					sect, "init_PDF_mode", false, /*Fail if not found*/ true))
				pdf.resetUniformFreeSpace(
					metricMap.m_gridMaps[0].get(), 0.7f, PARTICLE_COUNT,
					init_PDF_min_x, init_PDF_max_x, init_PDF_min_y,
					init_PDF_max_y,
					DEG2RAD(cfg.read_float(sect, "init_PDF_min_phi_deg", -180)),
					DEG2RAD(cfg.read_float(sect, "init_PDF_max_phi_deg", 180)));
			else
				pdf.resetUniform(
					init_PDF_min_x, init_PDF_max_x, init_PDF_min_y,
					init_PDF_max_y,
					DEG2RAD(cfg.read_float(sect, "init_PDF_min_phi_deg", -180)),
					DEG2RAD(cfg.read_float(sect, "init_PDF_max_phi_deg", 180)),
					PARTICLE_COUNT);

			printf(
				"PDF of %u particles initialized in %.03fms\n", M,
				1000 * tictac.Tac());

			// -----------------------------
			//		Particle filter
			// -----------------------------
			CPose2D pdfEstimation, odometryEstimation = initial_odo;
			CMatrixDouble cov;
			bool end = false;

			CFileOutputStream f_cov_est, f_pf_stats, f_odo_est;

			if (!SAVE_STATS_ONLY)
			{
				f_cov_est.open(sOUT_DIR.c_str() + string("/cov_est.txt"));
				f_pf_stats.open(sOUT_DIR.c_str() + string("/PF_stats.txt"));
				f_odo_est.open(sOUT_DIR.c_str() + string("/odo_est.txt"));
			}

			Clock::time_point cur_obs_timestamp;
			CPose2D last_used_abs_odo(0, 0, 0),
				pending_most_recent_odo(0, 0, 0);

			auto arch = archiveFrom(rawlog_in_stream);

			while (!end)
			{
				// Finish if ESC is pushed:
				if (os::kbhit())
					if (os::getch() == 27) end = true;

				// Load pose change from the rawlog:
				// ----------------------------------------
				CActionCollection::Ptr action;
				CSensoryFrame::Ptr observations;
				CObservation::Ptr obs;

				if (!CRawlog::getActionObservationPairOrObservation(
						arch,  // In stream
						action, observations,  // Out pair <action,SF>, or:
						obs,  // Out single observation
						rawlogEntry  // In/Out index counter.
						))
				{
					end = true;
					continue;
				}

				// Determine if we are reading a Act-SF or an Obs-only rawlog:
				if (obs)
				{
					// It's an observation-only rawlog: build an auxiliary pair
					// of action-SF, since
					//  montecarlo-localization only accepts those pairs as
					//  input:

					// If it's an odometry reading, don't feed it to the PF.
					// Instead,
					// store its value for use as an "action" together with the
					// next
					// actual observation:
					if (IS_CLASS(obs, CObservationOdometry))
					{
						auto obs_odo =
							std::dynamic_pointer_cast<CObservationOdometry>(
								obs);
						pending_most_recent_odo = obs_odo->odometry;
						static bool is_1st_odo = true;
						if (is_1st_odo)
						{
							is_1st_odo = false;
							last_used_abs_odo = pending_most_recent_odo;
						}
						continue;
					}
					else
					{
						// SF: Just one observation:
						// ------------------------------------------------------
						observations =
							mrpt::make_aligned_shared<CSensoryFrame>();
						observations->insert(obs);

						// ActionCollection: Just one action with a dummy
						// odometry
						// ------------------------------------------------------
						action = mrpt::make_aligned_shared<CActionCollection>();

						CActionRobotMovement2D dummy_odom;

						const CPose2D odo_incr =
							pending_most_recent_odo - last_used_abs_odo;
						last_used_abs_odo = pending_most_recent_odo;

						dummy_odom.computeFromOdometry(
							odo_incr, dummy_odom_params);
						action->insert(dummy_odom);
					}
				}
				else
				{
					// Already in Act-SF format, nothing else to do!
				}

				CPose2D expectedPose;  // Ground truth

				if (observations->size() > 0)
					cur_obs_timestamp =
						observations->getObservationByIndex(0)->timestamp;

				if (step >= rawlog_offset)
				{
					// Do not execute the PF at "step=0", to let the initial PDF
					// to be
					//   reflected in the logs.
					if (step > rawlog_offset)
					{
						// Show 3D?
						if (SHOW_PROGRESS_3D_REAL_TIME)
						{
							CPose2D meanPose;
							CMatrixDouble33 cov;
							pdf.getCovarianceAndMean(cov, meanPose);

							if (rawlogEntry >= 2)
								getGroundTruth(
									expectedPose, rawlogEntry - 2, GT,
									cur_obs_timestamp);

							COpenGLScene::Ptr ptrScene =
								win3D->get3DSceneAndLock();

							win3D->setCameraPointingToPoint(
								meanPose.x(), meanPose.y(), 0);

							win3D->addTextMessage(
								10, 10,
								mrpt::format(
									"timestamp: %s",
									mrpt::system::dateTimeLocalToString(
										cur_obs_timestamp)
										.c_str()),
								mrpt::img::TColorf(.8f, .8f, .8f), "mono", 15,
								mrpt::opengl::NICE, 6001);

							win3D->addTextMessage(
								10, 33,
								mrpt::format(
									"#particles= %7u",
									static_cast<unsigned int>(pdf.size())),
								mrpt::img::TColorf(.8f, .8f, .8f), "mono", 15,
								mrpt::opengl::NICE, 6002);

							win3D->addTextMessage(
								10, 55,
								mrpt::format(
									"mean pose (x y phi_deg)= %s",
									meanPose.asString().c_str()),
								mrpt::img::TColorf(.8f, .8f, .8f), "mono", 15,
								mrpt::opengl::NICE, 6003);

							{
								CRenderizable::Ptr grid_ground =
									ptrScene->getByName("ground_lines");
								if (!grid_ground)
								{
									grid_ground = mrpt::make_aligned_shared<
										mrpt::opengl::CGridPlaneXY>(
										-50, 50, -50, 50, 0, 5);
									grid_ground->setName("ground_lines");
									ptrScene->insert(grid_ground);
									ptrScene->insert(
										stock_objects::CornerXYZSimple(
											1.0f, 3.0f));
								}
							}

							// The Ground Truth (GT):
							if (GT.rows() > 0)
							{
								CRenderizable::Ptr GTpt =
									ptrScene->getByName("GT");
								if (!GTpt)
								{
									GTpt = mrpt::make_aligned_shared<CDisk>();
									GTpt->setName("GT");
									GTpt->setColor(0, 0, 0, 0.9);

									mrpt::ptr_cast<CDisk>::from(GTpt)
										->setDiskRadius(0.04f);
									ptrScene->insert(GTpt);
								}

								GTpt->setPose(expectedPose);
							}

							// The particles:
							{
								CRenderizable::Ptr parts =
									ptrScene->getByName("particles");
								if (parts) ptrScene->removeObject(parts);

								auto p =
									pdf.getAs3DObject<CSetOfObjects::Ptr>();
								p->setName("particles");
								ptrScene->insert(p);
							}

							// The particles' cov:
							{
								CRenderizable::Ptr ellip =
									ptrScene->getByName("parts_cov");
								if (!ellip)
								{
									ellip =
										mrpt::make_aligned_shared<CEllipsoid>();
									ellip->setName("parts_cov");
									ellip->setColor(1, 0, 0, 0.6);

									mrpt::ptr_cast<CEllipsoid>::from(ellip)
										->setLineWidth(2);
									mrpt::ptr_cast<CEllipsoid>::from(ellip)
										->setQuantiles(3);
									mrpt::ptr_cast<CEllipsoid>::from(ellip)
										->set2DsegmentsCount(60);
									ptrScene->insert(ellip);
								}
								ellip->setLocation(
									meanPose.x(), meanPose.y(), 0.05);

								mrpt::ptr_cast<CEllipsoid>::from(ellip)
									->setCovMatrix(cov, 2);
							}

							// The laser scan:
							{
								CRenderizable::Ptr scanPts =
									ptrScene->getByName("scan");
								if (!scanPts)
								{
									scanPts = mrpt::make_aligned_shared<
										CPointCloud>();
									scanPts->setName("scan");
									scanPts->setColor(1, 0, 0, 0.9);
									mrpt::ptr_cast<CPointCloud>::from(scanPts)
										->enableColorFromZ(false);
									mrpt::ptr_cast<CPointCloud>::from(scanPts)
										->setPointSize(4);
									ptrScene->insert(scanPts);
								}

								CSimplePointsMap map;

								CPose3D robotPose3D(meanPose);

								map.clear();
								observations->insertObservationsInto(&map);

								mrpt::ptr_cast<CPointCloud>::from(scanPts)
									->loadFromPointsMap(&map);
								mrpt::ptr_cast<CPointCloud>::from(scanPts)
									->setPose(robotPose3D);
							}

							// The camera:
							ptrScene->enableFollowCamera(true);

							// Views:
							COpenGLViewport::Ptr view1 =
								ptrScene->getViewport("main");
							{
								CCamera& cam = view1->getCamera();
								cam.setAzimuthDegrees(-90);
								cam.setElevationDegrees(90);
								cam.setPointingAt(meanPose);
								cam.setZoomDistance(5);
								cam.setOrthogonal();
							}

							/*COpenGLViewport::Ptr view2=
							ptrScene->createViewport("small_view"); // Create,
							or get existing one.
							view2->setCloneView("main");
							view2->setCloneCamera(false);
							view2->setBorderSize(3);
							{
								CCamera  &cam = view1->getCamera();
								cam.setAzimuthDegrees(-90);
								cam.setElevationDegrees(90);
								cam.setPointingAt( meanPose );
								cam.setZoomDistance(15);
								cam.setOrthogonal();

								view2->setTransparent(false);
								view2->setViewportPosition(0.59,0.01,0.4,0.3);
							}*/

							win3D->unlockAccess3DScene();

							// Move camera:
							// win3D->setCameraPointingToPoint( curRobotPose.x,
							// curRobotPose.y, curRobotPose.z );

							// Update:
							win3D->forceRepaint();

							std::this_thread::sleep_for(
								std::chrono::milliseconds(
									SHOW_PROGRESS_3D_REAL_TIME_DELAY_MS));
						}  // end show 3D real-time

						// ----------------------------------------
						// RUN ONE STEP OF THE PARTICLE FILTER:
						// ----------------------------------------
						tictac.Tic();
						if (!SAVE_STATS_ONLY)
							printf(
								"Step %u -- Executing ParticleFilter on %u "
								"particles....",
								(unsigned int)step,
								(unsigned int)pdf.particlesCount());

						PF.executeOn(
							pdf,
							action.get(),  // Action
							observations.get(),  // Obs.
							&PF_stats  // Output statistics
						);

						double run_time = tictac.Tac();
						executionTimes.push_back(run_time);
						if (!SAVE_STATS_ONLY)
							printf(
								" Done! in %.03fms, ESS=%f\n", 1e3f * run_time,
								pdf.ESS());
					}

					// Avrg. error:
					// ----------------------------------------
					CActionRobotMovement2D::Ptr best_mov_estim =
						action->getBestMovementEstimation();
					if (best_mov_estim)
					{
						odometryEstimation =
							odometryEstimation +
							best_mov_estim->poseChange->getMeanVal();
					}

					pdf.getMean(pdfEstimation);

					getGroundTruth(
						expectedPose, rawlogEntry, GT, cur_obs_timestamp);

					if (expectedPose.x() != 0 || expectedPose.y() != 0 ||
						expectedPose.phi() != 0)
					{  // Averaged error to GT
						double sumW = 0;
						double locErr = 0;
						for (size_t k = 0; k < pdf.size(); k++)
							sumW += exp(pdf.getW(k));
						for (size_t k = 0; k < pdf.size(); k++)
						{
							const auto pk = pdf.getParticlePose(k);
							locErr +=
								(decltype(pk)(expectedPose.asTPose()) - pk)
									.norm() *
								exp(pdf.getW(k)) / sumW;
						}
						covergenceErrors.push_back(locErr);
						indivConvergenceErrors.push_back(locErr);
						odoError.push_back(
							expectedPose.distanceTo(odometryEstimation));
					}

					CPosePDFGaussian current_pdf_gaussian;
					pdf.getCovarianceAndMean(
						current_pdf_gaussian.cov, current_pdf_gaussian.mean);

					// Text output:
					// ----------------------------------------
					if (!SAVE_STATS_ONLY)
					{
						cout << "    Odometry est: " << odometryEstimation
							 << "\n";
						cout << "         PDF est: " << pdfEstimation
							 << ", ESS (B.R.)= " << PF_stats.ESS_beforeResample
							 << " tr(cov): "
							 << std::sqrt(current_pdf_gaussian.cov.trace())
							 << "\n";
						if (GT.rows() > 0)
							cout << "    Ground truth: " << expectedPose
								 << "\n";
					}

					// Evaluate the "reliability" of the pose estimation
					// (for now, only for 2D laser scans + grid maps)
					double obs_reliability_estim = .0;
					if (DO_RELIABILITY_ESTIMATE)
					{
						// We need: a gridmap & a 2D LIDAR:
						CObservation2DRangeScan::Ptr obs_scan;
						if (observations)
							obs_scan = observations->getObservationByClass<
								CObservation2DRangeScan>(0);  // Get the 0'th
						// scan, if
						// several are
						// present.
						COccupancyGridMap2D::Ptr gridmap =
							metricMap.getMapByClass<COccupancyGridMap2D>();
						if (obs_scan && gridmap)  // We have both, go on:
						{
							// Simulate scan + uncertainty:
							COccupancyGridMap2D::TLaserSimulUncertaintyParams
								ssu_params;
							COccupancyGridMap2D::TLaserSimulUncertaintyResult
								ssu_out;
							ssu_params.method =
								COccupancyGridMap2D::sumUnscented;
							// ssu_params.UT_alpha = 0.99;
							// obs_scan->stdError = 0.07;
							// obs_scan->maxRange = 10.0;

							ssu_params.robotPose = current_pdf_gaussian;
							ssu_params.aperture = obs_scan->aperture;
							ssu_params.rangeNoiseStd = obs_scan->stdError;
							ssu_params.nRays = obs_scan->scan.size();
							ssu_params.rightToLeft = obs_scan->rightToLeft;
							ssu_params.sensorPose = obs_scan->sensorPose;
							ssu_params.maxRange = obs_scan->maxRange;

							gridmap->laserScanSimulatorWithUncertainty(
								ssu_params, ssu_out);

							// Evaluate reliability:
							CObservation2DRangeScanWithUncertainty::TEvalParams
								evalParams;
							// evalParams.prob_outliers = 0.40;
							// evalParams.max_prediction_std_dev = 1.0;
							obs_reliability_estim =
								ssu_out.scanWithUncert.evaluateScanLikelihood(
									*obs_scan, evalParams);

							if (DO_SCAN_LIKELIHOOD_DEBUG)
							{
								static mrpt::gui::CDisplayWindowPlots win;

								std::vector<float> ranges_mean, ranges_obs;
								for (float i :
									 ssu_out.scanWithUncert.rangeScan.scan)
									ranges_mean.push_back(i);
								for (float i : obs_scan->scan)
									ranges_obs.push_back(i);

								win.plot(ranges_mean, "3k-", "mean");
								win.plot(ranges_obs, "r-", "obs");

								Eigen::VectorXd ci1 =
									ssu_out.scanWithUncert.rangesMean +
									3 * ssu_out.scanWithUncert.rangesCovar
											.diagonal()
											.array()
											.sqrt()
											.matrix();
								Eigen::VectorXd ci2 =
									ssu_out.scanWithUncert.rangesMean -
									3 * ssu_out.scanWithUncert.rangesCovar
											.diagonal()
											.array()
											.sqrt()
											.matrix();
								win.plot(ci1, "k-", "CI+");
								win.plot(ci2, "k-", "CI-");

								win.setWindowTitle(mrpt::format(
									"obs_reliability_estim: %f",
									obs_reliability_estim));
								win.axis_fit();
							}
						}
						cout << "    Reliability measure [0-1]: "
							 << obs_reliability_estim << "\n";
					}

					if (!SAVE_STATS_ONLY)
					{
						f_cov_est.printf("%e\n", sqrt(cov.det()));
						f_pf_stats.printf(
							"%u %e %e %f %f\n", (unsigned int)pdf.size(),
							PF_stats.ESS_beforeResample,
							PF_stats.weightsVariance_beforeResample,
							obs_reliability_estim,
							sqrt(current_pdf_gaussian.cov.det()));
						f_odo_est.printf(
							"%f %f %f\n", odometryEstimation.x(),
							odometryEstimation.y(), odometryEstimation.phi());
					}

					CPose2D meanPose;
					CMatrixDouble33 cov;
					pdf.getCovarianceAndMean(cov, meanPose);

					if (!SAVE_STATS_ONLY && SCENE3D_FREQ > 0 &&
						(step % SCENE3D_FREQ) == 0)
					{
						// Generate 3D scene:
						// ------------------------------
						// MRPT_TODO("Someday I should clean up this mess, since
						// two different 3D scenes are built -> refactor code")

						// The Ground Truth (GT):
						if (GT.rows() > 0)
						{
							CRenderizable::Ptr GTpt = scene.getByName("GT");
							if (!GTpt)
							{
								GTpt = mrpt::make_aligned_shared<CDisk>();
								GTpt = mrpt::make_aligned_shared<CDisk>();
								GTpt->setName("GT");
								GTpt->setColor(0, 0, 0, 0.9);

								mrpt::ptr_cast<CDisk>::from(GTpt)
									->setDiskRadius(0.04f);
								scene.insert(GTpt);
							}

							GTpt->setPose(expectedPose);
						}

						// The particles:
						{
							CRenderizable::Ptr parts =
								scene.getByName("particles");
							if (parts) scene.removeObject(parts);

							auto p = pdf.getAs3DObject<CSetOfObjects::Ptr>();
							p->setName("particles");
							scene.insert(p);
						}

						// The particles' cov:
						{
							CRenderizable::Ptr ellip =
								scene.getByName("parts_cov");
							if (!ellip)
							{
								ellip = mrpt::make_aligned_shared<CEllipsoid>();
								ellip->setName("parts_cov");
								ellip->setColor(1, 0, 0, 0.6);

								mrpt::ptr_cast<CEllipsoid>::from(ellip)
									->setLineWidth(4);
								mrpt::ptr_cast<CEllipsoid>::from(ellip)
									->setQuantiles(3);
								mrpt::ptr_cast<CEllipsoid>::from(ellip)
									->set2DsegmentsCount(60);
								scene.insert(ellip);
							}
							ellip->setLocation(meanPose.x(), meanPose.y(), 0);

							mrpt::ptr_cast<CEllipsoid>::from(ellip)
								->setCovMatrix(cov, 2);
						}

						// The laser scan:
						{
							CRenderizable::Ptr scanPts =
								scene.getByName("scan");
							if (!scanPts)
							{
								scanPts =
									mrpt::make_aligned_shared<CPointCloud>();
								scanPts->setName("scan");
								scanPts->setColor(1, 0, 0, 0.9);
								mrpt::ptr_cast<CPointCloud>::from(scanPts)
									->enableColorFromZ(false);
								mrpt::ptr_cast<CPointCloud>::from(scanPts)
									->setPointSize(4);
								scene.insert(scanPts);
							}

							CSimplePointsMap map;

							CPose3D robotPose3D(meanPose);

							map.clear();
							observations->insertObservationsInto(&map);

							mrpt::ptr_cast<CPointCloud>::from(scanPts)
								->loadFromPointsMap(&map);
							mrpt::ptr_cast<CPointCloud>::from(scanPts)->setPose(
								robotPose3D);
						}

						// The camera:
						scene.enableFollowCamera(SCENE3D_FOLLOW);

						// Views:
						COpenGLViewport::Ptr view1 = scene.getViewport("main");
						{
							CCamera& cam = view1->getCamera();
							cam.setAzimuthDegrees(-90);
							cam.setElevationDegrees(90);
							cam.setPointingAt(meanPose);
							cam.setZoomDistance(5);
							cam.setOrthogonal();
						}
					}

					if (!SAVE_STATS_ONLY && SCENE3D_FREQ != -1 &&
						(step % SCENE3D_FREQ) == 0)
					{
						// Save 3D scene:
						CFileGZOutputStream f(format(
							"%s/progress_%05u.3Dscene", sOUT_DIR_3D.c_str(),
							(unsigned)step));
						archiveFrom(f) << scene;

						// Generate text files for matlab:
						// ------------------------------------
						pdf.saveToTextFile(format(
							"%s/particles_%05u.txt", sOUT_DIR_PARTS.c_str(),
							(unsigned)step));
					}

				}  // end if rawlog_offset

				step++;

				// Test for end condition if we are testing convergence:
				if (step == testConvergenceAt)
				{
					nConvergenceTests++;

					// Convergence??
					if (sqrt(cov.det()) < 2)
					{
						if (pdfEstimation.distanceTo(expectedPose) < 2)
							nConvergenceOK++;
					}
					end = true;
				}
			};  // while rawlogEntries

			indivConvergenceErrors.saveToTextFile(sOUT_DIR + "/GT_error.txt");
			odoError.saveToTextFile(sOUT_DIR + "/ODO_error.txt");
			executionTimes.saveToTextFile(sOUT_DIR + "/exec_times.txt");
		}  // for repetitions

		double repetitionTime = tictacGlobal.Tac();

		// Avr. error:
		double covergenceErrorMean, covergenceErrorsMin, covergenceErrorsMax;
		math::confidenceIntervals(
			covergenceErrors, covergenceErrorMean, covergenceErrorsMin,
			covergenceErrorsMax, STATS_CONF_INTERVAL);

		// Save overall results:
		{
			CFileOutputStream f(
				format("%s_SUMMARY.txt", OUT_DIR_PREFIX.c_str()),
				true /* append */);

			f.printf(
				"%% Ratio_covergence_success  #particles  "
				"average_time_per_execution  convergence_mean_error "
				"convergence_error_conf_int_inf convergence_error_conf_int_sup "
				"\n");
			if (!nConvergenceTests) nConvergenceTests = 1;
			f.printf(
				"%f %u %f %f %f %f\n",
				((double)nConvergenceOK) / nConvergenceTests, PARTICLE_COUNT,
				repetitionTime / NUM_REPS, covergenceErrorMean,
				covergenceErrorsMin, covergenceErrorsMax);
		}

		printf("\n TOTAL EXECUTION TIME = %.06f sec\n", repetitionTime);

	}  // end of loop for different # of particles

	if (win3D) mrpt::system::pause();
}

void getGroundTruth(
	CPose2D& expectedPose, size_t rawlogEntry, const CMatrixDouble& GT,
	const Clock::time_point& cur_time)
{
	// Either:
	// - time x y phi
	// or
	// - time x y z yaw pitch roll
	if (GT.cols() == 4 || GT.cols() == 7)
	{
		static bool first_step = true;
		static bool GT_index_is_time;
		const bool GT_is_3D = (GT.cols() == 7);

		// First column can be: timestamps, or rawlogentries:
		//  Auto-figure it out:
		if (GT.rows() > 2)
		{
			GT_index_is_time =
				floor(GT(0, 0)) != GT(0, 0) && floor(GT(1, 0)) != GT(1, 0);
		}
		else
		{
			GT_index_is_time = false;
		}

		if (GT_index_is_time)
		{
			// Look for the timestamp:
			static CPose2DInterpolator GT_path;
			using namespace std::chrono_literals;
			GT_path.setMaxTimeInterpolation(200ms);
			if (first_step)
			{
				for (int i = 0; i < GT.rows(); i++)
				{
					GT_path.insert(
						mrpt::Clock::fromDouble(GT(i, 0)),
						TPose2D(GT(i, 1), GT(i, 2), GT(i, GT_is_3D ? 4 : 3)));
				}
			}
			bool interp_ok = false;
			GT_path.interpolate(cur_time, expectedPose, interp_ok);
			if (!interp_ok)
				cerr << format(
					"GT time not found: %f\n",
					mrpt::system::timestampTotime_t(cur_time));
		}
		else
		{
			// Look for the rawlogEntry:
			size_t k, N = GT.rows();
			for (k = 0; k < N; k++)
			{
				if (GT(k, 0) == rawlogEntry) break;
			}

			if (k < N)
			{
				expectedPose.x(GT(k, 1));
				expectedPose.y(GT(k, 2));
				expectedPose.phi(GT(k, 3));
			}
		}
		first_step = false;
	}
	else if (GT.cols() == 3)
	{
		if ((int)rawlogEntry < GT.rows())
		{
			expectedPose.x(GT(rawlogEntry, 0));
			expectedPose.y(GT(rawlogEntry, 1));
			expectedPose.phi(GT(rawlogEntry, 2));
		}
	}
	else if (GT.cols() > 0)
		THROW_EXCEPTION("Unexpected number of columns in ground truth file");
}
