/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2019, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

/*---------------------------------------------------------------
	APPLICATION: Particle Filter (Global) Localization Demo
	FILE: pf_localization_main.cpp
	AUTHOR: Jose Luis Blanco Claraco <joseluisblancoc@gmail.com>

	For instructions and more:
	 https://www.mrpt.org/list-of-mrpt-apps/application-pf-localization
  ---------------------------------------------------------------*/

#include <mrpt/bayes/CParticleFilter.h>
#include <mrpt/config/CConfigFile.h>
#include <mrpt/gui/CDisplayWindow3D.h>
#include <mrpt/gui/CDisplayWindowPlots.h>
#include <mrpt/io/CFileGZInputStream.h>
#include <mrpt/io/CFileGZOutputStream.h>
#include <mrpt/io/CFileOutputStream.h>
#include <mrpt/maps/CMultiMetricMap.h>
#include <mrpt/maps/COccupancyGridMap2D.h>
#include <mrpt/maps/CSimpleMap.h>
#include <mrpt/maps/CSimplePointsMap.h>
#include <mrpt/math/data_utils.h>
#include <mrpt/math/distributions.h>
#include <mrpt/math/ops_vectors.h>  // << for vector<>
#include <mrpt/math/utils.h>
#include <mrpt/obs/CActionCollection.h>
#include <mrpt/obs/CActionRobotMovement2D.h>
#include <mrpt/obs/CActionRobotMovement3D.h>
#include <mrpt/obs/CObservationOdometry.h>
#include <mrpt/obs/CRawlog.h>
#include <mrpt/opengl/CDisk.h>
#include <mrpt/opengl/CEllipsoid.h>
#include <mrpt/opengl/CGridPlaneXY.h>
#include <mrpt/opengl/CPointCloud.h>
#include <mrpt/opengl/stock_objects.h>
#include <mrpt/poses/CPose2D.h>
#include <mrpt/poses/CPose2DInterpolator.h>
#include <mrpt/random.h>
#include <mrpt/serialization/CArchive.h>
#include <mrpt/slam/CMonteCarloLocalization2D.h>
#include <mrpt/slam/CMonteCarloLocalization3D.h>
#include <mrpt/system/CTicTac.h>
#include <mrpt/system/filesystem.h>
#include <mrpt/system/os.h>
#include <mrpt/system/vector_loadsave.h>
#include <Eigen/Dense>
#include <thread>

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
template <class MONTECARLO_TYPE>
void do_pf_localization(
	const std::string& iniFilename, const std::string& cmdline_rawlog_file);
void getGroundTruth(
	CPose2D& expectedPose, size_t rawlogEntry, const CMatrixDouble& GT,
	const Clock::time_point& cur_time);
void prepareGT(const CMatrixDouble& GT);

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

		// Detect 2D vs 3D particle filter?
		CConfigFile cfg(argv[1]);
		const bool is_3D =
			cfg.read_bool("LocalizationExperiment", "use_3D_poses", false);

		if (is_3D)
			do_pf_localization<CMonteCarloLocalization3D>(
				argv[1], cmdline_rawlog_file);
		else
			do_pf_localization<CMonteCarloLocalization2D>(
				argv[1], cmdline_rawlog_file);

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

template <class PDF>
struct pf2gauss_t
{
};

template <>
struct pf2gauss_t<CMonteCarloLocalization2D>
{
	using type = CPosePDFGaussian;
	static constexpr bool PF_IS_3D = false;
};
template <>
struct pf2gauss_t<CMonteCarloLocalization3D>
{
	using type = CPose3DPDFGaussian;
	static constexpr bool PF_IS_3D = true;
};

// ------------------------------------------------------
//				do_pf_localization
// ------------------------------------------------------
template <class MONTECARLO_TYPE>
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
	const auto NUM_REPS = cfg.read_uint64_t(sect, "experimentRepetitions", 1);
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

	// Default odometry uncertainty parameters in "actOdom2D_params" depending
	// on how fast the robot moves, etc...
	//  Only used for observations-only rawlogs:
	CActionRobotMovement2D::TMotionModelOptions actOdom2D_params;
	actOdom2D_params.modelSelection = CActionRobotMovement2D::mmGaussian;
	actOdom2D_params.gaussianModel.minStdXY =
		cfg.read_double("DummyOdometryParams", "minStdXY", 0.04);
	actOdom2D_params.gaussianModel.minStdPHI =
		DEG2RAD(cfg.read_double("DummyOdometryParams", "minStdPHI", 2.0));

	CActionRobotMovement3D::TMotionModelOptions actOdom3D_params;
	actOdom3D_params.mm6DOFModel.additional_std_XYZ =
		cfg.read_double("DummyOdometryParams", "additional_std_XYZ", 0.01);
	actOdom3D_params.mm6DOFModel.additional_std_angle = DEG2RAD(
		cfg.read_double("DummyOdometryParams", "additional_std_angle", 0.1));

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
	CTicTac tictac;
	CSimpleMap simpleMap;
	CParticleFilter::TParticleFilterStats PF_stats;

	// Load the set of metric maps to consider in the experiments:
	CMultiMetricMap metricMap;
	metricMap.setListOfMaps(mapList);
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
			auto grid = metricMap.mapByClass<COccupancyGridMap2D>();
			ASSERT_(grid);
			{
				CFileGZInputStream f(MAP_FILE);
				archiveFrom(f) >> (*grid);
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

		prepareGT(GT);
	}
	else
		printf("Ground truth file: NO\n");

	// PDF initialization uniform distribution limits:
	const auto init_min = mrpt::math::TPose3D(
		cfg.read_double(sect, "init_PDF_min_x", 0),
		cfg.read_double(sect, "init_PDF_min_y", 0),
		cfg.read_double(sect, "init_PDF_min_z", 0),
		mrpt::DEG2RAD(cfg.read_double(sect, "init_PDF_min_yaw_deg", -180.0)),
		mrpt::DEG2RAD(cfg.read_double(sect, "init_PDF_min_pitch_deg", 0)),
		mrpt::DEG2RAD(cfg.read_double(sect, "init_PDF_min_roll_deg", 0)));

	const auto init_max = mrpt::math::TPose3D(
		cfg.read_double(sect, "init_PDF_max_x", 0),
		cfg.read_double(sect, "init_PDF_max_y", 0),
		cfg.read_double(sect, "init_PDF_max_z", 0),
		mrpt::DEG2RAD(cfg.read_double(sect, "init_PDF_max_yaw_deg", +180.0)),
		mrpt::DEG2RAD(cfg.read_double(sect, "init_PDF_max_pitch_deg", 0)),
		mrpt::DEG2RAD(cfg.read_double(sect, "init_PDF_max_roll_deg", 0)));

	std::cout << "Initial PDF limits:\n Min=" << init_min.asString()
			  << "\n Max=" << init_max.asString() << "\n";

	// Gridmap / area of initial uncertainty:
	COccupancyGridMap2D::TEntropyInfo gridInfo;
	if (auto grid = metricMap.mapByClass<COccupancyGridMap2D>(); grid)
	{
		grid->computeEntropy(gridInfo);
		printf(
			"The gridmap has %.04fm2 observed area, %u observed cells\n",
			gridInfo.effectiveMappedArea,
			(unsigned)gridInfo.effectiveMappedCells);
	}
	else
	{
		gridInfo.effectiveMappedArea =
			(init_max.x - init_min.x) * (init_max.y - init_min.y);
	}

	for (int PARTICLE_COUNT : particles_count)
	{
		printf(
			"Initial PDF: %f particles/m2\n",
			PARTICLE_COUNT / gridInfo.effectiveMappedArea);

		// Global stats for all the experiment loops:
		int nConvergenceTests = 0, nConvergenceOK = 0;
		CVectorDouble convergenceErrors;
		std::mutex convergenceErrors_mtx;

		// --------------------------------------------------------------------
		//					EXPERIMENT REPETITIONS LOOP
		// --------------------------------------------------------------------

		// Generate list of repetition indices:
		std::vector<unsigned int> rep_indices(NUM_REPS);
		std::iota(rep_indices.begin(), rep_indices.end(), 0);

		// Run each repetition in parallel:
		// GCC 7 still has not implemented C++17 for_each(std::execution::par
		// So: home-made parallel loops:

		// std::for_each(std::execution::par, rep_indices.begin(),
		// rep_indices.end(),
		auto run_localization_code = [&](const size_t repetition) {
			CVectorDouble indivConvergenceErrors, executionTimes, odoError;
			cout << "\n----------------------------------------------------"
					"----"
					"-----\n"
				 << "      RUNNING FOR " << PARTICLE_COUNT
				 << " INITIAL PARTICLES  - Repetition " << 1 + repetition
				 << " / " << NUM_REPS << "\n"
				 << "------------------------------------------------------"
					"----"
					"---\n\n";

			// Create 3D window if requested:
			CDisplayWindow3D::Ptr win3D;
			if (SHOW_PROGRESS_3D_REAL_TIME)
			{
				win3D = std::make_shared<CDisplayWindow3D>(
					"pf-localization - The MRPT project", 1000, 600);
				win3D->setCameraAzimuthDeg(-45);
			}

			// Create the 3D scene and get the map only once, later we'll modify
			// only the particles, etc..
			COpenGLScene scene;
			{
				mrpt::math::TPoint3D bbox_max(50, 50, 0), bbox_min(-50, -50, 0);
				if (auto pts = metricMap.getAsSimplePointsMap(); pts)
				{
					pts->boundingBox(bbox_min, bbox_max);
				}

				scene.insert(mrpt::opengl::CGridPlaneXY::Create(
					bbox_min.x, bbox_max.x, bbox_min.y, bbox_max.y, 0, 5));

				if (win3D)
					win3D->setCameraZoom(
						2 *
						std::max(
							bbox_max.x - bbox_min.x, bbox_max.y - bbox_min.y));

				CSetOfObjects::Ptr gl_obj = std::make_shared<CSetOfObjects>();
				metricMap.getAs3DObject(gl_obj);
				scene.insert(gl_obj);
			}

			// --------------------------
			// Load the rawlog:
			// --------------------------
			printf("Opening the rawlog file...");
			CFileGZInputStream rawlog_in_stream(RAWLOG_FILE);
			printf("OK\n");

			// The experiment directory is:
			string sOUT_DIR_PARTS, sOUT_DIR_3D;
			const auto sOUT_DIR = format(
				"%s_%03u_%07i", OUT_DIR_PREFIX.c_str(), repetition,
				PARTICLE_COUNT);
			printf("Creating directory: %s\n", sOUT_DIR.c_str());
			createDirectory(sOUT_DIR);
			ASSERT_(fileExists(sOUT_DIR));

			if (!SAVE_STATS_ONLY)
			{
				sOUT_DIR_PARTS = format("%s/particles", sOUT_DIR.c_str());
				sOUT_DIR_3D = format("%s/3D", sOUT_DIR.c_str());

				deleteFiles(format("%s/*.*", sOUT_DIR.c_str()));

				printf("Creating directory: %s\n", sOUT_DIR_PARTS.c_str());
				createDirectory(sOUT_DIR_PARTS);
				ASSERT_(fileExists(sOUT_DIR_PARTS));
				deleteFiles(format("%s/*.*", sOUT_DIR_PARTS.c_str()));

				printf("Creating directory: %s\n", sOUT_DIR_3D.c_str());
				createDirectory(sOUT_DIR_3D);
				ASSERT_(fileExists(sOUT_DIR_3D));
				deleteFiles(format("%s/*.*", sOUT_DIR_3D.c_str()));

				using namespace std::string_literals;
				metricMap.saveMetricMapRepresentationToFile(sOUT_DIR + "/map"s);
			}

			int M = PARTICLE_COUNT;

			MONTECARLO_TYPE pdf;

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
					sect, "init_PDF_mode", false,
					/*Fail if not found*/ true))
			{
				// Reset uniform on free space:
				if constexpr (pf2gauss_t<MONTECARLO_TYPE>::PF_IS_3D)
				{
					THROW_EXCEPTION(
						"init_PDF_mode=0 not supported for 3D particles");
				}
				else
				{
					pdf.resetUniformFreeSpace(
						metricMap.mapByClass<COccupancyGridMap2D>().get(), 0.7f,
						PARTICLE_COUNT, init_min.x, init_max.x, init_min.y,
						init_max.y, init_min.yaw, init_max.yaw);
				}
			}
			else
			{
				// Reset uniform:
				if constexpr (pf2gauss_t<MONTECARLO_TYPE>::PF_IS_3D)
				{
					pdf.resetUniform(init_min, init_max, PARTICLE_COUNT);
				}
				else
				{
					pdf.resetUniform(
						init_min.x, init_max.x, init_min.y, init_max.y,
						init_min.yaw, init_max.yaw, PARTICLE_COUNT);
				}
			}

			printf(
				"PDF of %u particles initialized in %.03fms\n", M,
				1000 * tictac.Tac());

			pdf.saveToTextFile(
				format("%s/particles_0_initial.txt", sOUT_DIR_PARTS.c_str()));

			// -----------------------------
			//		Particle filter
			// -----------------------------
			CPose2D odometryEstimation = initial_odo;

			using PDF_MEAN_TYPE = typename MONTECARLO_TYPE::type_value;

			PDF_MEAN_TYPE pdfEstimation;

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

				// Determine if we are reading a Act-SF or an Obs-only
				// rawlog:
				if (obs)
				{
					// It's an observation-only rawlog: build an auxiliary
					// pair of action-SF, since
					//  montecarlo-localization only accepts those pairs as
					//  input:

					// If it's an odometry reading, don't feed it to the PF.
					// Instead,
					// store its value for use as an "action" together with
					// the next actual observation:
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
						observations = std::make_shared<CSensoryFrame>();
						observations->insert(obs);

						// ActionCollection: Just one action with a dummy
						// odometry
						// ------------------------------------------------------
						action = std::make_shared<CActionCollection>();

						if (pf2gauss_t<MONTECARLO_TYPE>::PF_IS_3D)
						{
							CActionRobotMovement3D actOdom3D;

							const CPose3D odo_incr = CPose3D(
								pending_most_recent_odo - last_used_abs_odo);
							last_used_abs_odo = pending_most_recent_odo;

							actOdom3D.computeFromOdometry(
								odo_incr, actOdom3D_params);
							action->insert(actOdom3D);
						}
						else
						{
							CActionRobotMovement2D actOdom2D;

							const CPose2D odo_incr =
								pending_most_recent_odo - last_used_abs_odo;
							last_used_abs_odo = pending_most_recent_odo;

							actOdom2D.computeFromOdometry(
								odo_incr, actOdom2D_params);
							action->insert(actOdom2D);
						}
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

				int cov_size;
				if (pf2gauss_t<MONTECARLO_TYPE>::PF_IS_3D)
					cov_size = 3;
				else
					cov_size = 2;

				if (step >= rawlog_offset)
				{
					// Do not execute the PF at "step=0", to let the initial
					// PDF to be
					//   reflected in the logs.
					if (step > rawlog_offset)
					{
						// Show 3D?
						if (SHOW_PROGRESS_3D_REAL_TIME)
						{
							const auto [cov, meanPose] =
								pdf.getCovarianceAndMean();

							if (rawlogEntry >= 2)
								getGroundTruth(
									expectedPose, rawlogEntry - 2, GT,
									cur_obs_timestamp);

							// The particles' cov:
							{
								CRenderizable::Ptr ellip =
									scene.getByName("parts_cov");
								CEllipsoid::Ptr el;
								if (!ellip)
								{
									el = std::make_shared<CEllipsoid>();
									ellip =
										mrpt::ptr_cast<CRenderizable>::from(el);
									ellip->setName("parts_cov");
									ellip->setColor(1, 0, 0, 0.6);

									el->setLineWidth(2);
									el->setQuantiles(3);
									el->set2DsegmentsCount(60);
									el->enableDrawSolid3D(false);
									scene.insert(ellip);
								}
								else
								{
									el =
										mrpt::ptr_cast<CEllipsoid>::from(ellip);
								}
								double ellipse_z;
								if constexpr (pf2gauss_t<
												  MONTECARLO_TYPE>::PF_IS_3D)
									ellipse_z = meanPose.z();
								else
									ellipse_z = 0.05;

								ellip->setLocation(
									meanPose.x(), meanPose.y(), ellipse_z);

								el->setCovMatrix(
									mrpt::math::CMatrixDouble(cov), cov_size);
							}

							COpenGLScene::Ptr ptrSceneWin =
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
								mrpt::img::TColorf(1, 1, 1), "mono", 15,
								mrpt::opengl::FILL, 6001, 1.5, 0.1, true);

							win3D->addTextMessage(
								10, 33,
								mrpt::format(
									"#particles= %7u",
									static_cast<unsigned int>(pdf.size())),
								mrpt::img::TColorf(1, 1, 1), "mono", 15,
								mrpt::opengl::FILL, 6002, 1.5, 0.1, true);

							win3D->addTextMessage(
								10, 55,
								mrpt::format(
									"mean pose (x y phi_deg)= %s",
									meanPose.asString().c_str()),
								mrpt::img::TColorf(1, 1, 1), "mono", 15,
								mrpt::opengl::FILL, 6003, 1.5, 0.1, true);

							*ptrSceneWin = scene;
							win3D->unlockAccess3DScene();

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
							locErr += mrpt::hypot_fast(
										  expectedPose.x() - pk.x,
										  expectedPose.y() - pk.y) *
									  exp(pdf.getW(k)) / sumW;
						}
						convergenceErrors_mtx.lock();
						convergenceErrors.push_back(locErr);
						convergenceErrors_mtx.unlock();

						indivConvergenceErrors.push_back(locErr);
						odoError.push_back(
							expectedPose.distanceTo(odometryEstimation));
					}

					const auto [C, M] = pdf.getCovarianceAndMean();
					const auto current_pdf_gaussian =
						typename pf2gauss_t<MONTECARLO_TYPE>::type(M, C);

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
							metricMap.mapByClass<COccupancyGridMap2D>();
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

							ssu_params.robotPose =
								CPosePDFGaussian(current_pdf_gaussian);
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
									ssu_out.scanWithUncert.rangesMean
										.asEigen() +
									3 * ssu_out.scanWithUncert.rangesCovar
											.asEigen()
											.diagonal()
											.array()
											.sqrt()
											.matrix();
								Eigen::VectorXd ci2 =
									ssu_out.scanWithUncert.rangesMean
										.asEigen() -
									3 * ssu_out.scanWithUncert.rangesCovar
											.asEigen()
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

					const auto [cov, meanPose] = pdf.getCovarianceAndMean();

					if ((!SAVE_STATS_ONLY && SCENE3D_FREQ > 0) ||
						SHOW_PROGRESS_3D_REAL_TIME)
					{
						// Generate 3D scene:
						// ------------------------------

						// The Ground Truth (GT):
						if (GT.rows() > 0)
						{
							CRenderizable::Ptr GTpt = scene.getByName("GT");
							if (!GTpt)
							{
								GTpt = std::make_shared<CDisk>();
								GTpt = std::make_shared<CDisk>();
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

							auto p = pdf.template getAs3DObject<
								CSetOfObjects::Ptr>();
							p->setName("particles");
							scene.insert(p);
						}

						// The laser scan:
						{
							CRenderizable::Ptr scanPts =
								scene.getByName("scan");
							if (!scanPts)
							{
								scanPts = std::make_shared<CPointCloud>();
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
						((step + 1) % SCENE3D_FREQ) == 0)
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

			if (win3D && NUM_REPS == 1) mrpt::system::pause();
		};  // for repetitions

		CTicTac tictacGlobal;
		tictacGlobal.Tic();

		const auto max_num_threads = std::thread::hardware_concurrency();
		size_t runs_per_thread = NUM_REPS;
		if (max_num_threads > 1)
			runs_per_thread = static_cast<size_t>(
				std::ceil((NUM_REPS) / static_cast<double>(max_num_threads)));

		std::cout << "Running " << NUM_REPS << " repetitions, on max "
				  << max_num_threads << " parallel threads: " << runs_per_thread
				  << " runs/thread.\n";

		std::vector<std::thread> running_tasks;
		for (size_t r = 0; r < NUM_REPS; r += runs_per_thread)
		{
			auto runner = [&](size_t i_start, size_t i_end) {
				if (i_end > NUM_REPS) i_end = NUM_REPS;  // sanity check
				for (size_t i = i_start; i < i_end; i++)
					run_localization_code(i);
			};

			running_tasks.emplace_back(runner, r, r + runs_per_thread);
			std::this_thread::sleep_for(std::chrono::milliseconds(100));
		}

		// Wait for all threads to end:
		for (auto& t : running_tasks)
			if (t.joinable()) t.join();

		double repetitionTime = tictacGlobal.Tac();

		// Avr. error:
		double covergenceErrorMean = 0, convergenceErrorsMin = 0,
			   convergenceErrorsMax = 0;
		if (!convergenceErrors.empty())
			math::confidenceIntervals(
				convergenceErrors, covergenceErrorMean, convergenceErrorsMin,
				convergenceErrorsMax, STATS_CONF_INTERVAL);

		// Save overall results:
		{
			CFileOutputStream f(
				format("%s_SUMMARY.txt", OUT_DIR_PREFIX.c_str()),
				true /* append */);

			f.printf(
				"%% Ratio_covergence_success  #particles  "
				"average_time_per_execution  convergence_mean_error "
				"convergence_error_conf_int_inf "
				"convergence_error_conf_int_sup "
				"\n");
			if (!nConvergenceTests) nConvergenceTests = 1;
			f.printf(
				"%f %u %f %f %f %f\n",
				((double)nConvergenceOK) / nConvergenceTests, PARTICLE_COUNT,
				repetitionTime / NUM_REPS, covergenceErrorMean,
				convergenceErrorsMin, convergenceErrorsMax);
		}

		printf("\n TOTAL EXECUTION TIME = %.06f sec\n", repetitionTime);

	}  // end of loop for different # of particles
}

static CPose2DInterpolator GT_path;

void prepareGT(const CMatrixDouble& GT)
{
	// Either:
	// - time x y phi
	// or
	// - time x y z yaw pitch roll
	if (GT.cols() == 4 || GT.cols() == 7)
	{
		const bool GT_is_3D = (GT.cols() == 7);
		bool GT_index_is_time = false;

		// First column can be: timestamps, or rawlogentries:
		//  Auto-figure it out:
		if (GT.rows() > 2)
		{
			GT_index_is_time =
				floor(GT(0, 0)) != GT(0, 0) && floor(GT(1, 0)) != GT(1, 0);
		}

		if (GT_index_is_time)
		{
			// Look for the timestamp:
			using namespace std::chrono_literals;
			GT_path.setMaxTimeInterpolation(200ms);

			for (int i = 0; i < GT.rows(); i++)
			{
				GT_path.insert(
					mrpt::Clock::fromDouble(GT(i, 0)),
					TPose2D(GT(i, 1), GT(i, 2), GT(i, GT_is_3D ? 4 : 3)));
			}
		}
	}
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
		bool GT_index_is_time;

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
			using namespace std::chrono_literals;

			bool interp_ok = false;
			GT_path.interpolate(cur_time, expectedPose, interp_ok);
			if (!interp_ok)
			{
				/*
				cerr << format(
					"GT time not found: %f\n",
					mrpt::system::timestampTotime_t(cur_time));
					*/
			}
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
