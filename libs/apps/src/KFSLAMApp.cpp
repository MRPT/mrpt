/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2019, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#include "apps-precomp.h"  // Precompiled headers

#include <mrpt/apps/KFSLAMApp.h>

#include <mrpt/config/CConfigFile.h>
#include <mrpt/gui/CDisplayWindow3D.h>
#include <mrpt/io/CFileGZInputStream.h>
#include <mrpt/io/CFileGZOutputStream.h>
#include <mrpt/io/vector_loadsave.h>
#include <mrpt/math/ops_containers.h>
#include <mrpt/obs/CRawlog.h>
#include <mrpt/opengl/CGridPlaneXY.h>
#include <mrpt/opengl/CSetOfLines.h>
#include <mrpt/opengl/stock_objects.h>
#include <mrpt/slam/CRangeBearingKFSLAM.h>
#include <mrpt/slam/CRangeBearingKFSLAM2D.h>
#include <mrpt/system/filesystem.h>
#include <mrpt/system/os.h>
#include <mrpt/system/string_utils.h>
#include <fstream>

using namespace mrpt::apps;

KFSLAMApp::KFSLAMApp() : mrpt::system::COutputLogger("KFSLAMApp") {}

void KFSLAMApp::initialize(int argc, const char** argv)
{
	MRPT_START

	MRPT_LOG_INFO_FMT(
		" kf-slam - Part of the MRPT\n"
		" MRPT C++ Library: %s - Sources timestamp: %s\n\n",
		mrpt::system::MRPT_getVersion().c_str(),
		mrpt::system::MRPT_getCompilationDate().c_str());

	// Process arguments:
	if (argc < 2)
	{
		THROW_EXCEPTION("Usage: kf-slam <config_file> [dataset.rawlog]");
	}

	// Config file:
	const std::string configFile = std::string(argv[1]);

	ASSERT_FILE_EXISTS_(configFile);
	params.setContent(mrpt::io::file_get_contents(configFile));

	// Rawlog file: from args. line or from config file:
	if (argc == 3)
		rawlogFileName = std::string(argv[2]);
	else
		rawlogFileName = params.read_string(
			"MappingApplication", "rawlog_file", std::string("log.rawlog"));

	MRPT_END
}

void KFSLAMApp::run()
{
	MRPT_START

	// 2D or 3D implementation:
	const auto kf_implementation = mrpt::system::trim(params.read_string(
		"MappingApplication", "kf_implementation", "CRangeBearingKFSLAM"));

	if (kf_implementation == "CRangeBearingKFSLAM")
		Run_KF_SLAM<mrpt::slam::CRangeBearingKFSLAM>();
	else if (kf_implementation == "CRangeBearingKFSLAM2D")
		Run_KF_SLAM<mrpt::slam::CRangeBearingKFSLAM2D>();
	else
		throw std::runtime_error(
			"kf_implementation: Invalid value found in the config file.");

	MRPT_END
}

using namespace mrpt;
using namespace mrpt::slam;
using namespace mrpt::maps;
using namespace mrpt::opengl;
using namespace mrpt::system;
using namespace mrpt::math;
using namespace mrpt::poses;
using namespace mrpt::config;
using namespace mrpt::io;
using namespace mrpt::img;
using namespace mrpt::obs;
using namespace std;

// ------------------------------------------------------
//				traits
// ------------------------------------------------------
template <class IMPL>
struct kfslam_traits;

// Specialization for 2D (3D) SLAM:
template <>
struct kfslam_traits<CRangeBearingKFSLAM2D>
{
	using ekfslam_t = CRangeBearingKFSLAM2D;
	using posepdf_t = CPosePDFGaussian;
	using pose_t = CPose2D;
	using lm_t = TPoint2D;
	template <class ARR>
	static void landmark_to_3d(const ARR& lm, TPoint3D& p)
	{
		p.x = lm[0];
		p.y = lm[1];
		p.z = 0;
	}

	static void doPartitioningExperiment(
		[[maybe_unused]] ekfslam_t& mapping,
		[[maybe_unused]] CMatrixDouble& fullCov,
		[[maybe_unused]] const string& OUT_DIR)
	{
		// Nothing to do
	}
};

// Specialization for 3D (6D) SLAM:
template <>
struct kfslam_traits<CRangeBearingKFSLAM>
{
	using ekfslam_t = CRangeBearingKFSLAM;
	using posepdf_t = CPose3DQuatPDFGaussian;
	using pose_t = CPose3D;
	using lm_t = CPoint3D;

	template <class ARR>
	static void landmark_to_3d(const ARR& lm, TPoint3D& p)
	{
		p.x = lm[0];
		p.y = lm[1];
		p.z = lm[2];
	}

	static void doPartitioningExperiment(
		ekfslam_t& mapping, CMatrixDouble& fullCov, const string& OUT_DIR)
	{
		// Compute the "information" between partitions:
		if (mapping.options.doPartitioningExperiment)
		{
			// --------------------------------------------
			// PART I:
			//  Comparison to fixed partitioning every K obs.
			// --------------------------------------------

			// Compute the information matrix:
			for (size_t i = 0; i < 6; i++)
				fullCov(i, i) = max(fullCov(i, i), 1e-6);

			CMatrixF H(fullCov.inverse_LLt());
			H.saveToTextFile(OUT_DIR + string("/information_matrix_final.txt"));

			// Replace by absolute values:
			H = H.array().abs().matrix();
			CMatrixF H2(H);
			CImage imgF;
			imgF.setFromMatrix(H2, false /*it's not normalized*/);
			imgF.saveToFile(OUT_DIR + string("/information_matrix_final.png"));

			// ----------------------------------------
			// Compute the "approximation error factor" E:
			//  E = SUM() / SUM(ALL ELEMENTS IN MATRIX)
			// ----------------------------------------
			vector<std::vector<uint32_t>> landmarksMembership, partsInObsSpace;
			mrpt::math::CMatrixDouble ERRS(50, 3);

			for (int i = 0; i < ERRS.rows(); i++)
			{
				int K;

				if (i == 0)
				{
					K = 0;
					mapping.getLastPartitionLandmarks(landmarksMembership);
				}
				else
				{
					K = i + 1;
					mapping.getLastPartitionLandmarksAsIfFixedSubmaps(
						i + 1, landmarksMembership);
				}

				mapping.getLastPartition(partsInObsSpace);

				ERRS(i, 0) = K;
				ERRS(i, 1) = partsInObsSpace.size();
				ERRS(i, 2) = mapping.computeOffDiagonalBlocksApproximationError(
					landmarksMembership);
			}

			ERRS.saveToTextFile(OUT_DIR + string("/ERRORS.txt"));

			// --------------------------------------------
			// PART II:
			//  Sweep partitioning threshold:
			// --------------------------------------------
			size_t STEPS = 50;
			CVectorFloat ERRS_SWEEP(STEPS), ERRS_SWEEP_THRESHOLD(STEPS);

			// Compute the error for each partitioning-threshold
			for (size_t i = 0; i < STEPS; i++)
			{
				float th = (1.0f * i) / (STEPS - 1.0f);
				ERRS_SWEEP_THRESHOLD[i] = th;
				mapping.mapPartitionOptions()->partitionThreshold = th;

				mapping.reconsiderPartitionsNow();

				mapping.getLastPartitionLandmarks(landmarksMembership);
				ERRS_SWEEP[i] =
					mapping.computeOffDiagonalBlocksApproximationError(
						landmarksMembership);
			}

			ERRS_SWEEP.saveToTextFile(OUT_DIR + string("/ERRORS_SWEEP.txt"));
			ERRS_SWEEP_THRESHOLD.saveToTextFile(
				OUT_DIR + string("/ERRORS_SWEEP_THRESHOLD.txt"));

		}  // end if doPartitioningExperiment
	}
};

template <class IMPL>
void KFSLAMApp::Run_KF_SLAM()
{
	MRPT_START

	auto& cfgFile = params;  // for backwards compat of old code

	// The EKF-SLAM class:
	// Traits for this KF implementation (2D or 3D)
	using traits_t = kfslam_traits<IMPL>;
	using ekfslam_t = typename traits_t::ekfslam_t;

	ekfslam_t mapping;

	// The rawlog file:
	// ----------------------------------------
	const unsigned int rawlog_offset =
		cfgFile.read_int("MappingApplication", "rawlog_offset", 0);

	const unsigned int SAVE_LOG_FREQUENCY =
		cfgFile.read_int("MappingApplication", "SAVE_LOG_FREQUENCY", 1);

	const bool SAVE_DA_LOG =
		cfgFile.read_bool("MappingApplication", "SAVE_DA_LOG", true);

	const bool SAVE_3D_SCENES =
		cfgFile.read_bool("MappingApplication", "SAVE_3D_SCENES", true);
	const bool SAVE_MAP_REPRESENTATIONS = cfgFile.read_bool(
		"MappingApplication", "SAVE_MAP_REPRESENTATIONS", true);
	bool SHOW_3D_LIVE =
		cfgFile.read_bool("MappingApplication", "SHOW_3D_LIVE", false);
	const bool CAMERA_3DSCENE_FOLLOWS_ROBOT = cfgFile.read_bool(
		"MappingApplication", "CAMERA_3DSCENE_FOLLOWS_ROBOT", false);

#if !MRPT_HAS_WXWIDGETS
	SHOW_3D_LIVE = false;
#endif

	string OUT_DIR = cfgFile.read_string(
		"MappingApplication", "logOutput_dir", "OUT_KF-SLAM");
	string ground_truth_file =
		cfgFile.read_string("MappingApplication", "ground_truth_file", "");
	string ground_truth_file_robot = cfgFile.read_string(
		"MappingApplication", "ground_truth_file_robot", "");

	string ground_truth_data_association = cfgFile.read_string(
		"MappingApplication", "ground_truth_data_association", "");

	MRPT_LOG_INFO_STREAM("RAWLOG FILE: " << rawlogFileName);
	ASSERT_FILE_EXISTS_(rawlogFileName);
	CFileGZInputStream rawlogFile(rawlogFileName);

	deleteFilesInDirectory(OUT_DIR);
	createDirectory(OUT_DIR);

	// Load the config options for mapping:
	// ----------------------------------------
	mapping.loadOptions(cfgFile);
	{
		std::stringstream o;
		mapping.KF_options.dumpToTextStream(o);
		mapping.options.dumpToTextStream(o);
		MRPT_LOG_INFO(o.str());
	}

	// debug:
	// mapping.KF_options.use_analytic_observation_jacobian = true;
	// mapping.KF_options.use_analytic_transition_jacobian = true;
	// mapping.KF_options.debug_verify_analytic_jacobians = true;

	// Is there ground truth of the robot poses??
	CMatrixDouble GT_PATH(0, 0);
	if (ground_truth_file_robot.size() && fileExists(ground_truth_file_robot))
	{
		GT_PATH.loadFromTextFile(ground_truth_file_robot);
		ASSERT_(GT_PATH.rows() > 0 && GT_PATH.cols() == 6);
	}

	// Is there a ground truth file of the data association?
	// Map: timestamp -> vector(index in observation -> real index)
	std::map<double, std::vector<int>> GT_DA;
	// Landmark indices bimapping: SLAM DA <---> GROUND TRUTH DA
	mrpt::containers::bimap<int, int> DA2GTDA_indices;
	if (!ground_truth_data_association.empty() &&
		fileExists(ground_truth_data_association))
	{
		CMatrixDouble mGT_DA;
		mGT_DA.loadFromTextFile(ground_truth_data_association);
		ASSERT_ABOVEEQ_(mGT_DA.cols(), 3);
		// Convert the loaded matrix into a std::map in GT_DA:
		for (int i = 0; i < mGT_DA.rows(); i++)
		{
			std::vector<int>& v = GT_DA[mGT_DA(i, 0)];
			if (v.size() <= mGT_DA(i, 1)) v.resize(mGT_DA(i, 1) + 1);
			v[mGT_DA(i, 1)] = mGT_DA(i, 2);
		}
		MRPT_LOG_INFO_STREAM(
			"Loaded " << GT_DA.size() << " entries from DA ground truth file.");
	}

	// Create output file for DA perf:
	std::ofstream out_da_performance_log;
	{
		const std::string f = std::string(
			OUT_DIR + std::string("/data_association_performance.log"));
		out_da_performance_log.open(f.c_str());
		ASSERTMSG_(
			out_da_performance_log.is_open(),
			std::string("Error writing to: ") + f);

		// Header:
		out_da_performance_log
			<< "%           TIMESTAMP                INDEX_IN_OBS    TruePos "
			   "FalsePos TrueNeg FalseNeg  NoGroundTruthSoIDontKnow \n"
			<< "%--------------------------------------------------------------"
			   "--------------------------------------------------\n";
	}

	mrpt::gui::CDisplayWindow3D::Ptr win3d;

	if (SHOW_3D_LIVE)
	{
		win3d =
			mrpt::gui::CDisplayWindow3D::Create("KF-SLAM live view", 800, 500);

		win3d->addTextMessage(
			0.01, 0.96, "Red: Estimated path", TColorf(0.8f, 0.8f, 0.8f), 100,
			MRPT_GLUT_BITMAP_HELVETICA_10);
		win3d->addTextMessage(
			0.01, 0.93, "Black: Ground truth path", TColorf(0.8f, 0.8f, 0.8f),
			101, MRPT_GLUT_BITMAP_HELVETICA_10);
	}

	// Create DA-log output file:
	std::ofstream out_da_log;
	if (SAVE_DA_LOG)
	{
		const std::string f =
			std::string(OUT_DIR + std::string("/data_association.log"));
		out_da_log.open(f.c_str());
		ASSERTMSG_(out_da_log.is_open(), std::string("Error writing to: ") + f);

		// Header:
		out_da_log << "%           TIMESTAMP                INDEX_IN_OBS    ID "
					  "   RANGE(m)    YAW(rad)   PITCH(rad) \n"
				   << "%-------------------------------------------------------"
					  "-------------------------------------\n";
	}

	// The main loop:
	// ---------------------------------------
	CActionCollection::Ptr action;
	CSensoryFrame::Ptr observations;
	size_t rawlogEntry = 0, step = 0;

	vector<TPose3D> meanPath;  // The estimated path
	typename traits_t::posepdf_t robotPose;
	const bool is_pose_3d = robotPose.state_length != 3;

	std::vector<typename IMPL::landmark_point_t> LMs;
	std::map<unsigned int, CLandmark::TLandmarkID> LM_IDs;
	CMatrixDouble fullCov;
	CVectorDouble fullState;
	CTicTac kftictac;

	auto rawlogArch = mrpt::serialization::archiveFrom(rawlogFile);

	for (;;)
	{
		if (os::kbhit())
		{
			char pushKey = os::getch();
			if (27 == pushKey) break;
		}

		// Load action/observation pair from the rawlog:
		// --------------------------------------------------
		if (!CRawlog::readActionObservationPair(
				rawlogArch, action, observations, rawlogEntry))
			break;  // file EOF

		if (rawlogEntry >= rawlog_offset)
		{
			// Process the action and observations:
			// --------------------------------------------
			kftictac.Tic();

			mapping.processActionObservation(action, observations);

			const double tim_kf_iter = kftictac.Tac();

			// Get current state:
			// -------------------------------
			mapping.getCurrentState(robotPose, LMs, LM_IDs, fullState, fullCov);
			MRPT_LOG_INFO_STREAM("Mean pose: " << robotPose.mean);
			MRPT_LOG_INFO_STREAM("# of landmarks in the map: " << LMs.size());

			// Get the mean robot pose as 3D:
			const CPose3D robotPoseMean3D = CPose3D(robotPose.mean);

			// Build the path:
			meanPath.push_back(robotPoseMean3D.asTPose());

			// Save mean pose:
			if (!(step % SAVE_LOG_FREQUENCY))
			{
				const auto p = robotPose.mean.asVectorVal();
				p.saveToTextFile(
					OUT_DIR +
					format("/robot_pose_%05u.txt", (unsigned int)step));
			}

			// Save full cov:
			if (!(step % SAVE_LOG_FREQUENCY))
			{
				fullCov.saveToTextFile(
					OUT_DIR + format("/full_cov_%05u.txt", (unsigned int)step));
			}

			// Generate Data Association log?
			if (SAVE_DA_LOG)
			{
				const typename ekfslam_t::TDataAssocInfo& da =
					mapping.getLastDataAssociation();

				const CObservationBearingRange::Ptr obs =
					observations
						->getObservationByClass<CObservationBearingRange>();
				if (obs)
				{
					const CObservationBearingRange* obsRB = obs.get();
					const double tim =
						mrpt::system::timestampToDouble(obsRB->timestamp);

					for (size_t i = 0; i < obsRB->sensedData.size(); i++)
					{
						auto it = da.results.associations.find(i);
						int assoc_ID_in_SLAM;
						if (it != da.results.associations.end())
							assoc_ID_in_SLAM = it->second;
						else
						{
							// It should be a newly created LM:
							auto itNewLM = da.newly_inserted_landmarks.find(i);
							if (itNewLM != da.newly_inserted_landmarks.end())
								assoc_ID_in_SLAM = itNewLM->second;
							else
								assoc_ID_in_SLAM = -1;
						}

						out_da_log << format(
							"%35.22f %8i %10i %10f %12f %12f\n", tim, (int)i,
							assoc_ID_in_SLAM,
							(double)obsRB->sensedData[i].range,
							(double)obsRB->sensedData[i].yaw,
							(double)obsRB->sensedData[i].pitch);
					}
				}
			}

			// Save report on DA performance:
			{
				const typename ekfslam_t::TDataAssocInfo& da =
					mapping.getLastDataAssociation();

				const CObservationBearingRange::Ptr obs =
					observations
						->getObservationByClass<CObservationBearingRange>();
				if (obs)
				{
					const CObservationBearingRange* obsRB = obs.get();
					const double tim =
						mrpt::system::timestampToDouble(obsRB->timestamp);

					auto itDA = GT_DA.find(tim);

					for (size_t i = 0; i < obsRB->sensedData.size(); i++)
					{
						bool is_FP = false, is_TP = false, is_FN = false,
							 is_TN = false;

						if (itDA != GT_DA.end())
						{
							const std::vector<int>& vDA = itDA->second;
							ASSERT_BELOW_(i, vDA.size());
							const int GT_ASSOC = vDA[i];

							auto it = da.results.associations.find(i);
							if (it != da.results.associations.end())
							{
								// This observation was assigned the already
								// existing LM in the map: "it->second"
								// TruePos -> If that LM index corresponds to
								// that in the GT (with index mapping):

								// mrpt::containers::bimap<int,int>
								// DA2GTDA_indices;
								// // Landmark indices bimapping: SLAM DA <--->
								// GROUND TRUTH DA
								if (DA2GTDA_indices.hasKey(it->second))
								{
									const int slam_asigned_LM_idx =
										DA2GTDA_indices.direct(it->second);
									if (slam_asigned_LM_idx == GT_ASSOC)
										is_TP = true;
									else
										is_FP = true;
								}
								else
								{
									// Is this case possible? Assigned to an
									// index not ever seen for the first time
									// with a GT....
									//  Just in case:
									is_FP = true;
								}
							}
							else
							{
								// No pairing, but should be a newly created LM:
								auto itNewLM =
									da.newly_inserted_landmarks.find(i);
								if (itNewLM !=
									da.newly_inserted_landmarks.end())
								{
									const int new_LM_in_SLAM = itNewLM->second;

									// Was this really a NEW LM not observed
									// before?
									if (DA2GTDA_indices.hasValue(GT_ASSOC))
									{
										// GT says this LM was already observed,
										// so it shouldn't appear here as new:
										is_FN = true;
									}
									else
									{
										// Really observed for the first time:
										is_TN = true;
										DA2GTDA_indices.insert(
											new_LM_in_SLAM, GT_ASSOC);
									}
								}
								else
								{
									// Not associated neither inserted:
									// Shouldn't really never arrive here.
								}
							}
						}

						// "%           TIMESTAMP                INDEX_IN_OBS
						// TruePos FalsePos TrueNeg FalseNeg
						// NoGroundTruthSoIDontKnow \n"
						out_da_performance_log << format(
							"%35.22f %13i %8i %8i %8i %8i %8i\n", tim, (int)i,
							(int)(is_TP ? 1 : 0), (int)(is_FP ? 1 : 0),
							(int)(is_TN ? 1 : 0), (int)(is_FN ? 1 : 0),
							(int)(!is_FP && !is_TP && !is_FN && !is_TN ? 1 : 0));
					}
				}
			}

			// Save map to file representations?
			if (SAVE_MAP_REPRESENTATIONS && !(step % SAVE_LOG_FREQUENCY))
			{
				mapping.saveMapAndPath2DRepresentationAsMATLABFile(
					OUT_DIR + format("/slam_state_%05u.m", (unsigned int)step));
			}

			// Save 3D view of the filter state:
			if (win3d || (SAVE_3D_SCENES && !(step % SAVE_LOG_FREQUENCY)))
			{
				COpenGLScene::Ptr scene3D = std::make_shared<COpenGLScene>();
				{
					opengl::CGridPlaneXY::Ptr grid =
						std::make_shared<opengl::CGridPlaneXY>(
							-1000, 1000, -1000, 1000, 0, 5);
					grid->setColor(0.4f, 0.4f, 0.4f);
					scene3D->insert(grid);
				}

				// Robot path:
				{
					opengl::CSetOfLines::Ptr linesPath =
						std::make_shared<opengl::CSetOfLines>();
					linesPath->setColor(1, 0, 0);

					TPose3D init_pose;
					if (!meanPath.empty())
						init_pose = CPose3D(meanPath[0]).asTPose();

					int path_decim = 0;
					for (auto& it : meanPath)
					{
						linesPath->appendLine(init_pose, it);
						init_pose = it;

						if (++path_decim > 10)
						{
							path_decim = 0;
							mrpt::opengl::CSetOfObjects::Ptr xyz =
								mrpt::opengl::stock_objects::CornerXYZSimple(
									0.3f, 2.0f);
							xyz->setPose(CPose3D(it));
							scene3D->insert(xyz);
						}
					}
					scene3D->insert(linesPath);

					// finally a big corner for the latest robot pose:
					{
						mrpt::opengl::CSetOfObjects::Ptr xyz =
							mrpt::opengl::stock_objects::CornerXYZSimple(
								1.0, 2.5);
						xyz->setPose(robotPoseMean3D);
						scene3D->insert(xyz);
					}

					// The camera pointing to the current robot pose:
					if (win3d && CAMERA_3DSCENE_FOLLOWS_ROBOT)
					{
						win3d->setCameraPointingToPoint(
							robotPoseMean3D.x(), robotPoseMean3D.y(),
							robotPoseMean3D.z());
					}
				}

				// Do we have a ground truth?
				if (GT_PATH.cols() == 6 || GT_PATH.cols() == 3)
				{
					opengl::CSetOfLines::Ptr GT_path =
						std::make_shared<opengl::CSetOfLines>();
					GT_path->setColor(0, 0, 0);
					size_t N =
						std::min((int)GT_PATH.rows(), (int)meanPath.size());

					if (GT_PATH.cols() == 6)
					{
						double gtx0 = 0, gty0 = 0, gtz0 = 0;
						for (size_t i = 0; i < N; i++)
						{
							const CPose3D p(
								GT_PATH(i, 0), GT_PATH(i, 1), GT_PATH(i, 2),
								GT_PATH(i, 3), GT_PATH(i, 4), GT_PATH(i, 5));

							GT_path->appendLine(
								gtx0, gty0, gtz0, p.x(), p.y(), p.z());
							gtx0 = p.x();
							gty0 = p.y();
							gtz0 = p.z();
						}
					}
					else if (GT_PATH.cols() == 3)
					{
						double gtx0 = 0, gty0 = 0;
						for (size_t i = 0; i < N; i++)
						{
							const CPose2D p(
								GT_PATH(i, 0), GT_PATH(i, 1), GT_PATH(i, 2));

							GT_path->appendLine(gtx0, gty0, 0, p.x(), p.y(), 0);
							gtx0 = p.x();
							gty0 = p.y();
						}
					}
					scene3D->insert(GT_path);
				}

				// Draw latest data association:
				{
					const typename ekfslam_t::TDataAssocInfo& da =
						mapping.getLastDataAssociation();

					mrpt::opengl::CSetOfLines::Ptr lins =
						mrpt::opengl::CSetOfLines::Create();
					lins->setLineWidth(1.2f);
					lins->setColor(1, 1, 1);
					for (auto it = da.results.associations.begin();
						 it != da.results.associations.end(); ++it)
					{
						const prediction_index_t idxPred = it->second;
						// This index must match the internal list of features
						// in the map:
						typename ekfslam_t::KFArray_FEAT featMean;
						mapping.getLandmarkMean(idxPred, featMean);

						TPoint3D featMean3D;
						traits_t::landmark_to_3d(featMean, featMean3D);

						// Line: robot -> landmark:
						lins->appendLine(
							robotPoseMean3D.x(), robotPoseMean3D.y(),
							robotPoseMean3D.z(), featMean3D.x, featMean3D.y,
							featMean3D.z);
					}
					scene3D->insert(lins);
				}

				// The current state of KF-SLAM:
				{
					opengl::CSetOfObjects::Ptr objs =
						std::make_shared<opengl::CSetOfObjects>();
					mapping.getAs3DObject(objs);
					scene3D->insert(objs);
				}

				if (win3d)
				{
					mrpt::opengl::COpenGLScene::Ptr& scn =
						win3d->get3DSceneAndLock();
					scn = scene3D;

					// Update text messages:
					win3d->addTextMessage(
						0.02, 0.02,
						format(
							"Step %u - Landmarks in the map: %u",
							(unsigned int)step, (unsigned int)LMs.size()),
						TColorf(1, 1, 1), 0, MRPT_GLUT_BITMAP_HELVETICA_12);

					win3d->addTextMessage(
						0.02, 0.06,
						format(
							is_pose_3d
								? "Estimated pose: (x y z qr qx qy qz) = %s"
								: "Estimated pose: (x y yaw) = %s",
							robotPose.mean.asString().c_str()),
						TColorf(1, 1, 1), 1, MRPT_GLUT_BITMAP_HELVETICA_12);

					static vector<double> estHz_vals;
					const double curHz = 1.0 / std::max(1e-9, tim_kf_iter);
					estHz_vals.push_back(curHz);
					if (estHz_vals.size() > 50)
						estHz_vals.erase(estHz_vals.begin());
					const double meanHz = mrpt::math::mean(estHz_vals);

					win3d->addTextMessage(
						0.02, 0.10,
						format(
							"Iteration time: %7ss",
							mrpt::system::unitsFormat(tim_kf_iter).c_str()),
						TColorf(1, 1, 1), 2, MRPT_GLUT_BITMAP_HELVETICA_12);

					win3d->addTextMessage(
						0.02, 0.14,
						format(
							"Execution rate: %7sHz",
							mrpt::system::unitsFormat(meanHz).c_str()),
						TColorf(1, 1, 1), 3, MRPT_GLUT_BITMAP_HELVETICA_12);

					win3d->unlockAccess3DScene();
					win3d->repaint();
				}

				if (SAVE_3D_SCENES && !(step % SAVE_LOG_FREQUENCY))
				{
					// Save to file:
					CFileGZOutputStream f(
						OUT_DIR +
						format("/kf_state_%05u.3Dscene", (unsigned int)step));
					mrpt::serialization::archiveFrom(f) << *scene3D;
				}
			}

			// Free rawlog items memory:
			// --------------------------------------------
			action.reset();
			observations.reset();

		}  // (rawlogEntry>=rawlog_offset)

		MRPT_LOG_INFO_STREAM(
			"Step " << step << " - Rawlog entries processed: " << rawlogEntry);

		step++;
	};  // end "while(1)"

	// Partitioning experiment: Only for 6D SLAM:
	traits_t::doPartitioningExperiment(mapping, fullCov, OUT_DIR);

	// Is there ground truth of landmarks positions??
	if (ground_truth_file.size() && fileExists(ground_truth_file))
	{
		CMatrixFloat GT(0, 0);
		try
		{
			GT.loadFromTextFile(ground_truth_file);
		}
		catch (const std::exception& e)
		{
			MRPT_LOG_ERROR_STREAM(
				"Ignoring the following error loading ground truth file: "
				<< mrpt::exception_to_str(e));
		}

		if (GT.rows() > 0 && !LMs.empty())
		{
			// Each row has:
			//   [0] [1] [2]  [6]
			//    x   y   z    ID
			CVectorDouble ERRS(0);
			for (size_t i = 0; i < LMs.size(); i++)
			{
				// Find the entry in the GT for this mapped LM:
				bool found = false;
				for (int r = 0; r < GT.rows(); r++)
				{
					if (std::abs(LM_IDs[i] - GT(r, 6)) < 1e-9)
					{
						const CPoint3D gtPt(GT(r, 0), GT(r, 1), GT(r, 2));
						// All these conversions are to make it work with either
						// CPoint3D & TPoint2D:
						ERRS.push_back(
							gtPt.distanceTo(CPoint3D(TPoint3D(LMs[i]))));
						found = true;
						break;
					}
				}
				if (!found)
				{
					MRPT_LOG_ERROR_STREAM(
						"Ground truth entry not found for landmark ID:"
						<< LM_IDs[i]);
				}
			}

			loc_error_wrt_gt = math::mean(ERRS);

			MRPT_LOG_INFO("ERRORS VS. GROUND TRUTH:");
			MRPT_LOG_INFO_FMT("Mean Error: %f meters\n", loc_error_wrt_gt);
			MRPT_LOG_INFO_FMT(
				"Minimum error: %f meters\n", math::minimum(ERRS));
			MRPT_LOG_INFO_FMT(
				"Maximum error: %f meters\n", math::maximum(ERRS));
		}
	}  // end if GT

	MRPT_LOG_INFO("********* KF-SLAM finished! **********");

	if (win3d)
	{
		MRPT_LOG_WARN("Close the 3D window to quit the application.");
		win3d->waitForKey();
	}

	MRPT_END
}
