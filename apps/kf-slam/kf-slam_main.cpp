/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

/*---------------------------------------------------------------
    APPLICATION: Kalman Filter-based SLAM implementation
    FILE: kf-slam_main.cpp
    AUTHOR: Jose Luis Blanco Claraco <joseluisblancoc@gmail.com>

	See README.txt for instructions.
 ---------------------------------------------------------------*/

#include <mrpt/utils/CConfigFile.h>
#include <mrpt/utils/CFileGZInputStream.h>
#include <mrpt/utils/CFileGZOutputStream.h>
#include <mrpt/system/os.h>
#include <mrpt/system/string_utils.h>
#include <mrpt/system/filesystem.h>
#include <mrpt/slam/CRangeBearingKFSLAM.h>
#include <mrpt/slam/CRangeBearingKFSLAM2D.h>
#include <mrpt/obs/CRawlog.h>
#include <mrpt/math/ops_containers.h>
#include <mrpt/gui/CDisplayWindow3D.h>
#include <mrpt/opengl/CGridPlaneXY.h>
#include <mrpt/opengl/CSetOfLines.h>
#include <mrpt/opengl/stock_objects.h>

using namespace mrpt;
using namespace mrpt::slam;
using namespace mrpt::maps;
using namespace mrpt::opengl;
using namespace mrpt::system;
using namespace mrpt::math;
using namespace mrpt::poses;
using namespace mrpt::utils;
using namespace mrpt::obs;
using namespace std;

// Fordward declaration.
template <class IMPL>
void Run_KF_SLAM( CConfigFile &cfgFile, const std::string &rawlogFileName );


// 3D Window (smart pointer) declared here so after an exception it's not
// immediately closed.
mrpt::gui::CDisplayWindow3DPtr win3d;


// ------------------------------------------------------
//						MAIN
// ------------------------------------------------------
int main(int argc, char **argv)
{
	try
	{
		printf(" kf-slam - Part of the MRPT\n");
		printf(" MRPT C++ Library: %s - Sources timestamp: %s\n", MRPT_getVersion().c_str(), MRPT_getCompilationDate().c_str());
		printf("-------------------------------------------------------------------\n");

		// Process arguments:
		if (argc<2)
		{
			printf("Use: kf-slam <config_file> [dataset.rawlog]\n\nPush any key to exit...\n");
			os::getch();
			return -1;
		}

		// Config file:
		const std::string configFile = std::string( argv[1] );
		CConfigFile cfg( configFile );

		// Rawlog file: from args. line or from config file:
		string rawlogFileName;
		if (argc==3)
			rawlogFileName = std::string( argv[2] );
		else
			rawlogFileName = cfg.read_string("MappingApplication","rawlog_file",std::string("log.rawlog"));

		// 2D or 3D implementation:
		const string kf_implementation =
			mrpt::system::trim( cfg.read_string("MappingApplication","kf_implementation","CRangeBearingKFSLAM" ) );

		if (kf_implementation=="CRangeBearingKFSLAM")
			Run_KF_SLAM<CRangeBearingKFSLAM>(cfg,rawlogFileName);
		else
		if (kf_implementation=="CRangeBearingKFSLAM2D")
			Run_KF_SLAM<CRangeBearingKFSLAM2D>(cfg,rawlogFileName);
		else
			throw std::runtime_error("kf_implementation: Invalid value found in the config file.");

		return 0;
	}
	catch (std::exception &e)
	{
		std::cerr << e.what() << std::endl << "Program finished for an exception!!" << std::endl;
		mrpt::system::pause();
		return -1;
	}
	catch (...)
	{
		std::cerr << "Untyped exception!!" << std::endl;
		mrpt::system::pause();
		return -1;
	}
}


// ------------------------------------------------------
//				traits
// ------------------------------------------------------
template <class IMPL> struct kfslam_traits;

// Specialization for 2D (3D) SLAM:
template <> struct kfslam_traits<CRangeBearingKFSLAM2D>
{
	typedef CRangeBearingKFSLAM2D     ekfslam_t;
	typedef CPosePDFGaussian	      posepdf_t;
	typedef CPose2D                   pose_t;
	typedef TPoint2D                  lm_t;
	template <class ARR> static void landmark_to_3d(const ARR &lm, TPoint3D &p) { p.x=lm[0]; p.y=lm[1]; p.z=0; }

	static void doPartitioningExperiment(
		ekfslam_t  &mapping,
		CMatrixDouble  &fullCov,
		const string &OUT_DIR)
	{
		// Nothing to do
	}
};

// Specialization for 3D (6D) SLAM:
template <> struct kfslam_traits<CRangeBearingKFSLAM>
{
	typedef CRangeBearingKFSLAM       ekfslam_t;
	typedef CPose3DQuatPDFGaussian	  posepdf_t;
	typedef CPose3D                   pose_t;
	typedef CPoint3D                  lm_t;

	template <class ARR> static void landmark_to_3d(const ARR &lm, TPoint3D &p) { p.x=lm[0]; p.y=lm[1]; p.z=lm[2]; }

	static void doPartitioningExperiment(
		ekfslam_t  &mapping,
		CMatrixDouble  &fullCov,
		const string &OUT_DIR)
	{
		// Compute the "information" between partitions:
		if (mapping.options.doPartitioningExperiment)
		{
			// --------------------------------------------
			// PART I:
			//  Comparison to fixed partitioning every K obs.
			// --------------------------------------------

			// Compute the information matrix:
			for (size_t i=0;i<6;i++) fullCov(i,i) = max(fullCov(i,i), 1e-6);

			CMatrix		H( fullCov.inv() );
			H.saveToTextFile(OUT_DIR+string("/information_matrix_final.txt"));

			// Replace by absolute values:
			H = H.array().abs().matrix();
			CMatrix H2(H); H2.normalize(0,1);
			CImage   imgF(H2, true);
			imgF.saveToFile(OUT_DIR+string("/information_matrix_final.png"));


			// ----------------------------------------
			// Compute the "approximation error factor" E:
			//  E = SUM() / SUM(ALL ELEMENTS IN MATRIX)
			// ----------------------------------------
			vector<vector_uint>  landmarksMembership,partsInObsSpace;
			CMatrix  ERRS(50,3);

			for (size_t i=0;i<ERRS.getRowCount();i++)
			{
				size_t K;

				if (i==0)
				{
					K=0;
					mapping.getLastPartitionLandmarks( landmarksMembership );
				}
				else
				{
					K=i+1;
					mapping.getLastPartitionLandmarksAsIfFixedSubmaps(i+1,landmarksMembership);
				}

				mapping.getLastPartition(partsInObsSpace);

				ERRS(i,0) = (float)K;
				ERRS(i,1) = (float)partsInObsSpace.size();
				ERRS(i,2) = mapping.computeOffDiagonalBlocksApproximationError(landmarksMembership);
			}


			ERRS.saveToTextFile( OUT_DIR+string("/ERRORS.txt" ));
			//printf("Approximation error from partition:\n"); cout << ERRS << endl;

			// --------------------------------------------
			// PART II:
			//  Sweep partitioning threshold:
			// --------------------------------------------
			size_t STEPS = 50;
			CVectorFloat	ERRS_SWEEP(STEPS),ERRS_SWEEP_THRESHOLD(STEPS);

			// Compute the error for each partitioning-threshold
			for (size_t i=0;i<STEPS;i++)
			{
				float th = (1.0f*i)/(STEPS-1.0f);
				ERRS_SWEEP_THRESHOLD[i] = th;
				mapping.mapPartitionOptions()->partitionThreshold =  th;

				mapping.reconsiderPartitionsNow();

				mapping.getLastPartitionLandmarks( landmarksMembership );
				ERRS_SWEEP[i] = mapping.computeOffDiagonalBlocksApproximationError(landmarksMembership);
			}

			ERRS_SWEEP.saveToTextFile( OUT_DIR+string("/ERRORS_SWEEP.txt" ));
			ERRS_SWEEP_THRESHOLD.saveToTextFile( OUT_DIR+string("/ERRORS_SWEEP_THRESHOLD.txt" ));

		} // end if doPartitioningExperiment
	}

};


// ------------------------------------------------------
//				Run_KF_SLAM
// ------------------------------------------------------
template <class IMPL>
void Run_KF_SLAM( CConfigFile &cfgFile, const std::string &rawlogFileName )
{
	// The EKF-SLAM class:
	typedef kfslam_traits<IMPL> traits_t; // Traits for this KF implementation (2D or 3D)

	typedef typename traits_t::ekfslam_t ekfslam_t;

	ekfslam_t  mapping;

	// The rawlog file:
	// ----------------------------------------
	const unsigned int	rawlog_offset = cfgFile.read_int("MappingApplication","rawlog_offset",0);

	const unsigned int SAVE_LOG_FREQUENCY= cfgFile.read_int("MappingApplication","SAVE_LOG_FREQUENCY",1);

	const bool  SAVE_DA_LOG = cfgFile.read_bool("MappingApplication","SAVE_DA_LOG", true);

	const bool  SAVE_3D_SCENES = cfgFile.read_bool("MappingApplication","SAVE_3D_SCENES", true);
	const bool  SAVE_MAP_REPRESENTATIONS = cfgFile.read_bool("MappingApplication","SAVE_MAP_REPRESENTATIONS", true);
	bool  SHOW_3D_LIVE = cfgFile.read_bool("MappingApplication","SHOW_3D_LIVE", false);
	const bool  CAMERA_3DSCENE_FOLLOWS_ROBOT = cfgFile.read_bool("MappingApplication","CAMERA_3DSCENE_FOLLOWS_ROBOT", false);

#if !MRPT_HAS_WXWIDGETS
	SHOW_3D_LIVE = false;
#endif


	string OUT_DIR = cfgFile.read_string("MappingApplication","logOutput_dir","OUT_KF-SLAM");
	string ground_truth_file = cfgFile.read_string("MappingApplication","ground_truth_file","");
	string ground_truth_file_robot= cfgFile.read_string("MappingApplication","ground_truth_file_robot","");

	string ground_truth_data_association = cfgFile.read_string("MappingApplication","ground_truth_data_association","");

	cout << "RAWLOG FILE:" << endl << rawlogFileName << endl;
	ASSERT_FILE_EXISTS_( rawlogFileName )
	CFileGZInputStream	rawlogFile( rawlogFileName );


	cout << "---------------------------------------------------" << endl << endl;

	deleteFilesInDirectory(OUT_DIR);
	createDirectory(OUT_DIR);

	// Load the config options for mapping:
	// ----------------------------------------
	mapping.loadOptions( cfgFile );
	mapping.KF_options.dumpToConsole();
	mapping.options.dumpToConsole();

	// debug:
	//mapping.KF_options.use_analytic_observation_jacobian = true;
	//mapping.KF_options.use_analytic_transition_jacobian = true;
	//mapping.KF_options.debug_verify_analytic_jacobians = true;


    // Is there ground truth of the robot poses??
	CMatrixDouble GT_PATH(0,0);
	if (ground_truth_file_robot.size() && fileExists(ground_truth_file_robot))
	{
		GT_PATH.loadFromTextFile(ground_truth_file_robot);
		ASSERT_(size(GT_PATH,1)>0 && size(GT_PATH,2)==6)
	}

	// Is there a ground truth file of the data association?
	std::map<double,std::vector<int> >  GT_DA; // Map: timestamp -> vector(index in observation -> real index)
	mrpt::utils::bimap<int,int>             DA2GTDA_indices; // Landmark indices bimapping: SLAM DA <---> GROUND TRUTH DA
	if (!ground_truth_data_association.empty() && fileExists(ground_truth_data_association))
	{
		CMatrixDouble mGT_DA;
		mGT_DA.loadFromTextFile(ground_truth_data_association);
		ASSERT_ABOVEEQ_(mGT_DA.getColCount(),3)

		// Convert the loaded matrix into a std::map in GT_DA:
		for (size_t i=0;i<mGT_DA.getRowCount();i++)
		{
			std::vector<int> & v = GT_DA[mGT_DA(i,0)];
			if (v.size()<=mGT_DA(i,1)) v.resize(mGT_DA(i,1)+1);
			v[mGT_DA(i,1)] = mGT_DA(i,2);
		}
		cout << "Loaded " << GT_DA.size() << " entries from DA ground truth file\n";
	}

	// Create output file for DA perf:
	std::ofstream   out_da_performance_log;
	{
		const std::string f = std::string(OUT_DIR+std::string("/data_association_performance.log"));
		out_da_performance_log.open( f.c_str() );
		ASSERTMSG_(out_da_performance_log.is_open(), std::string("Error writing to: ") + f )

		// Header:
		out_da_performance_log
			<< "%           TIMESTAMP                INDEX_IN_OBS    TruePos FalsePos TrueNeg FalseNeg  NoGroundTruthSoIDontKnow \n"
			<< "%----------------------------------------------------------------------------------------------------------------\n";
	}

	if (SHOW_3D_LIVE)
	{
		win3d = mrpt::gui::CDisplayWindow3D::Create("KF-SLAM live view",800,500);

		win3d->addTextMessage(0.01,0.96,"Red: Estimated path",TColorf(0.8f,0.8f,0.8f),100,MRPT_GLUT_BITMAP_HELVETICA_10);
		win3d->addTextMessage(0.01,0.93,"Black: Ground truth path",TColorf(0.8f,0.8f,0.8f),101,MRPT_GLUT_BITMAP_HELVETICA_10);
	}

	// Create DA-log output file:
	std::ofstream   out_da_log;
	if (SAVE_DA_LOG)
	{
		const std::string f = std::string(OUT_DIR+std::string("/data_association.log"));
		out_da_log.open( f.c_str() );
		ASSERTMSG_(out_da_log.is_open(), std::string("Error writing to: ") + f )

		// Header:
		out_da_log << "%           TIMESTAMP                INDEX_IN_OBS    ID    RANGE(m)    YAW(rad)   PITCH(rad) \n"
			       << "%--------------------------------------------------------------------------------------------\n";

	}


	// The main loop:
	// ---------------------------------------
	CActionCollectionPtr	action;
	CSensoryFramePtr		observations;
	size_t rawlogEntry = 0, step = 0;


	vector<TPose3D>  meanPath; // The estimated path
	typename traits_t::posepdf_t   robotPose;
	const bool is_pose_3d = robotPose.state_length != 3;

	std::vector<typename IMPL::landmark_point_t>	 LMs;
	std::map<unsigned int,CLandmark::TLandmarkID>    LM_IDs;
	CMatrixDouble  fullCov;
	CVectorDouble  fullState;
	CTicTac kftictac;

	for (;;)
	{
		if (os::kbhit())
        {
			char	pushKey = os::getch();
			if (27 == pushKey)
				break;
        }


		// Load action/observation pair from the rawlog:
		// --------------------------------------------------
		if (! CRawlog::readActionObservationPair( rawlogFile, action, observations, rawlogEntry) )
			break; // file EOF

		if (rawlogEntry>=rawlog_offset)
		{
			// Process the action and observations:
			// --------------------------------------------
			kftictac.Tic();

			mapping.processActionObservation(action,observations);

			const double tim_kf_iter = kftictac.Tac();

			// Get current state:
			// -------------------------------
			mapping.getCurrentState( robotPose,LMs,LM_IDs,fullState,fullCov );
			cout << "Mean pose: " << endl << robotPose.mean << endl;
			cout << "# of landmarks in the map: " << LMs.size() << endl;

			// Get the mean robot pose as 3D:
			const CPose3D robotPoseMean3D = CPose3D(robotPose.mean);

			// Build the path:
			meanPath.push_back( TPose3D(robotPoseMean3D) );

			// Save mean pose:
			if (!(step % SAVE_LOG_FREQUENCY))
			{
				const CVectorDouble p = robotPose.mean.getAsVectorVal();
				p.saveToTextFile(OUT_DIR+format("/robot_pose_%05u.txt",(unsigned int)step));
			}

			// Save full cov:
			if (!(step % SAVE_LOG_FREQUENCY))
			{
				fullCov.saveToTextFile(OUT_DIR+format("/full_cov_%05u.txt",(unsigned int)step));
			}

			// Generate Data Association log?
			if (SAVE_DA_LOG)
			{
				const typename ekfslam_t::TDataAssocInfo & da = mapping.getLastDataAssociation();

				const CObservationBearingRangePtr obs = observations->getObservationByClass<CObservationBearingRange>();
				if (obs)
				{
					const CObservationBearingRange* obsRB = obs.pointer();
					const double tim = mrpt::system::timestampToDouble( obsRB->timestamp );

					for (size_t i=0;i<obsRB->sensedData.size();i++)
					{
						std::map<observation_index_t,prediction_index_t>::const_iterator it = da.results.associations.find(i);
						int assoc_ID_in_SLAM;
						if (it!=da.results.associations.end())
								assoc_ID_in_SLAM = it->second;
						else
						{
							// It should be a newly created LM:
							std::map<size_t,size_t>::const_iterator itNewLM = da.newly_inserted_landmarks.find(i);
							if (itNewLM!=da.newly_inserted_landmarks.end())
							      assoc_ID_in_SLAM = itNewLM->second;
							else  assoc_ID_in_SLAM = -1;
						}

						out_da_log << format("%35.22f %8i %10i %10f %12f %12f\n",
							tim,
							(int)i,
							assoc_ID_in_SLAM,
							(double)obsRB->sensedData[i].range,
							(double)obsRB->sensedData[i].yaw,
							(double)obsRB->sensedData[i].pitch );
					}
				}
			}


			// Save report on DA performance:
			{
				const typename ekfslam_t::TDataAssocInfo & da = mapping.getLastDataAssociation();

				const CObservationBearingRangePtr obs = observations->getObservationByClass<CObservationBearingRange>();
				if (obs)
				{
					const CObservationBearingRange* obsRB = obs.pointer();
					const double tim = mrpt::system::timestampToDouble( obsRB->timestamp );

					std::map<double,std::vector<int> >::const_iterator itDA = GT_DA.find( tim );

					for (size_t i=0;i<obsRB->sensedData.size();i++)
					{
						bool is_FP=false, is_TP=false, is_FN=false, is_TN = false;

						if (itDA!=GT_DA.end())
						{
							const std::vector<int> & vDA = itDA->second;
							ASSERT_BELOW_(i,vDA.size())
							const int GT_ASSOC = vDA[i];

							std::map<observation_index_t,prediction_index_t>::const_iterator it = da.results.associations.find(i);
							if (it!=da.results.associations.end())
							{
								// This observation was assigned the already existing LM in the map: "it->second"
								// TruePos -> If that LM index corresponds to that in the GT (with index mapping):

								//mrpt::utils::bimap<int,int>  DA2GTDA_indices; // Landmark indices bimapping: SLAM DA <---> GROUND TRUTH DA
								if (DA2GTDA_indices.hasKey(it->second))
								{
									const int slam_asigned_LM_idx = DA2GTDA_indices.direct(it->second);
									if (slam_asigned_LM_idx==GT_ASSOC)
									      is_TP = true;
									else  is_FP = true;
								}
								else
								{
									// Is this case possible? Assigned to an index not ever seen for the first time with a GT....
									//  Just in case:
									is_FP = true;
								}
							}
							else
							{
								// No pairing, but should be a newly created LM:
								std::map<size_t,size_t>::const_iterator itNewLM = da.newly_inserted_landmarks.find(i);
								if (itNewLM!=da.newly_inserted_landmarks.end())
								{
									const int new_LM_in_SLAM = itNewLM->second;

									// Was this really a NEW LM not observed before?
									if (DA2GTDA_indices.hasValue(GT_ASSOC))
									{
										// GT says this LM was already observed, so it shouldn't appear here as new:
										is_FN = true;
									}
									else
									{
										// Really observed for the first time:
										is_TN = true;
										DA2GTDA_indices.insert(new_LM_in_SLAM,GT_ASSOC);
									}
								}
								else
								{
									// Not associated neither inserted: Shouldn't really never arrive here.
								}
							}
						}

						// "%           TIMESTAMP                INDEX_IN_OBS    TruePos FalsePos TrueNeg FalseNeg  NoGroundTruthSoIDontKnow \n"
						out_da_performance_log << format("%35.22f %13i %8i %8i %8i %8i %8i\n",
							tim,
							(int)i,
							(int)(is_TP ? 1:0),
							(int)(is_FP ? 1:0),
							(int)(is_TN ? 1:0),
							(int)(is_FN ? 1:0),
							(int)(!is_FP && !is_TP && !is_FN && !is_TN ? 1:0)
							);
					}
				}

			}



			// Save map to file representations?
			if (SAVE_MAP_REPRESENTATIONS  && !(step % SAVE_LOG_FREQUENCY))
			{
				mapping.saveMapAndPath2DRepresentationAsMATLABFile( OUT_DIR+format("/slam_state_%05u.m",(unsigned int)step) );
			}

			// Save 3D view of the filter state:
			if (win3d.present() || ( SAVE_3D_SCENES && !(step % SAVE_LOG_FREQUENCY) ) )
			{
				COpenGLScenePtr   scene3D = COpenGLScene::Create();
				{
					opengl::CGridPlaneXYPtr grid = opengl::CGridPlaneXY::Create(-1000,1000,-1000,1000,0,5);
					grid->setColor(0.4,0.4,0.4);
					scene3D->insert( grid );
				}

				// Robot path:
				{
					opengl::CSetOfLinesPtr linesPath = opengl::CSetOfLines::Create();
					linesPath->setColor(1,0,0);

					TPose3D init_pose;
					if (!meanPath.empty())
						init_pose = TPose3D(CPose3D(meanPath[0]));

					int path_decim = 0;
					for (vector<TPose3D>::iterator it=meanPath.begin();it!=meanPath.end();++it)
					{
						linesPath->appendLine(init_pose,*it);
						init_pose = *it;

						if (++path_decim>10)
						{
							path_decim = 0;
							mrpt::opengl::CSetOfObjectsPtr xyz = mrpt::opengl::stock_objects::CornerXYZSimple(0.3f,2.0f);
							xyz->setPose(CPose3D(*it));
							scene3D->insert(xyz);
						}
					}
					scene3D->insert( linesPath );

					// finally a big corner for the latest robot pose:
					{
						mrpt::opengl::CSetOfObjectsPtr xyz = mrpt::opengl::stock_objects::CornerXYZSimple(1.0,2.5);
						xyz->setPose(robotPoseMean3D);
						scene3D->insert(xyz);
					}

					// The camera pointing to the current robot pose:
					if (CAMERA_3DSCENE_FOLLOWS_ROBOT)
					{
						win3d->setCameraPointingToPoint(robotPoseMean3D.x(),robotPoseMean3D.y(),robotPoseMean3D.z());
					}
				}

				// Do we have a ground truth?
				if (size(GT_PATH,2)==6 || size(GT_PATH,2)==3)
				{
					opengl::CSetOfLinesPtr GT_path = opengl::CSetOfLines::Create();
					GT_path->setColor(0,0,0);
					size_t N = std::min(size(GT_PATH,1), meanPath.size() );

					if (size(GT_PATH,2)==6)
					{
						double gtx0=0,gty0=0,gtz0=0;
						for (size_t i=0;i<N;i++)
						{
							const CPose3D  p( GT_PATH(i,0),GT_PATH(i,1),GT_PATH(i,2), GT_PATH(i,3),GT_PATH(i,4),GT_PATH(i,5) );

							GT_path->appendLine(
								gtx0,gty0,gtz0,
								p.x(),p.y(),p.z() );
							gtx0=p.x();
							gty0=p.y();
							gtz0=p.z();
						}
					}
					else if (size(GT_PATH,2)==3)
					{
						double gtx0=0,gty0=0;
						for (size_t i=0;i<N;i++)
						{
							const CPose2D p( GT_PATH(i,0),GT_PATH(i,1),GT_PATH(i,2) );

							GT_path->appendLine(
								gtx0,gty0,0,
								p.x(),p.y(),0 );
							gtx0=p.x();
							gty0=p.y();
						}
					}
					scene3D->insert( GT_path );
				}

				// Draw latest data association:
				{
					const typename ekfslam_t::TDataAssocInfo & da = mapping.getLastDataAssociation();

					mrpt::opengl::CSetOfLinesPtr lins = mrpt::opengl::CSetOfLines::Create();
					lins->setLineWidth(1.2f);
					lins->setColor(1,1,1);
					for (std::map<observation_index_t,prediction_index_t>::const_iterator it=da.results.associations.begin();it!=da.results.associations.end();++it)
					{
						const prediction_index_t idxPred = it->second;
						// This index must match the internal list of features in the map:
						typename ekfslam_t::KFArray_FEAT featMean;
						mapping.getLandmarkMean(idxPred, featMean);

						TPoint3D featMean3D;
						traits_t::landmark_to_3d(featMean,featMean3D);

						// Line: robot -> landmark:
						lins->appendLine(
							robotPoseMean3D.x(),robotPoseMean3D.y(),robotPoseMean3D.z(),
							featMean3D.x,featMean3D.y,featMean3D.z);
					}
					scene3D->insert( lins );
				}

				// The current state of KF-SLAM:
				{
					opengl::CSetOfObjectsPtr  objs = opengl::CSetOfObjects::Create();
					mapping.getAs3DObject(objs);
					scene3D->insert( objs );
				}

				if (win3d.present())
				{
					mrpt::opengl::COpenGLScenePtr &scn = win3d->get3DSceneAndLock();
					scn = scene3D;

					// Update text messages:
					win3d->addTextMessage(
						0.02,0.02,
						format("Step %u - Landmarks in the map: %u",(unsigned int)step, (unsigned int)LMs.size() ),
						TColorf(1,1,1), 0, MRPT_GLUT_BITMAP_HELVETICA_12 );

					win3d->addTextMessage(
						0.02,0.06,
						format(is_pose_3d ? 
							"Estimated pose: (x y z qr qx qy qz) = %s"
							:
							"Estimated pose: (x y yaw) = %s"
							, robotPose.mean.asString().c_str() ),
						TColorf(1,1,1), 1, MRPT_GLUT_BITMAP_HELVETICA_12 );

					static vector<double> estHz_vals;
					const double curHz = 1.0/std::max(1e-9,tim_kf_iter);
					estHz_vals.push_back(curHz);
					if (estHz_vals.size()>50)
						estHz_vals.erase(estHz_vals.begin());
					const double meanHz = mrpt::math::mean(estHz_vals);


					win3d->addTextMessage(
						0.02,0.10,
						format("Iteration time: %7ss",
							mrpt::system::unitsFormat(tim_kf_iter).c_str()),
							TColorf(1,1,1), 2, MRPT_GLUT_BITMAP_HELVETICA_12 );

					win3d->addTextMessage(
						0.02,0.14,
						format("Execution rate: %7sHz",
							mrpt::system::unitsFormat(meanHz).c_str()),
							TColorf(1,1,1), 3, MRPT_GLUT_BITMAP_HELVETICA_12 );

					win3d->unlockAccess3DScene();
					win3d->repaint();
				}

				if ( SAVE_3D_SCENES && !(step % SAVE_LOG_FREQUENCY) )
				{
					// Save to file:
					CFileGZOutputStream(OUT_DIR+format("/kf_state_%05u.3Dscene",(unsigned int)step)) << *scene3D;
				}
			}


			// Free rawlog items memory:
			// --------------------------------------------
			action.clear_unique();
			observations.clear_unique();

		} // (rawlogEntry>=rawlog_offset)

		cout << format("\nStep %u  - Rawlog entries processed: %i\n", (unsigned int)step, (unsigned int)rawlogEntry);

        step++;
	};	// end "while(1)"


	// Partitioning experiment: Only for 6D SLAM:
	traits_t::doPartitioningExperiment( mapping, fullCov, OUT_DIR );

    // Is there ground truth of landmarks positions??
    if (ground_truth_file.size() && fileExists(ground_truth_file))
    {
        CMatrixFloat    GT(0,0);
        try
        {
            GT.loadFromTextFile(ground_truth_file);
        }
        catch(std::exception &e)
        {
            cerr << "Ignoring the following error loading ground truth file: " << e.what() << endl;
        }

        if (GT.getRowCount()>0 && !LMs.empty())
        {
            // Each row has:
            //   [0] [1] [2]  [6]
            //    x   y   z    ID
            CVectorDouble ERRS(0);
            for (size_t i=0;i<LMs.size();i++)
            {
                // Find the entry in the GT for this mapped LM:
                bool found = false;
                for (size_t r=0;r<GT.getRowCount();r++)
                {
                    if ( LM_IDs[i] == GT(r,6) )
                    {
                    	const CPoint3D  gtPt( GT(r,0),GT(r,1),GT(r,2) );
                        ERRS.push_back( gtPt.distanceTo(CPoint3D(TPoint3D(LMs[i]))) );  // All these conversions are to make it work with either CPoint3D & TPoint2D
                        found = true;
                        break;
                    }
                }
                if (!found)
                {
                    cerr << "Ground truth entry not found for landmark ID:" << LM_IDs[i] << endl;
                }
            }

            printf("ERRORS VS. GROUND TRUTH:\n");
            printf("Mean Error: %f meters\n", math::mean(ERRS) );
            printf("Minimum error: %f meters\n", math::minimum(ERRS) );
            printf("Maximum error: %f meters\n", math::maximum(ERRS) );
        }
    } // end if GT

	cout << "********* KF-SLAM finished! **********" << endl;

	if (win3d)
	{
		cout << "\n Close the 3D window to quit the application.\n";
		win3d->waitForKey();
	}

}

