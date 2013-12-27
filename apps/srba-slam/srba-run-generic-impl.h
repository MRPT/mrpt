/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2014, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#pragma once

#include "srba-slam_common.h"

#include <mrpt/system/filesystem.h>  // for ASSERT_FILE_EXISTS_
#include <mrpt/system/threads.h>  // for sleep()
#include <mrpt/random.h>
#include <mrpt/utils/CFileOutputStream.h>  // For CMatrixDouble
//#include <mrpt/math/CMatrixTemplateNumeric.h>  // For CMatrixDouble
//#include <mrpt/math/CMatrixD.h>  // For the serializable version of matrices
#include <mrpt/vision/CVideoFileWriter.h>

#include <mrpt/gui/CDisplayWindow3D.h>

// We can use "using namespace" in this header since it's designed to be only included in this app, not in user code.
using namespace std;
using namespace mrpt;
using namespace mrpt::srba;
using namespace mrpt::utils;


template <class KF2KF_POSE_TYPE,class LM_TYPE,class OBS_TYPE>
RBA_Run_BasePtr RBA_Run_Factory<KF2KF_POSE_TYPE,LM_TYPE,OBS_TYPE>::create()
{
	return RBA_Run_BasePtr(
		new RBA_Run<
			KF2KF_POSE_TYPE,
			LM_TYPE,
			OBS_TYPE,
			typename problem_options_traits_t<KF2KF_POSE_TYPE,LM_TYPE,OBS_TYPE>::srba_options_t
			>() );
}


// ---------------- INCLUDE DATASET PARSERS -------------------------
#include "CDatasetParserBase.h"
// ------------------------------------------------------------------

// -------------------- InitializerSensorParams -------------------------
template <class OBS_TYPE>
struct InitializerSensorParams
{
	template <class RBA>
	static void init(RBA &rba, RBASLAM_Params &config)
	{
		// Nothing to do by default:
	}
};

// -------------------- InitializerSensorPoseParams  -------------------------
template <class SENSOR_POSE_OPTION>
struct InitializerSensorPoseParams;

template <>
struct InitializerSensorPoseParams<options::sensor_pose_on_robot_none>
{
	template <class RBA>
	static void init(RBA &rba, RBASLAM_Params &config)
	{
		// Nothing to do.
	}
};

template <>
struct InitializerSensorPoseParams<options::sensor_pose_on_robot_se3>
{
	template <class RBA>
	static void init(RBA &rba, RBASLAM_Params &config)
	{
		// Sensor pose on the robot parameters:
		rba.parameters.sensor_pose.relative_pose = mrpt::poses::CPose3D(0,0,0,DEG2RAD(-90),DEG2RAD(0),DEG2RAD(-90) );
	}
};


template <class KF2KF_POSE_TYPE,class LM_TYPE,class OBS_TYPE, class RBA_OPTIONS>
struct RBA_Run : public RBA_Run_Base
{
	typedef RbaEngine<
		KF2KF_POSE_TYPE, // Parameterization  KF-to-KF poses
		LM_TYPE,         // Parameterization of landmark positions
		OBS_TYPE,        // Type of observations
		RBA_OPTIONS      // Other parameters
		>
		my_srba_t;

	// Constructor:
	virtual int run(RBASLAM_Params &cfg)
	{
		// Open & load dataset:
		// ------------------------------------------
		CDatasetParserTempl<OBS_TYPE>  dataset(cfg);


		// Create an empty RBA problem:
		// ------------------------------------------
		my_srba_t rba;

		// Init/load sensor parameters:
		InitializerSensorParams<OBS_TYPE>::init(rba,cfg);

		// Init/load sensor noise params:
		dataset.loadNoiseParamsInto( rba.parameters.obs_noise );


		// Init sensor-to-robot relative pose parameters:
		InitializerSensorPoseParams<typename RBA_OPTIONS::sensor_pose_on_robot_t>::init(rba, cfg);

		// Process cmd-line flags:
		// ------------------------------------------
		if (cfg.arg_random_seed.getValue()<0)
				mrpt::random::randomGenerator.randomize();
		else 	mrpt::random::randomGenerator.randomize( cfg.arg_random_seed.getValue() );

		const bool DEBUG_DUMP_ALL_SPANNING_TREES =cfg.arg_debug_dump_cur_spantree.getValue();

		rba.setVerbosityLevel(cfg.arg_verbose.getValue() );

		rba.parameters.srba.use_robust_kernel = false;

		rba.parameters.srba.compute_condition_number = false;

		rba.parameters.srba.max_error_per_obs_to_stop = 1e-8;
		//rba.parameters.srba.feedback_user_iteration = &optimization_feedback;

		if (cfg.arg_rba_params_cfg_file.isSet() && mrpt::system::fileExists(cfg.arg_rba_params_cfg_file.getValue()))
			rba.parameters.srba.loadFromConfigFileName(cfg.arg_rba_params_cfg_file.getValue(), "srba");

		if (cfg.arg_write_rba_params_cfg_file.isSet())
		{
			rba.parameters.srba.saveToConfigFileName(cfg.arg_write_rba_params_cfg_file.getValue(), "srba");
			return 0; // end program
		}

		// Override max ST depth:
		if (cfg.arg_max_tree_depth.isSet())
		{
			rba.parameters.srba.max_tree_depth =cfg.arg_max_tree_depth.getValue();
			cout << "Overriding max_tree_depth to value: " << rba.parameters.srba.max_tree_depth << endl;
		}

		// Override policy:
		if (cfg.arg_edge_policy.isSet())
		{
			rba.parameters.srba.edge_creation_policy = mrpt::utils::TEnumType<srba::TEdgeCreationPolicy>::name2value(cfg.arg_edge_policy.getValue());
			cout << "Overriding edge_creation_policy to value: " <<cfg.arg_edge_policy.getValue() << endl;
		}

		// Override max optimize depth:
		if (cfg.arg_max_opt_depth.isSet())
		{
			rba.parameters.srba.max_optimize_depth =cfg.arg_max_opt_depth.getValue();
			cout << "Overriding max_optimize_depth to value: " << rba.parameters.srba.max_optimize_depth << endl;
		}

		// Override max lambda:
		if (cfg.arg_max_lambda.isSet())
		{
			rba.parameters.srba.max_lambda =cfg.arg_max_lambda.getValue();
			cout << "Overriding max_lambda to value: " << rba.parameters.srba.max_lambda << endl;
		}

		// Override max optimize depth:
		if (cfg.arg_max_iters.isSet())
		{
			rba.parameters.srba.max_iters =cfg.arg_max_iters.getValue();
			cout << "Overriding max_iters to value: " << rba.parameters.srba.max_iters << endl;
		}

		// Override submap
		if (cfg.arg_submap_size.isSet())
		{
			rba.parameters.srba.submap_size =cfg.arg_submap_size.getValue();
			cout << "Overriding submap_size to value: " << rba.parameters.srba.submap_size << endl;
		}

		if (cfg.arg_graph_slam.isSet())
		{
			rba.parameters.srba.min_obs_to_loop_closure = 1;
			rba.parameters.srba.optimize_new_edges_alone = true;
		}

		if (cfg.arg_verbose.getValue()>=1)
		{
			cout << "RBA parameters:\n"
					"-----------------\n";
			rba.parameters.srba.dumpToConsole();
			cout << endl;
		}

		const unsigned int	INCREMENTAL_FRAMES_AT_ONCE  = 1;
		//const unsigned int	MAX_KNOWN_FEATS_PER_FRAME   =cfg.arg_max_known_feats_per_frame.getValue();

		//const double REL_POS_NOISE_STD_KNOWN    = 0.0001;  // m
		//const double REL_POS_NOISE_STD_UNKNOWN  = 0.3;  // m

		const bool DEMO_SIMUL_SHOW_GUI = !cfg.arg_no_gui.getValue();
		bool DEMO_SIMUL_SHOW_STEPBYSTEP =cfg.arg_gui_step_by_step.getValue();

		const bool   SAVE_TIMING_STATS =cfg.arg_profile_stats.isSet();
		const size_t SAVE_TIMING_SEGMENT_LENGTH =cfg.arg_profile_stats_length.getValue();

		// Load dataset:
		const size_t nTotalObs = dataset.obs().getRowCount();

		// ------------------------------------------------------------------------
		// Process the dataset sequentially, add observations to the RBA problem
		// and solve it:
		// ------------------------------------------------------------------------

		mrpt::gui::CDisplayWindow3DPtr win;
		mrpt::opengl::COpenGLViewportPtr gl_view_cur_tree, gl_view_GT;
		mrpt::opengl::CSetOfObjectsPtr gl_rba_scene, gl_cur_kf_tree, gl_gt_pose;
		mrpt::opengl::CPointCloudPtr  gl_gt_map;
		mrpt::opengl::CSetOfLinesPtr  gl_gt_path_track;

		// Arbitrary unique IDs for text labels in the 3D gui.
		const size_t TXTID_KF   = 1000;
		const size_t TXTID_RMSE = 1001;
		const size_t TXTID_LABEL_ST = 1002;
		const size_t TXTID_LABEL_GT = 1003;

		if (DEMO_SIMUL_SHOW_GUI)
		{
			win = mrpt::gui::CDisplayWindow3D::Create("Simulation state",1000,800);
			win->setCameraPointingToPoint(8,8,0);

			{
				mrpt::opengl::COpenGLScenePtr scene = win->get3DSceneAndLock();

				gl_rba_scene   = mrpt::opengl::CSetOfObjects::Create();
				scene->insert(gl_rba_scene);

				// Viewport: tree view
				gl_view_cur_tree = scene->createViewport("view_tree");
				gl_view_cur_tree->setViewportPosition(-500,0, 500,300);
				gl_view_cur_tree->setBorderSize(1);
				gl_view_cur_tree->setTransparent(true);
				mrpt::opengl::CCamera & cam = gl_view_cur_tree->getCamera();
				cam.setPointingAt(0,-10,0);
				cam.setElevationDegrees(90);
				cam.setAzimuthDegrees(-90);
				cam.setOrthogonal();
				cam.setZoomDistance(25);

				win->addTextMessage(
					-478,302, "LOCAL SPANNING TREE:",
					mrpt::utils::TColorf(0.8,0.8,0), "mono",11, mrpt::opengl::NICE, TXTID_LABEL_ST,1.5,0.1,
					true /* draw shadow */ );


				gl_cur_kf_tree = mrpt::opengl::CSetOfObjects::Create();
				gl_view_cur_tree->insert(gl_cur_kf_tree);

				// Show ground-truth map?
				if (dataset.has_GT_map() && dataset.has_GT_path())
				{
					win->addTextMessage(
						20,302, "GROUND TRUTH:",
						mrpt::utils::TColorf(0.8,0.8,0), "mono",11, mrpt::opengl::NICE, TXTID_LABEL_GT,1.5,0.1,
						true /* draw shadow */ );

					gl_view_GT = scene->createViewport("view_GT");
					gl_view_GT->setViewportPosition(0,0,500,300);
					gl_view_GT->setBorderSize(1);
					gl_view_GT->setTransparent(false);
					mrpt::opengl::CCamera & cam = gl_view_GT->getCamera();
					cam.setPointingAt(0,0,0);
					cam.setElevationDegrees(90);
					cam.setAzimuthDegrees(-90);
					cam.setOrthogonal();
					cam.setZoomDistance(55);

					// GT Map:
					const mrpt::math::CMatrixD &M = dataset.gt_map();
					const size_t nLMs = M.rows();
					ASSERTMSG_(M.cols()==3, "Ground truth map file: expected exactly 3 columns (X,Y,Z)!")

					gl_gt_map = mrpt::opengl::CPointCloud::Create();
					gl_gt_map->resize(nLMs);
					for (size_t i=0;i<nLMs;i++)
						gl_gt_map->setPoint_fast(i, M(i,0),M(i,1),M(i,2));

					gl_view_GT->insert(gl_gt_map);

					// Ground grid:
					mrpt::math::TPoint3D bb_min, bb_max;
					gl_gt_map->getBoundingBox(bb_min,bb_max);
					gl_view_GT->insert( mrpt::opengl::CGridPlaneXY::Create(bb_min.x-10,bb_max.x+10, bb_min.y-10,bb_max.y+10, -0.001, 10) );

					// GT pose:
					gl_gt_pose = mrpt::opengl::stock_objects::CornerXYZSimple(2.0f,4.0f);
					gl_view_GT->insert(gl_gt_pose);

					gl_gt_path_track = mrpt::opengl::CSetOfLines::Create();
					gl_view_GT->insert(gl_gt_path_track);
					gl_gt_path_track->appendLine(0,0,0, 0,0,0); // Create a first dummy segment so the rest can be "strip lines".
				}

				win->unlockAccess3DScene();
			}
		}

		// Create video?
		mrpt::vision::CVideoFileWriter  outVideo;
		const string sOutVideo =cfg.arg_video.getValue();
		if (cfg.arg_video.isSet() && win)
			win->captureImagesStart();

		const double VIDEO_FPS =cfg.arg_video_fps.getValue();
		const unsigned int GUI_DELAY_MS = cfg.arg_gui_delay.getValue();

		// All LMs with known, fixed relative coordinates: this will be automatically filled-in
		//  the first time a LM is seen, so the current frame  ID becomes its base frame.
		typename my_srba_t::TRelativeLandmarkPosMap  all_fixed_LMs;
		typename my_srba_t::TRelativeLandmarkPosMap  all_unknown_LMs_GT; // Ground truth of relative position for feats with unknown rel.pos.

		// Timming stats:
		map<string,vector<double> >  timming_section_mean_times;
		map<string,vector<size_t> >  timming_section_call_count;
		vector<size_t> timming_section_ts; // The time indices for the above.

		// Simul counters
		srba::TKeyFrameID  frameIdx;
		srba::TKeyFrameID  next_rba_keyframe_ID = 0;
		size_t obsIdx;

		size_t last_obs_idx_reported = 0;
		const size_t REPORT_PROGRESS_OBS_COUNT = 200;

		mrpt::utils::CTicTac  mytimer, mytimer2;
		double new_kf_time=0;
		mytimer.Tic();


		for (frameIdx=0,obsIdx=0; obsIdx<nTotalObs ; )
		{
			const unsigned int nFramesAtOnce =  frameIdx==0  ? INCREMENTAL_FRAMES_AT_ONCE+1 : INCREMENTAL_FRAMES_AT_ONCE;

			// Collect all the observations for the present time step:
			srba::TKeyFrameID curFrameIdx=frameIdx;

			const srba::TKeyFrameID frameIdxMax = frameIdx+nFramesAtOnce;
			frameIdx+=nFramesAtOnce; // for the next round

			//mrpt::vision::TSequenceFeatureObservations  feats_to_draw; // For the GUI

			typename my_srba_t::TNewKeyFrameInfo new_kf_info;

			while (obsIdx<nTotalObs && curFrameIdx<frameIdxMax)
			{
				//size_t  nKnownFeatsInThisFrame = 0;
				typename my_srba_t::new_kf_observations_t  new_obs_in_this_frame;

				if (obsIdx-last_obs_idx_reported>REPORT_PROGRESS_OBS_COUNT)
				{
					last_obs_idx_reported= obsIdx;
					const double dataset_percent_done = (100.*obsIdx)/nTotalObs;

					const double elapsed_tim = mytimer.Tac();
					const double estimated_rem_tim = (dataset_percent_done<100) ? elapsed_tim*(1.0-1.0/(0.01*dataset_percent_done)) : 0;

					cout << "Progress in dataset: obs " << obsIdx << " / " << nTotalObs<< mrpt::format(" (%.4f%%)  Remaining: %s \r",dataset_percent_done, mrpt::system::formatTimeInterval(estimated_rem_tim).c_str() );  // '\r' so if verbose=0 only 1 line is refreshed.
					cout.flush();
				}

				// To emulate graph-SLAM, each keyframe MUST have exactly ONE fixed "fake landmark", representing its pose:
				// ------------------------------------------------------------------------------------------------------------
				if (cfg.arg_graph_slam.isSet())
				{
					typename my_srba_t::new_kf_observation_t obs_field;
					obs_field.is_fixed = true;
					obs_field.obs.feat_id = next_rba_keyframe_ID; // Feature ID == keyframe ID
					//obs_field.obs.obs_data.x = 0;   // Landmark values are actually ignored.
					new_obs_in_this_frame.push_back( obs_field );
				}

				while (obsIdx<nTotalObs && (curFrameIdx=dataset.obs().coeff(obsIdx,0))==next_rba_keyframe_ID)
				{
					// Is this observation of a LM with known (i.e. FIXED) relative position?
					//bool this_feat_has_known_rel_pos = (nKnownFeatsInThisFrame<MAX_KNOWN_FEATS_PER_FRAME);

					// Get observation from the specific dataset parser:
					typename my_srba_t::new_kf_observation_t new_obs;
					dataset.getObs(obsIdx,new_obs.obs);

					new_obs.is_fixed                 = false;
					new_obs.is_unknown_with_init_val = false;

					// Already has a base ID?
#if 0
					const size_t LMidx = new_obs.obs.feat_id;
					typename my_srba_t::TRelativeLandmarkPosMap::const_iterator itRelFeat = all_fixed_LMs.find(LMidx);
					if (itRelFeat == all_fixed_LMs.end())
					{
						nKnownFeatsInThisFrame++;

						// Nope, it's the first observation:
						// Get global location of LM:
						mrpt::math::TPoint3D globalPos;
						GT_MAP.getPoint(LMidx,globalPos);

						// Convert to relative:
						mrpt::math::TPoint3D relPos;
						GT_path[curFrameIdx].inverseComposePoint(
							globalPos.x,globalPos.y,globalPos.z,
							relPos.x,relPos.y,relPos.z);

						const my_srba_t::TRelativeLandmarkPos rfp_GT= my_srba_t::TRelativeLandmarkPos( curFrameIdx, relPos );

						// Add random noise to "measured" relative positions:
						if (this_feat_has_known_rel_pos)
						{
							relPos.x += mrpt::random::randomGenerator.drawGaussian1D(0, REL_POS_NOISE_STD_KNOWN);
							relPos.y += mrpt::random::randomGenerator.drawGaussian1D(0, REL_POS_NOISE_STD_KNOWN);
							relPos.z += mrpt::random::randomGenerator.drawGaussian1D(0, REL_POS_NOISE_STD_KNOWN);
						}
						else
						{
							relPos.x += mrpt::random::randomGenerator.drawGaussian1D(0, REL_POS_NOISE_STD_UNKNOWN);
							relPos.y += mrpt::random::randomGenerator.drawGaussian1D(0, REL_POS_NOISE_STD_UNKNOWN);
							relPos.z += mrpt::random::randomGenerator.drawGaussian1D(0, REL_POS_NOISE_STD_UNKNOWN);
						}

						const my_srba_t::TRelativeLandmarkPos rfp= my_srba_t::TRelativeLandmarkPos( curFrameIdx, relPos );

						// Append to list of feats with Known positions:
						new_obs.setRelPos(relPos);
						new_obs.is_fixed                 =  this_feat_has_known_rel_pos;
						new_obs.is_unknown_with_init_val = !this_feat_has_known_rel_pos;

						// Add also to our simulation list, so future observations of this landmark do not contain a known relative pos.
						all_fixed_LMs[LMidx] = rfp;
						all_unknown_LMs_GT[LMidx] = rfp_GT;
					}
					else
					{
						// Already added to the known relative position,
						//  now we are observing the feature as a 2D pixel coords:
						//rba.add_observation(new_obs);

						//if (DEMO_SIMUL_SHOW_GUI) feats_to_draw.push_back( mrpt::vision::TFeatureObservation(LMidx,curFrameIdx,px) );
					}

#endif
					// Append to the list of observations in the current KF:
					new_obs_in_this_frame.push_back( new_obs );

					++obsIdx; // next observation
				} // end while

				ASSERT_(!new_obs_in_this_frame.empty())

				// Append new key_frame to the RBA state:

				mytimer2.Tic();
				//  ======================================
				//  ==          Here is the beef        ==
				//  ======================================
				rba.define_new_keyframe(
					new_obs_in_this_frame,
					new_kf_info,
					true // Optimize?
					);
				//  ======================================
				new_kf_time = mytimer2.Tac();

				// Append optimization stat as new entries in the time logger:
				{
					mrpt::utils::CTimeLogger &tl = rba.get_time_profiler();
					tl.registerUserMeasure("num_jacobians", new_kf_info.optimize_results.num_jacobians );
					tl.registerUserMeasure("num_kf2kf_edges_optimized", new_kf_info.optimize_results.num_kf2kf_edges_optimized );
					tl.registerUserMeasure("num_kf2lm_edges_optimized", new_kf_info.optimize_results.num_kf2lm_edges_optimized );
				}

				if (DEBUG_DUMP_ALL_SPANNING_TREES)
				{
					static int i=0;
					if (i==0) {
						if (::system("del spantree_*.*")) {;}
						if (::system("rm spantree_*.*")) {;}
					}

					//const string sFil = mrpt::format("spantree_%05i.txt",i);
					//rba.get_rba_state().spanning_tree.dump_as_text_to_file(sFil);

					vector<srba::TKeyFrameID> roots_to_save;  // leave empty: save all
					roots_to_save.push_back( new_kf_info.kf_id );

					const string sFilDot = mrpt::format("spantree_%05i.dot",i);
					const string sFilPng = mrpt::format("spantree_%05i.png",i);
					rba.get_rba_state().spanning_tree.save_as_dot_file(sFilDot, roots_to_save);

					// Render DOT -> PNG
					if (::system(mrpt::format("dot %s -o %s -Tpng",sFilDot.c_str(), sFilPng.c_str() ).c_str()))
						std::cout << "Error running external dot command...\n";

					i++;
				}

				next_rba_keyframe_ID = 1 + new_kf_info.kf_id;  // Update last KF ID:

				if (obsIdx<nTotalObs)
				{
					ASSERT_EQUAL_(next_rba_keyframe_ID, curFrameIdx)  // This should occur if key_frames in simulation are ordered
				}

			} // end while (for each KF to process at once)

			// Eval RMSE:
			const double RMSE = new_kf_info.optimize_results.num_observations ? std::sqrt(new_kf_info.optimize_results.total_sqr_error_final / new_kf_info.optimize_results.num_observations) : 0;

			// Draw 3D scene of the SRBA map:
			if (DEMO_SIMUL_SHOW_GUI && win && win->isOpen())
			{
				// Update text labels:
				win->addTextMessage(
					5,-15,
					mrpt::format("Current KF=%u",static_cast<unsigned int>(new_kf_info.kf_id)),
					mrpt::utils::TColorf(0.9,0.9,0.9), "mono",10, mrpt::opengl::NICE, TXTID_KF,1.5,0.1,
					true /* draw shadow */ );

				win->addTextMessage(
					5,-30,
					mrpt::format("After opt RMSE=%.06f | Total step time=%.03fms",RMSE, 1e3*new_kf_time ),
					mrpt::utils::TColorf(0.9,0.9,0.9), "mono",10, mrpt::opengl::NICE, TXTID_RMSE,1.5,0.1,
					true /* draw shadow */ );

				// Update 3D objects:
				const srba::TKeyFrameID root_kf = next_rba_keyframe_ID-1;
				typename my_srba_t::TOpenGLRepresentationOptions opengl_params;
				opengl_params.span_tree_max_depth = rba.parameters.srba.max_tree_depth; // Render these past keyframes at most.
				opengl_params.draw_unknown_feats_ellipses_quantiles = 3;

				mrpt::opengl::CSetOfObjectsPtr gl_obj = mrpt::opengl::CSetOfObjects::Create();
				mrpt::opengl::CSetOfObjectsPtr gl_obj_tree = mrpt::opengl::CSetOfObjects::Create();

				rba.build_opengl_representation(
					root_kf,
					opengl_params,
					gl_obj,
					gl_obj_tree
					);


				win->get3DSceneAndLock();

				gl_rba_scene->clear();
				gl_rba_scene->insert( mrpt::opengl::CGridPlaneXY::Create(-50,50,-50,50,0,1) );
				gl_rba_scene->insert(gl_obj);

				gl_view_cur_tree->clear();
				gl_view_cur_tree->insert(gl_obj_tree);

				mrpt::math::TPoint3D bbmin,bbmax;
				gl_obj_tree->getBoundingBox(bbmin,bbmax);
				const double cur_tree_width = 1+std::min(bbmax.x-bbmin.x, bbmax.y-bbmin.y);
				gl_view_cur_tree->getCamera().setZoomDistance(1.5*cur_tree_width);

				// Update ground truth:
				if (gl_view_GT)
				{
					const mrpt::poses::CPose3DQuat & gt_pose = dataset.gt_path(new_kf_info.kf_id);

					mrpt::opengl::CCamera & cam = gl_view_GT->getCamera();
					cam.setPointingAt(gt_pose.x(),gt_pose.y(),gt_pose.z());
					gl_gt_pose->setPose(mrpt::poses::CPose3D(gt_pose));

					gl_gt_path_track->appendLineStrip(gt_pose.x(),gt_pose.y(),gt_pose.z());
				}


				win->unlockAccess3DScene();
				win->repaint();
				if (GUI_DELAY_MS) mrpt::system::sleep(GUI_DELAY_MS);
			}

			// Write images to video?
			if (cfg.arg_video.isSet() && win && win->isOpen())
			{
				// Screenshot:
				const mrpt::utils::CImagePtr pImg = win->getLastWindowImagePtr();
				if (pImg)
				{
					// Create video upon first pass:
					if (!outVideo.isOpen())
					{
						outVideo.open(sOutVideo,VIDEO_FPS, pImg->getSize(), "PIM1" );

						if (!outVideo.isOpen())
							cerr << "Error opening output video file!" << endl;
					}

					// Write image to video:
					outVideo << *pImg;
				}
			}


	#if 0
			cout << "Optimized edges:\n";
			for (srba::RbaEngine::k2k_edges_t::const_iterator itEdge=rba.get_k2k_edges().begin();itEdge!=rba.get_k2k_edges().end();++itEdge)
			{
				const mrpt::utils::TNodeID nodeFrom = itEdge->from;
				const mrpt::utils::TNodeID nodeTo   = itEdge->to;
				cout << nodeFrom << " -> " << nodeTo << ": " << itEdge->inv_pose << endl;
			}
	#endif
	#if 0
			cout << "Optimized feats:\n";
			const mrpt::srba::TRelativeLandmarkPosMap &unkn_feats = rba.get_unknown_feats();
			// vs: all_unknown_LMs_GT
			for (mrpt::srba::TRelativeLandmarkPosMap::const_iterator it=unkn_feats.begin();it!=unkn_feats.end();++it)
			{
				mrpt::srba::TRelativeLandmarkPosMap::const_iterator it_gt = all_unknown_LMs_GT.find(it->first);
				ASSERT_(it_gt != all_unknown_LMs_GT.end())
				cout << "Feat: " << it->first << " -> " << it->second.pos << " vs GT= " << it_gt->second.pos << endl;
			}
	#endif

			// Profile stats:
			if (SAVE_TIMING_STATS)
			{
				if ((curFrameIdx % SAVE_TIMING_SEGMENT_LENGTH) == 0  )
				{
					// Append time index:
					timming_section_ts.push_back( curFrameIdx );

					// Append times:
					map<string,mrpt::utils::CTimeLogger::TCallStats> cur_stats;
					rba.get_time_profiler().getStats(cur_stats);

					for (map<string,mrpt::utils::CTimeLogger::TCallStats>::const_iterator it=cur_stats.begin();it!=cur_stats.end();++it)
					{
						timming_section_mean_times[ it->first ].push_back( it->second.mean_t );
						timming_section_call_count[ it->first ].push_back( it->second.n_calls );
					}

					// Other useful stats (although not being times):
					timming_section_mean_times[ "total_num_kfs" ].push_back( rba.get_rba_state().keyframes.size() );
					timming_section_mean_times[ "total_num_lms" ].push_back( rba.get_known_feats().size() + rba.get_unknown_feats().size()  );
					timming_section_mean_times[ "total_num_obs" ].push_back( rba.get_rba_state().all_observations.size() );

					rba.get_time_profiler().clear(); // Reset timers stats
				}
			}



			// Process user keys?
			if (win && win->isOpen() && (DEMO_SIMUL_SHOW_STEPBYSTEP || win->keyHit()))
			{
				const int char_code = win->waitForKey();

				switch (char_code)
				{
				case 'r': // Resume:
					DEMO_SIMUL_SHOW_STEPBYSTEP=!DEMO_SIMUL_SHOW_STEPBYSTEP;
					break;
				case 's': // save 3d scene
					{
						static int cnt=0;
						mrpt::opengl::COpenGLScene scene;
						scene.insert(gl_rba_scene);
						const string sFil = mrpt::format("rba-simul-%05i.3Dscene",++cnt);
						cout << "** SAVING 3D SCENE TO: " << sFil << " **\n";
						// This file can be visualized with "SceneViewer3D"
						mrpt::utils::CFileGZOutputStream(sFil) << scene;
					}
					break;
				case 'd': // save DOT graph
					{
						static int cnt=0;
						const string sDotFil = mrpt::format("rba-simul-%05i-full.dot",cnt);
						const string sPngFil = mrpt::format("rba-simul-%05i-full.png",cnt);
						cnt++;

						cout << "** SAVING FULL DOT SCENE TO: " << sDotFil << " **\n";
						rba.save_graph_as_dot(sDotFil, false);

						cout << "Converting to PNG...";
						const string sCmd = mrpt::format("dot -Tpng -o %s %s",sPngFil.c_str(), sDotFil.c_str());
						if (0!=::system(sCmd.c_str()))
						{
							cerr << "> ERROR Running: " << sCmd << endl;
						}
						else
							cout << "Done! " << sPngFil << endl;
					}
					break;

				case 27:  // "ESC" key
					{
						obsIdx=nTotalObs+1; // End simulation
						cout << "Quitting by user command.\n";
					}
					break;
				default:
					break;
				}
			}

			if ((!win || !win->isOpen()) && mrpt::system::os::kbhit())
			{
				const int c = mrpt::system::os::getch();
				switch (c)
				{
				case 27:  // "ESC" key
					{
						obsIdx=nTotalObs+1; // End simulation
						cout << "Quitting by user command.\n";
					}
					break;
				default:
					break;
				}
			}

		} // end for-each frame ID in the dataset


		// Evaluate errors vs. ground truth:
		// ---------------------------------------
#if 0
		if (cfg.arg_eval_overall_se3_error.isSet())
		{
			double total_edge_sqr_err=0;
			for (my_srba_t::k2k_edges_deque_t::const_iterator itEdge=rba.get_k2k_edges().begin();itEdge!=rba.get_k2k_edges().end();++itEdge)
			{
				const mrpt::utils::TNodeID nodeFrom = itEdge->from;
				const mrpt::utils::TNodeID nodeTo   = itEdge->to;

				const mrpt::poses::CPose3D GT_edge = mrpt::poses::CPose3D( GT_path[nodeFrom] - GT_path[nodeTo] ); // (edges store *inverse* poses)
				const mrpt::poses::CPose3D edgeErr = itEdge->inv_pose - GT_edge;
				total_edge_sqr_err+= edgeErr.ln().squaredNorm();
			}
			const double mean_edge_err = sqrt(total_edge_sqr_err/rba.get_k2k_edges().size());
			cout << "Average edge se(3) error: " << mean_edge_err << endl;
		}

		// Evaluate overall observation reprojection errors:
		// ------------------------------------------------------
		if (cfg.arg_eval_overall_sqr_error.isSet())
		{
			cout << "Evaluating overall observation errors...\n"; cout.flush();
			double total_obs_sqr_err = rba.eval_overall_squared_error();
			cout << "total_obs_sqr_err = " << total_obs_sqr_err << endl;

			const size_t nObs = rba.get_rba_state().all_observations.size();
			if (nObs) cout << "RMSE per observation = " << sqrt(total_obs_sqr_err / nObs ) << endl;
		}

		if (cfg.arg_eval_connectivity.isSet())
		{
			{
				double deg_mean, deg_std, deg_max;
				rba.get_rba_state().compute_all_node_degrees(deg_mean, deg_std, deg_max);
				cout << "Keyframe connectivity (degree) stats:\n"
				<< " mean = " << deg_mean << endl
				<< " std  = " << deg_std << endl
				<< " max  = " << deg_max << endl;
			}
			{
				size_t num_min, num_max;
				double num_mean,num_std;
				rba.get_rba_state().spanning_tree.get_stats(num_min, num_max,num_mean,num_std);
				cout << "Spanning tree sizes stats:\n"
				<< " mean = " << num_mean << endl
				<< " std  = " << num_std << endl
				<< " max  = " << num_max << endl
				<< " min  = " << num_min << endl;
			}
		}
#endif

		// Save profile stats to disk
		if (SAVE_TIMING_STATS)
		{
			const string sStatsPrefix = mrpt::format("%s_%s", mrpt::system::fileNameStripInvalidChars(cfg.arg_profile_stats.getValue()).c_str(), mrpt::system::fileNameStripInvalidChars( mrpt::system::dateTimeToString( mrpt::system::getCurrentLocalTime() ) ).c_str() );

			const string sStatsMean = sStatsPrefix + string("_means.csv");
			const string sStatsCalls = sStatsPrefix + string("_calls.csv");

			cout << "Saving time stats to:\n" << sStatsMean << endl << sStatsCalls << endl;

			std::stringstream s_means, s_calls;

			const size_t N = timming_section_ts.size();

			// Header line:
			s_means << "timestep";
			s_calls << "timestep";
			for (map<string,vector<double> >::const_iterator it=timming_section_mean_times.begin();it!=timming_section_mean_times.end();++it)
			{
				if (it->second.size()!=N) continue;

				s_means << "\t \"" << it->first << "\"";
				s_calls << "\t \"" << it->first << "\"";
			}
			s_means << endl;
			s_calls << endl;

			// The rest of lines:
			for (size_t i=0;i<N;i++)
			{
				s_means << timming_section_ts[i];
				for (map<string,vector<double> >::const_iterator it=timming_section_mean_times.begin();it!=timming_section_mean_times.end();++it)
				{
					if (it->second.size()!=N) continue;
					s_means << "\t" << it->second[i];
				}
				s_means << endl;

				s_calls << timming_section_ts[i];
				for (map<string,vector<size_t> >::const_iterator it=timming_section_call_count.begin();it!=timming_section_call_count.end();++it)
				{
					if (it->second.size()!=N) continue;
					s_calls << "\t" << it->second[i];
				}
				s_calls << endl;
			}

			mrpt::utils::CFileOutputStream f_means(sStatsMean), f_calls(sStatsCalls);
			f_means.printf("%s", s_means.str().c_str() );
			f_calls.printf("%s", s_calls.str().c_str() );

			cout << "Done.\n";
		}


		if (cfg.arg_save_final_graph.isSet())
		{
			const string sFil =cfg.arg_save_final_graph.getValue();
			const string sFilPng = mrpt::system::fileNameChangeExtension(sFil,"png");

			cout << "Saving final graph of KFs to: " << sFil << endl;
			rba.save_graph_as_dot(sFil, false /* LMs=don't save */);
			cout << "Done. Converting to PNG...\n";

			// Render DOT -> PNG
			if (!::system(mrpt::format("dot %s -o %s -Tpng",sFil.c_str(), sFilPng.c_str() ).c_str()))
					std::cout << "Done.\n";
			else 	std::cout << "Error running external dot command...\n";
		}

		if (cfg.arg_save_final_graph_landmarks.isSet())
		{
			const string sFil =cfg.arg_save_final_graph_landmarks.getValue();
			cout << "Saving final graph of KFs and LMs to: " << sFil << endl;
			rba.save_graph_as_dot(sFil, true /* LMs=save */);
			cout << "Done.\n";
		}

		if (DEMO_SIMUL_SHOW_GUI && win && win->isOpen())
		{
			cout << "Press any key or close the window to end.\n";
			win->waitForKey();
		}




		return 0; // No error.
	} // end of run()

}; // end of RBA_Run

