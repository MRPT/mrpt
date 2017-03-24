/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

/* ===========================================================================
    EXAMPLE: bundle_adj_full_demo
    PURPOSE: Demonstrate "mrpt::vision::bundle_adj_full" with a set of
	          simulated or real data. If the program is called without command
			  line arguments, simulated measurements will be used.
			  To use real data, invoke:
			    bundle_adj_full_demo  <feats.txt> <cam_model.cfg>

			  Where <feats.txt> is a "TSequenceFeatureObservations" saved as
			  a text file, and <cam_model.cfg> is a .ini-like file with a
			  section named "CAMERA" loadable by mrpt::utils::TCamera.

    DATE: 20-Aug-2010
   =========================================================================== */

#include <mrpt/vision/bundle_adjustment.h>
#include <mrpt/vision/pinhole.h>
#include <mrpt/gui/CDisplayWindow3D.h>
#include <mrpt/gui/CDisplayWindowPlots.h>
#include <mrpt/random.h>
#include <mrpt/math/geometry.h>
#include <mrpt/system/filesystem.h>
#include <mrpt/utils/CConfigFile.h>
#include <mrpt/utils/CTextFileLinesParser.h>
#include <mrpt/opengl/CGridPlaneXY.h>
#include <mrpt/opengl/CPointCloud.h>
#include <mrpt/opengl/stock_objects.h>


using namespace mrpt;
using namespace mrpt::gui;
using namespace mrpt::utils;
using namespace mrpt::math;
using namespace mrpt::opengl;
using namespace mrpt::poses;
using namespace mrpt::vision;
using namespace std;


CVectorDouble history_avr_err;

double WORLD_SCALE = 1;  // Will change when loading SBA examples

// A feedback functor, which is called on each iteration by the optimizer to let us know on the progress:
void my_BundleAdjustmentFeedbackFunctor(
	const size_t cur_iter,
	const double cur_total_sq_error,
	const size_t max_iters,
	const TSequenceFeatureObservations & input_observations,
	const TFramePosesVec & current_frame_estimate,
	const TLandmarkLocationsVec & current_landmark_estimate )
{
	const double avr_err = std::sqrt(cur_total_sq_error/input_observations.size());
	history_avr_err.push_back(std::log(1e-100+avr_err));
	if ((cur_iter % 10)==0)
	{
		cout << "[PROGRESS] Iter: " << cur_iter << " avrg err in px: " << avr_err << endl;
		cout.flush();
	}
}


// ------------------------------------------------------
//				bundle_adj_full_demo
// ------------------------------------------------------
void bundle_adj_full_demo(
	const TCamera                        & camera_params,
	const TSequenceFeatureObservations   & allObs,
	TFramePosesVec			& frame_poses,
	TLandmarkLocationsVec   & landmark_points
	)
{
	cout << "Optimizing " << allObs.size() << " feature observations.\n";

	TParametersDouble 		extra_params;
	//extra_params["verbose"] = 1;
	extra_params["max_iterations"]= 2000; //250;
	//extra_params["num_fix_frames"] = 1;
	//extra_params["num_fix_points"] = 0;
	extra_params["robust_kernel"] = 0;
	extra_params["kernel_param"]  = 5.0;
	extra_params["profiler"] = 1;

	mrpt::vision::bundle_adj_full(
		allObs,
		camera_params,
		frame_poses,
		landmark_points,
		extra_params,
		&my_BundleAdjustmentFeedbackFunctor
		);
}
// ---------------------------------------------------------

mrpt::opengl::CSetOfObjectsPtr framePosesVecVisualize(
	const TFramePosesVec &poses,
	const double  len,
	const double  lineWidth);


// ------------------------------------------------------
//						MAIN
// ------------------------------------------------------
int main(int argc, char **argv)
{
	try
	{
		// Simulation or real-data? (read at the top of this file):
		if ((argc!=1 && argc!=3 && argc!=4) || (argc==2 && !strcpy(argv[1],"--help")) )
		{
			cout << "Usage:\n"
				<< argv[0] << " --help -> Shows this help\n"
				<< argv[0] << "     -> Simulation\n"
				<< argv[0] << " <feats.txt> <cam_model.cfg> -> Data in MRPT format\n"
				<< argv[0] << " <cams.txt> <points.cfg> <calib.txt> -> SBA format\n";
			return 1;
		}

		// BA data:
		TCamera                       camera_params;
		TSequenceFeatureObservations  allObs;
		TFramePosesVec                frame_poses;
		TLandmarkLocationsVec         landmark_points;

		// Only for simulation mode:
		TFramePosesVec                frame_poses_real, frame_poses_noisy; // Ground truth & starting point
		TLandmarkLocationsVec         landmark_points_real, landmark_points_noisy; // Ground truth & starting point

		if (argc==1)
		{
			random::CRandomGenerator rg(1234);

			//  Simulation
			// --------------------------
			// The projective camera model:
			camera_params.ncols = 800;
			camera_params.nrows = 600;
			camera_params.fx(400); camera_params.fy(400);
			camera_params.cx(400); camera_params.cy(300);

			//      Generate synthetic dataset:
			// -------------------------------------
			const size_t nPts = 100;  // # of 3D landmarks
			const double L1 = 60;   // Draw random poses in the rectangle L1xL2xL3
			const double L2 = 10;
			const double L3 = 10;
			const double max_camera_dist = L1*4;

			const double cameraPathLen = L1*1.2;
			//const double cameraPathEllipRadius1 = L1*2;
			//const double cameraPathEllipRadius2 = L2*2;
			// Noise params:
			const double STD_PX_ERROR= 0.10; // pixels
			
			const double STD_PX_ERROR_OUTLIER= 5; // pixels
			const double PROBABILITY_OUTLIERS = 0; //0.01;

			const double STD_PT3D    = 0.10; // meters
			const double STD_CAM_XYZ = 0.05; // meters
			const double STD_CAM_ANG = DEG2RAD(5); // degs


			landmark_points_real.resize(nPts);
			for (size_t i=0;i<nPts;i++)
			{
				landmark_points_real[i].x = rg.drawUniform(-L1,L1);
				landmark_points_real[i].y = rg.drawUniform(-L2,L2);
				landmark_points_real[i].z = rg.drawUniform(-L3,L3);
			}

			//const double angStep = M_PI*2.0/40;
			const double camPosesSteps = 2*cameraPathLen/20;
			frame_poses_real.clear();

			for (double x=-cameraPathLen;x<cameraPathLen;x+=camPosesSteps)
			{
				TPose3D  p;
				p.x = x; //cameraPathEllipRadius1 * cos(ang);
				p.y = 4*L2;  // cameraPathEllipRadius2 * sin(ang);
				p.z = 0;
				p.yaw = DEG2RAD(-90) - DEG2RAD(30)*x/cameraPathLen  ; // wrapToPi(ang+M_PI);
				p.pitch = 0;
				p.roll  = 0;
				// Angles above is for +X pointing to the (0,0,0), but we want instead +Z pointing there, as typical in camera models:
				frame_poses_real.push_back( CPose3D(p) + CPose3D(0,0,0,DEG2RAD(-90), 0, DEG2RAD(-90)) );
			}

			// Simulate the feature observations:
			size_t numOutliers=0;
			allObs.clear();
			map<TCameraPoseID, size_t> numViewedFrom;
			for (size_t i=0;i<frame_poses_real.size();i++) // for each pose
			{
				// predict all landmarks:
				for (size_t j=0;j<landmark_points_real.size();j++)
				{
					TPixelCoordf px = mrpt::vision::pinhole::projectPoint_no_distortion<false>(camera_params, frame_poses_real[i], landmark_points_real[j]);

					const bool is_outlier = (rg.drawUniform(0.0,1.0)<PROBABILITY_OUTLIERS);
					px.x += rg.drawGaussian1D(0, is_outlier ? STD_PX_ERROR_OUTLIER : STD_PX_ERROR);
					px.y += rg.drawGaussian1D(0, is_outlier ? STD_PX_ERROR_OUTLIER : STD_PX_ERROR);

					// Out of image?
					if (px.x<0 || px.y<0 || px.x>camera_params.ncols || px.y>camera_params.nrows)
						continue;

					// Too far?
					const double dist = math::distance( TPoint3D(frame_poses_real[i]), landmark_points_real[j]);
					if (dist>max_camera_dist)
						continue;

					// Ok, accept it:
					if (is_outlier) numOutliers++;
					allObs.push_back( TFeatureObservation(j,i, px) );
					numViewedFrom[i]++;
				}
			}
			cout << "Simulated: " << allObs.size() << " observations (of which: "<< numOutliers << " are outliers).\n";

			ASSERT_EQUAL_(numViewedFrom.size(), frame_poses_real.size())

			// Make sure all poses and all LMs appear at least once!
			{
				TSequenceFeatureObservations allObs2 = allObs;
				std::map<TCameraPoseID,TCameraPoseID>  old2new_camIDs;
				std::map<TLandmarkID,TLandmarkID>      old2new_lmIDs;
				allObs2.compressIDs(&old2new_camIDs, &old2new_lmIDs);

				ASSERT_EQUAL_(old2new_camIDs.size(),frame_poses_real.size() )
				ASSERT_EQUAL_(old2new_lmIDs.size(), landmark_points_real.size() )
			}


			// Add noise to the data:
			frame_poses_noisy     = frame_poses_real;
			landmark_points_noisy = landmark_points_real;
			for (size_t i=0;i<landmark_points_noisy.size();i++)
				landmark_points_noisy[i] += TPoint3D( rg.drawGaussian1D(0,STD_PT3D),rg.drawGaussian1D(0,STD_PT3D),rg.drawGaussian1D(0,STD_PT3D) );

			for (size_t i=1;i<frame_poses_noisy.size();i++) // DON'T add error to frame[0], the global reference!
			{
				CPose3D bef = frame_poses_noisy[i];
				frame_poses_noisy[i].setFromValues(
					frame_poses_noisy[i].x() + rg.drawGaussian1D(0,STD_CAM_XYZ),
					frame_poses_noisy[i].y() + rg.drawGaussian1D(0,STD_CAM_XYZ),
					frame_poses_noisy[i].z() + rg.drawGaussian1D(0,STD_CAM_XYZ),
					frame_poses_noisy[i].yaw()   + rg.drawGaussian1D(0,STD_CAM_ANG),
					frame_poses_noisy[i].pitch() + rg.drawGaussian1D(0,STD_CAM_ANG),
					frame_poses_noisy[i].roll()  + rg.drawGaussian1D(0,STD_CAM_ANG) );
			}

			// Optimize it:
			frame_poses     = frame_poses_noisy;
			landmark_points = landmark_points_noisy;

#if 0
			vector<CArray<double,2> > resids;
			const double initial_total_sq_err = mrpt::vision::reprojectionResiduals(allObs,camera_params,frame_poses, landmark_points,resids, false);
			cout << "Initial avr error in px: " << std::sqrt(initial_total_sq_err/allObs.size()) << endl;
#endif

			// Run Bundle Adjustmen
			bundle_adj_full_demo( camera_params, allObs, frame_poses, landmark_points );


			// Evaluate vs. ground truth:
			double landmarks_total_sq_err=0;
			for (size_t i=0;i<landmark_points.size();i++)
				landmarks_total_sq_err+= square( landmark_points_real[i].distanceTo(landmark_points[i]) );

			double cam_point_total_sq_err=0;
			for (size_t i=0;i<frame_poses.size();i++)
				cam_point_total_sq_err+= square( frame_poses[i].distanceTo(frame_poses_real[i]) );

			cout << "RMSE of recovered landmark positions: " << std::sqrt(landmarks_total_sq_err/landmark_points.size()) << endl;
			cout << "RMSE of recovered camera positions: " << std::sqrt(cam_point_total_sq_err/frame_poses.size()) << endl;

		}
		else
		{
			//  Real data
			// --------------------------
			if (argc==3)
			{
				const string feats_fil = string(argv[1]);
				const string cam_fil   = string(argv[2]);

				cout << "Loading observations from: " << feats_fil << "...";cout.flush();
				allObs.loadFromTextFile(feats_fil);
				cout << "Done.\n";

				allObs.decimateCameraFrames(20);
				allObs.compressIDs();

				ASSERT_(mrpt::system::fileExists(cam_fil))
				cout << "Loading camera params from: " << cam_fil;
				CConfigFile cfgCam(cam_fil);
				camera_params.loadFromConfigFile("CAMERA",cfgCam);
				cout << "Done.\n";

				cout << "Initial gross estimate...";
				mrpt::vision::ba_initial_estimate(
					allObs,
					camera_params,
					frame_poses,
					landmark_points);
				cout << "OK\n";

			}
			else
			{
				// Load data from 3 files in the same format as used by "eucsbademo" in the SBA library:
				const string cam_frames_fil = string(argv[1]);
				const string obs_fil   = string(argv[2]);
				const string calib_fil = string(argv[3]);

				{
					cout << "Loading initial camera frames from: " << cam_frames_fil << "...";cout.flush();

					mrpt::utils::CTextFileLinesParser  fil(cam_frames_fil);
					frame_poses.clear();

					std::istringstream ss;
					while (fil.getNextLine(ss))
					{
						double q[4],t[3];
						ss >> q[0] >>q[1] >>q[2] >>q[3] >> t[0]>> t[1]>> t[2];
						mrpt::poses::CPose3DQuat p(t[0],t[1],t[2], mrpt::math::CQuaternionDouble(q[0],q[1],q[2],q[3]) );
						//cout << "cam: " << p << endl;
						frame_poses.push_back( CPose3D(p) );
					}

					cout << "Done. " << frame_poses.size() << " cam frames loaded\n";

					frame_poses_noisy = frame_poses; // To draw in 3D the initial values as well.
				}

				{
					cout << "Loading observations & feature 3D points from: " << obs_fil << "...";cout.flush();

					mrpt::utils::CTextFileLinesParser  fil(obs_fil);
					landmark_points.clear();
					allObs.clear();

					std::istringstream ss;
					while (fil.getNextLine(ss))
					{
						// # X Y Z  nframes  frame0 x0 y0  frame1 x1 y1 ...
						double t[3];
						size_t N=0;
						ss >> t[0]>> t[1]>> t[2] >> N;

						const TLandmarkID feat_id = landmark_points.size();
						const TPoint3D pt(t[0],t[1],t[2]);
						landmark_points.push_back( pt );

						// Read obs:
						for (size_t i=0;i<N;i++)
						{
							TCameraPoseID frame_id;
							TPixelCoordf px;
							ss >> frame_id >> px.x >> px.y;
							allObs.push_back(TFeatureObservation(feat_id,frame_id,px));
							//cout << "feat: " << feat_id << " cam: " << frame_id << " px: " << px.x << "," << px.y << endl;
						}
					}

					cout << "Done. " << landmark_points.size() << " points, " << allObs.size() << " observations read.\n";

					landmark_points_real = landmark_points; // To draw in 3D the initial values as well.
				}

				CMatrixDouble33 cam_pars;
				cam_pars.loadFromTextFile(calib_fil);

				//cout << "Calib:\n" << cam_pars << endl;

				camera_params.fx( cam_pars(0,0) );
				camera_params.fy( cam_pars(1,1) );
				camera_params.cx( cam_pars(0,2) );
				camera_params.cy( cam_pars(1,2) );

				cout << "camera calib:\n" <<camera_params.dumpAsText()<<endl;

				// Change world scale:
				WORLD_SCALE = 2000;

			}

			// Do it:
			bundle_adj_full_demo( camera_params, allObs, frame_poses, landmark_points );
		}

		// Display results in 3D:
		// -------------------------------
		gui::CDisplayWindow3D  win("Bundle adjustment demo", 800,600);

		COpenGLScenePtr &scene = win.get3DSceneAndLock();


		{	// Ground plane:
			CGridPlaneXYPtr obj = CGridPlaneXY::Create(-200,200,-200,200,0, 5);
			obj->setColor(0.7,0.7,0.7);
			scene->insert(obj);
		}

		if (!landmark_points_real.empty())
		{	// Feature points: ground truth
			CPointCloudPtr obj =  CPointCloud::Create();
			obj->setPointSize(2);
			obj->setColor(0,0,0);
			obj->loadFromPointsList(landmark_points_real);
			obj->setScale(WORLD_SCALE);
			scene->insert(obj);
		}
		if (!landmark_points_noisy.empty())
		{	// Feature points: noisy
			CPointCloudPtr obj =  CPointCloud::Create();
			obj->setPointSize(4);
			obj->setColor(0.7,0.2,0.2, 0);
			obj->loadFromPointsList(landmark_points_noisy);
			obj->setScale(WORLD_SCALE);
			scene->insert(obj);
		}

		{	// Feature points: estimated
			CPointCloudPtr obj =  CPointCloud::Create();
			obj->setPointSize(3);
			obj->setColor(0,0,1, 1.0);
			obj->loadFromPointsList(landmark_points);
			obj->setScale(WORLD_SCALE);
			scene->insert(obj);
		}

		// Camera Frames: estimated
		scene->insert( framePosesVecVisualize(frame_poses_noisy,1.0,  1) );
		scene->insert( framePosesVecVisualize(frame_poses_real,2.0,  1) );
		scene->insert( framePosesVecVisualize(frame_poses,2.0, 3) );


		win.setCameraZoom(100);

		win.unlockAccess3DScene();
		win.repaint();

		// Also, show history of error:
		gui::CDisplayWindowPlots winPlot("Avr log-error with iterations",500,200);
		//winPlot.setPos(0,620);
		winPlot.plot( history_avr_err, "b.3");
		winPlot.axis_fit();

		cout << "Close the 3D window or press a key to exit.\n";
		win.waitForKey();

		return 0;
	} catch (std::exception &e)
	{
		std::cout << "MRPT exception caught: " << e.what() << std::endl;
		return -1;
	}
	catch (...)
	{
		printf("Untyped exception!!");
		return -1;
	}
}


mrpt::opengl::CSetOfObjectsPtr framePosesVecVisualize(
	const TFramePosesVec &poses,
	const double  len,
	const double  lineWidth)
{
	mrpt::opengl::CSetOfObjectsPtr obj = mrpt::opengl::CSetOfObjects::Create();

	for (size_t i=0;i<poses.size();i++)
	{
		CSetOfObjectsPtr corner = opengl::stock_objects::CornerXYZSimple(len, lineWidth);
		CPose3D  p = poses[i];
		p.x( WORLD_SCALE *p.x() );
		p.y( WORLD_SCALE *p.y() );
		p.z( WORLD_SCALE *p.z() );
		corner->setPose(p);
		corner->setName(format("%u",(unsigned int)i ));
		corner->enableShowName();
		obj->insert(corner);
	}
	return obj;
}

