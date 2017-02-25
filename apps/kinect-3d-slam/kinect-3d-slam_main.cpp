/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

/*
  Application  : kinect-3d-slam
  Web page     : http://www.mrpt.org/Kinect_and_MRPT

  Purpose      : Demonstrate grabbing from CKinect, multi-threading, live 3D
				  rendering and features tracking.

  Started by Jose Luis Blanco, Dec 6th, 2010.

Note: This is a very *simple* approach to SLAM. It would be better to first select
      3d points in the 3D point-cloud, then project them into the image plane
	  and then track them (instead of directly choosing poins in the image plane),
	  since in that case the (x,y) to 3D correspondence would be much more accurate.
	  Feel free to modify or improve this example as you need!

*/


#include <mrpt/hwdrivers/CKinect.h>
#include <mrpt/gui/CDisplayWindow3D.h>
#include <mrpt/vision/tracking.h>
#include <mrpt/tfest/se3.h>
#include <mrpt/system/filesystem.h>
#include <mrpt/synch/CThreadSafeVariable.h>
#include <mrpt/utils/CConfigFile.h>
#include <mrpt/maps/CColouredPointsMap.h>
#include <mrpt/opengl/CPointCloudColoured.h>
#include <mrpt/opengl/CGridPlaneXY.h>
#include <mrpt/opengl/stock_objects.h>

using namespace mrpt;
using namespace mrpt::vision;
using namespace mrpt::hwdrivers;
using namespace mrpt::poses;
using namespace mrpt::math;
using namespace mrpt::gui;
using namespace mrpt::utils;
using namespace mrpt::obs;
using namespace mrpt::maps;
using namespace mrpt::opengl;
using namespace std;

const double KEYFRAMES_MIN_DISTANCE = 0.10; // meters
const double KEYFRAMES_MIN_ANG      = DEG2RAD(10);

// Thread for grabbing: Do this is another thread so we divide rendering and grabbing
//   and exploit multicore CPUs.
struct TThreadParam
{
	TThreadParam() : quit(false), pushed_key(0), tilt_ang_deg(0), Hz(0) { }

	volatile bool   quit;
	volatile int    pushed_key;
	volatile double tilt_ang_deg;
	volatile double Hz;

	mrpt::synch::CThreadSafeVariable<CObservation3DRangeScanPtr> new_obs;
};

void thread_grabbing(TThreadParam &p)
{
	try
	{
		CKinect  kinect;

		// Set params:
		// kinect.enableGrab3DPoints(true);
		// kinect.enablePreviewRGB(true);
		//...
		const std::string cfgFile = "kinect_calib.cfg";
		if (mrpt::system::fileExists(cfgFile))
		{
			cout << "Loading calibration from: "<< cfgFile << endl;
			kinect.loadConfig( mrpt::utils::CConfigFile(cfgFile), "KINECT" );
		}
		else cerr << "Warning: Calibration file ["<< cfgFile <<"] not found -> Using default params. "
			"This may cause innacurate 3D point clouds.\n Checkout the kinect-stereo-calib application:\n http://www.google.com/search?q=kinect-stereo-calib%20mrpt&btnI \n";

		// Open:
		cout << "Calling CKinect::initialize()...";
		kinect.initialize();
		cout << "OK\n";

		CTicTac tictac;
		int nImgs = 0;
		bool there_is_obs=true, hard_error=false;

		while (!hard_error && !p.quit)
		{
			// Grab new observation from the camera:
			CObservation3DRangeScanPtr  obs = CObservation3DRangeScan::Create(); // Smart pointer to observation
			kinect.getNextObservation(*obs,there_is_obs,hard_error);

			if (!hard_error && there_is_obs)
			{
				p.new_obs.set(obs);
			}

			if (p.pushed_key!=0)
			{
				switch (p.pushed_key)
				{
					case 's':
						p.tilt_ang_deg-=2;
						if (p.tilt_ang_deg<-30) p.tilt_ang_deg=-30;
						kinect.setTiltAngleDegrees(p.tilt_ang_deg);
						break;
					case 'w':
						p.tilt_ang_deg+=2;
						if (p.tilt_ang_deg>30) p.tilt_ang_deg=30;
						kinect.setTiltAngleDegrees(p.tilt_ang_deg);
						break;
					case 27:
						p.quit = true;
						break;
				}

				// Clear pushed key flag:
				p.pushed_key = 0;
			}

			nImgs++;
			if (nImgs>10)
			{
				p.Hz = nImgs / tictac.Tac();
				nImgs=0;
				tictac.Tic();
			}
		}
	}
	catch(std::exception &e)
	{
		cout << "Exception in Kinect thread: " << e.what() << endl;
		p.quit = true;
	}
}

// ------------------------------------------------------
//				Test_Kinect
// ------------------------------------------------------
void Test_Kinect()
{
	// Launch grabbing thread:
	// --------------------------------------------------------
	TThreadParam thrPar;
	mrpt::system::TThreadHandle thHandle= mrpt::system::createThreadRef(thread_grabbing ,thrPar);

	// Wait until data stream starts so we can say for sure the sensor has been initialized OK:
	cout << "Waiting for sensor initialization...\n";
	do {
		CObservation3DRangeScanPtr possiblyNewObs = thrPar.new_obs.get();
		if (possiblyNewObs && possiblyNewObs->timestamp!=INVALID_TIMESTAMP)
				break;
		else 	mrpt::system::sleep(10);
	} while (!thrPar.quit);

	// Check error condition:
	if (thrPar.quit) return;


	// Feature tracking variables:
	CFeatureList	trackedFeats;
	unsigned int	step_num = 0;

	bool  SHOW_FEAT_IDS = true;
	bool  SHOW_RESPONSES = true;

	CGenericFeatureTrackerAutoPtr  tracker;

	// "CFeatureTracker_KL" is by far the most robust implementation for now:
	tracker = CGenericFeatureTrackerAutoPtr( new CFeatureTracker_KL );
	tracker->enableTimeLogger(true); // Do time profiling.

	// Set of parameters common to any tracker implementation:
	// To see all the existing params and documentation, see mrpt::vision::CGenericFeatureTracker
	//  http://reference.mrpt.org/devel/structmrpt_1_1vision_1_1_c_generic_feature_tracker.html
	tracker->extra_params["add_new_features"]             = 1;   // track, AND ALSO, add new features
	tracker->extra_params["add_new_feat_min_separation"]  = 25;
	tracker->extra_params["add_new_feat_max_features"]    = 150;
	tracker->extra_params["add_new_feat_patch_size"]      = 21;

	tracker->extra_params["minimum_KLT_response_to_add"]      = 40;
	tracker->extra_params["check_KLT_response_every"]	= 5;	// Re-check the KLT-response to assure features are in good points.
	tracker->extra_params["minimum_KLT_response"]	= 25;	// Re-check the KLT-response to assure features are in good points.

	tracker->extra_params["update_patches_every"]		= 0;  // Update patches

	// Specific params for "CFeatureTracker_KL"
	tracker->extra_params["window_width"]  = 25;
	tracker->extra_params["window_height"] = 25;


	// Global points map:
	CColouredPointsMap  globalPtsMap;
	globalPtsMap.colorScheme.scheme = CColouredPointsMap::cmFromIntensityImage;  // Take points color from RGB+D observations
	// globalPtsMap.colorScheme.scheme = CColouredPointsMap::cmFromHeightRelativeToSensorGray;


	// Create window and prepare OpenGL object in the scene:
	// --------------------------------------------------------
	mrpt::gui::CDisplayWindow3D  win3D("kinect-3d-slam 3D view",800,600);

	win3D.setCameraAzimuthDeg(140);
	win3D.setCameraElevationDeg(20);
	win3D.setCameraZoom(8.0);
	win3D.setFOV(90);
	win3D.setCameraPointingToPoint(2.5,0,0);

	mrpt::opengl::CPointCloudColouredPtr gl_points = mrpt::opengl::CPointCloudColoured::Create();
	gl_points->setPointSize(2.5);

	mrpt::opengl::CSetOfObjectsPtr gl_curFeats = mrpt::opengl::CSetOfObjects::Create();
	mrpt::opengl::CSetOfObjectsPtr gl_keyframes = mrpt::opengl::CSetOfObjects::Create();

	mrpt::opengl::CPointCloudColouredPtr gl_points_map = mrpt::opengl::CPointCloudColoured::Create();
	gl_points_map->setPointSize(2.0);

	const double aspect_ratio =  480.0 / 640.0; // kinect.getRowCount() / double( kinect.getColCount() );

	mrpt::opengl::CSetOfObjectsPtr gl_cur_cam_corner = mrpt::opengl::stock_objects::CornerXYZSimple(0.4f,4);

	opengl::COpenGLViewportPtr viewInt;
	{
		mrpt::opengl::COpenGLScenePtr &scene = win3D.get3DSceneAndLock();

		// Create the Opengl object for the point cloud:
		scene->insert( gl_points_map );
		scene->insert( gl_points );
		scene->insert( gl_curFeats );
		scene->insert( gl_keyframes );
		scene->insert( mrpt::opengl::CGridPlaneXY::Create() );

		scene->insert( gl_cur_cam_corner );

		const int VW_WIDTH = 350;	// Size of the viewport into the window, in pixel units.
		const int VW_HEIGHT = aspect_ratio*VW_WIDTH;

		// Create the Opengl objects for the planar images each in a separate viewport:
		viewInt = scene->createViewport("view2d_int");
		viewInt->setViewportPosition(2,2, VW_WIDTH,VW_HEIGHT);
		viewInt->setTransparent(true);

		win3D.unlockAccess3DScene();
		win3D.repaint();
	}



	CImage	previous_image;

	map<TFeatureID, TPoint3D>   lastVisibleFeats;
	std::vector<TPose3D>        camera_key_frames_path;  // The 6D path of the Kinect camera.
	CPose3D                     currentCamPose_wrt_last; // wrt last pose in "camera_key_frames_path"

	bool gl_keyframes_must_refresh = true;  // Need to update gl_keyframes from camera_key_frames_path??
	CObservation3DRangeScanPtr  last_obs;
	string str_status, str_status2;

	while (win3D.isOpen() && !thrPar.quit)
	{
		CObservation3DRangeScanPtr possiblyNewObs = thrPar.new_obs.get();
		if (possiblyNewObs && possiblyNewObs->timestamp!=INVALID_TIMESTAMP &&
			(!last_obs  || possiblyNewObs->timestamp!=last_obs->timestamp ) )
		{
			// It IS a new observation:
			last_obs = possiblyNewObs;

			// Feature tracking -------------------------------------------
			ASSERT_(last_obs->hasIntensityImage)

			CImage theImg;  // The grabbed image:
			theImg = last_obs->intensityImage;

			// Do tracking:
			if (step_num>1)  // we need "previous_image" to be valid.
			{
				tracker->trackFeatures(previous_image, theImg, trackedFeats);

				// Remove those now out of the image plane:
				CFeatureList::iterator itFeat = trackedFeats.begin();
				while (itFeat!=trackedFeats.end())
				{
					const TFeatureTrackStatus status = (*itFeat)->track_status;
					bool eras = (status_TRACKED!=status && status_IDLE!=status);
					if (!eras)
					{
						// Also, check if it's too close to the image border:
						const float x= (*itFeat)->x;
						const float y= (*itFeat)->y;
						static const float MIN_DIST_MARGIN_TO_STOP_TRACKING = 10;
						if (x<MIN_DIST_MARGIN_TO_STOP_TRACKING  || y<MIN_DIST_MARGIN_TO_STOP_TRACKING ||
							x>(last_obs->cameraParamsIntensity.ncols-MIN_DIST_MARGIN_TO_STOP_TRACKING) ||
							y>(last_obs->cameraParamsIntensity.nrows-MIN_DIST_MARGIN_TO_STOP_TRACKING))
						{
							eras = true;
						}
					}
					if (eras)	// Erase or keep?
						itFeat = trackedFeats.erase(itFeat);
					else ++itFeat;
				}
			}

			// Create list of 3D features in space, wrt current camera pose:
			// --------------------------------------------------------------------
			map<TFeatureID, TPoint3D>  curVisibleFeats;
			for (CFeatureList::iterator itFeat = trackedFeats.begin();itFeat!=trackedFeats.end();++itFeat)
			{
				// Pixel coordinates in the intensity image:
				const int int_x= (*itFeat)->x;
				const int int_y= (*itFeat)->y;

				// Convert to pixel coords in the range image:
				//  APPROXIMATION: Assume coordinates are equal (that's not exact!!)
				const int x = int_x;
				const int y = int_y;

				// Does this (x,y) have valid range data?
				const float d = last_obs->rangeImage(y,x);
				if (d>0.05 && d<10.0)
				{
					ASSERT_( size_t(last_obs->rangeImage.cols()*last_obs->rangeImage.rows()) == last_obs->points3D_x.size() )
					const size_t nPt = last_obs->rangeImage.cols() * y + x;
					curVisibleFeats[(*itFeat)->ID] = TPoint3D( last_obs->points3D_x[nPt],last_obs->points3D_y[nPt],last_obs->points3D_z[nPt] );
				}
			}

			// Load local points map from 3D points + color:
			CColouredPointsMap localPntsMap;
			localPntsMap.colorScheme.scheme = CColouredPointsMap::cmFromIntensityImage;
			localPntsMap.loadFromRangeScan(*last_obs);

			// Estimate our current camera pose from feature2feature matching:
			// --------------------------------------------------------------------
			if (!lastVisibleFeats.empty())
			{
				TMatchingPairList  corrs; // pairs of correspondences

				for (map<TFeatureID, TPoint3D>::const_iterator itCur= curVisibleFeats.begin();itCur!=curVisibleFeats.end();++itCur)
				{
					map<TFeatureID, TPoint3D>::const_iterator itFound = lastVisibleFeats.find(itCur->first);
					if (itFound!=lastVisibleFeats.end())
					{
						corrs.push_back( TMatchingPair(
							itFound->first,
							itCur->first,
							itFound->second.x,itFound->second.y,itFound->second.z,
							itCur->second.x,itCur->second.y,itCur->second.z
							));
					}
				}

				if (corrs.size()>=3)
				{
					// Find matchings:
					mrpt::tfest::TSE3RobustParams params;
					params.ransac_minSetSize = 3;
					params.ransac_maxSetSizePct = 6.0 / corrs.size();

					mrpt::tfest::TSE3RobustResult results;
					bool register_ok = false;
					try {
						mrpt::tfest::se3_l2_robust(corrs, params, results);
						register_ok = true;
					} catch (std::exception &) {
						/* Cannot find a minimum number of matches, inconsistent parameters due to very reduced numberof matches,etc. */
					}

					const CPose3D relativePose = results.transformation;

					str_status = mrpt::format("%d corrs | inliers: %d | rel.pose: %s ", int(corrs.size()), int(results.inliers_idx.size()), relativePose.asString().c_str() );
					str_status2 = string( results.inliers_idx.size()==0 ? "LOST! Please, press 'r' to restart" : "" );

					if (register_ok && std::abs(results.scale-1.0)<0.1)
					{
						// Seems a good match:
						if ( ( relativePose.norm() > KEYFRAMES_MIN_DISTANCE ||
							std::abs(relativePose.yaw())>KEYFRAMES_MIN_ANG ||
							std::abs(relativePose.pitch())>KEYFRAMES_MIN_ANG ||
							std::abs(relativePose.roll())>KEYFRAMES_MIN_ANG ))
						{
							// Accept this as a new key-frame pose ------------
							// Append new global pose of this key-frame:

							const CPose3D new_keyframe_global = CPose3D(*camera_key_frames_path.rbegin()) + relativePose;

							camera_key_frames_path.push_back( TPose3D(new_keyframe_global) );

							gl_keyframes_must_refresh = true;

							currentCamPose_wrt_last = CPose3D();  // It's (0,0,0) since the last key-frame is the current pose!
							lastVisibleFeats = curVisibleFeats;

							cout << "Adding new key-frame: pose="<< new_keyframe_global << endl;

							// Update global map: append another map at a given position:
							globalPtsMap.insertObservation(last_obs.pointer(),&new_keyframe_global);
							win3D.get3DSceneAndLock();
								gl_points_map->loadFromPointsMap(&globalPtsMap);
							win3D.unlockAccess3DScene();
						}
						else
						{
							currentCamPose_wrt_last = relativePose;
							//cout << "cur pose: " << currentCamPose_wrt_last << endl;
						}
					}
				}
			}

			if (camera_key_frames_path.empty() || lastVisibleFeats.empty())
			{
				// First iteration:
				camera_key_frames_path.clear();
				camera_key_frames_path.push_back(TPose3D(0,0,0,0,0,0));
				gl_keyframes_must_refresh = true;
				lastVisibleFeats = curVisibleFeats;

				// Update global map:
				globalPtsMap.clear();
				globalPtsMap.insertObservation(last_obs.pointer() );

				win3D.get3DSceneAndLock();
					gl_points_map->loadFromPointsMap(&globalPtsMap);
				win3D.unlockAccess3DScene();
			}


			// Save the image for the next step:
			previous_image = theImg;

			// Draw marks on the RGB image:
			theImg.selectTextFont("10x20");
			{	// Tracked feats:
				theImg.drawFeatures(trackedFeats, TColor(0,0,255), SHOW_FEAT_IDS, SHOW_RESPONSES);
				theImg.textOut(3,22,format("# feats: %u", (unsigned int)trackedFeats.size()  ),TColor(200,20,20) );
			}

			// Update visualization ---------------------------------------

			// Show intensity image
			win3D.get3DSceneAndLock();
				viewInt->setImageView(theImg);
			win3D.unlockAccess3DScene();

			// Show 3D points & current visible feats, at the current camera 3D pose "currentCamPose_wrt_last"
			// ---------------------------------------------------------------------
			if (last_obs->hasPoints3D )
			{
				const CPose3D curGlobalPose = CPose3D(*camera_key_frames_path.rbegin()) + currentCamPose_wrt_last;
				win3D.get3DSceneAndLock();
					// All 3D points:
					gl_points->loadFromPointsMap(&localPntsMap);
					gl_points->setPose(curGlobalPose);
					gl_cur_cam_corner->setPose(curGlobalPose);

					// Current visual landmarks:
					gl_curFeats->clear();
					for (map<TFeatureID, TPoint3D>::const_iterator it=curVisibleFeats.begin();it!=curVisibleFeats.end();++it)
					{
						static double D = 0.02;
						mrpt::opengl::CBoxPtr box = mrpt::opengl::CBox::Create(TPoint3D(-D,-D,-D),TPoint3D(D,D,D));
						box->setWireframe(true);
						box->setName(format("%d",int(it->first)));
						box->enableShowName(true);
						box->setLocation(it->second);
						gl_curFeats->insert(box);
					}
					gl_curFeats->setPose(curGlobalPose);

				win3D.unlockAccess3DScene();
				win3D.repaint();
			}


			win3D.get3DSceneAndLock();
				win3D.addTextMessage(-100,-20, format("%.02f Hz", thrPar.Hz ), TColorf(0,1,1), 100, MRPT_GLUT_BITMAP_HELVETICA_18 );
			win3D.unlockAccess3DScene();

			win3D.repaint();

			step_num++;

		} // end update visualization:

		if (gl_keyframes_must_refresh)
		{
			gl_keyframes_must_refresh = false;
			//cout << "Updating gl_keyframes with " << camera_key_frames_path.size() << " frames.\n";
			win3D.get3DSceneAndLock();
				gl_keyframes->clear();
				for (size_t i=0;i<camera_key_frames_path.size();i++)
				{
					CSetOfObjectsPtr obj = mrpt::opengl::stock_objects::CornerXYZSimple(0.3f,3);
					obj->setPose( camera_key_frames_path[i]);
					gl_keyframes->insert(obj);
				}
			win3D.unlockAccess3DScene();
		}


		// Process possible keyboard commands:
		// --------------------------------------
		if (win3D.keyHit() && thrPar.pushed_key==0)
		{
			const int key = tolower( win3D.getPushedKey() );

			switch(key)
			{
				// Some of the keys are processed in this thread:
				case 'r':
					lastVisibleFeats.clear();
					camera_key_frames_path.clear();
					gl_keyframes_must_refresh = true;
					globalPtsMap.clear();
					win3D.get3DSceneAndLock();
						gl_points_map->loadFromPointsMap(&globalPtsMap);
					win3D.unlockAccess3DScene();

					break;
				case 's':
					{
						const std::string s = "point_cloud.txt";
						cout << "Dumping 3D point-cloud to: " << s << endl;
						globalPtsMap.save3D_to_text_file(s);
						break;
					}
				case 'o':
					win3D.setCameraZoom( win3D.getCameraZoom() * 1.2 );
					win3D.repaint();
					break;
				case 'i':
					win3D.setCameraZoom( win3D.getCameraZoom() / 1.2 );
					win3D.repaint();
					break;
				// ...and the rest in the kinect thread:
				default:
					thrPar.pushed_key = key;
					break;
			};
		}

		win3D.get3DSceneAndLock();
		win3D.addTextMessage(2,-30,format("'s':save point cloud, 'r': reset, 'o'/'i': zoom out/in, mouse: orbit 3D, ESC: quit"),
				TColorf(1,1,1), 110, MRPT_GLUT_BITMAP_HELVETICA_12 );
		win3D.addTextMessage(2,-50,str_status,  TColorf(1,1,1), 111, MRPT_GLUT_BITMAP_HELVETICA_12 );
		win3D.addTextMessage(2,-70,str_status2, TColorf(1,1,1), 112, MRPT_GLUT_BITMAP_HELVETICA_18 );
		win3D.unlockAccess3DScene();

		mrpt::system::sleep(1);
	}


	cout << "Waiting for grabbing thread to exit...\n";
	thrPar.quit = true;
	mrpt::system::joinThread(thHandle);
	cout << "Bye!\n";
}


int main(int argc, char **argv)
{
	try
	{
		Test_Kinect();

		mrpt::system::sleep(50);
		return 0;

	} catch (std::exception &e)
	{
		std::cout << "EXCEPCION: " << e.what() << std::endl;
		return -1;
	}
	catch (...)
	{
		printf("Another exception!!");
		return -1;
	}

}
