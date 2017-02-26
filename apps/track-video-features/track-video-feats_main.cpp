/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

/*---------------------------------------------------------------
    DEMO: track-video-features
    Started by: Jose Luis Blanco Claraco <joseluisblancoc@gmail.com>
                @ Jun 2010

    Purpose: Illustrate MRPT classes and functions for live
             video features detection and tracking, as well
             as real-time visualization of all that stuff.
  ---------------------------------------------------------------*/

#include <mrpt/vision/tracking.h>
#include <mrpt/vision/CVideoFileWriter.h>
#include <mrpt/system/filesystem.h>
#include <mrpt/gui/CDisplayWindow3D.h>		// For visualization windows
#include <mrpt/hwdrivers/CCameraSensor.h>	// For capture of video from videos/cameras
#include <mrpt/obs/CRawlog.h>
#include <mrpt/system/os.h>

using namespace std;
using namespace mrpt;
using namespace mrpt::hwdrivers;
using namespace mrpt::utils;
using namespace mrpt::obs;
using namespace mrpt::system;
using namespace mrpt::vision;
using namespace mrpt::poses;

mrpt::gui::CDisplayWindow3DPtr win;  // This is global such as an exception within the main program do not abruptly closes the window

// ------------------------------------------------------
//		DoTrackingDemo
// ------------------------------------------------------
int DoTrackingDemo(CCameraSensorPtr  cam, bool  DO_SAVE_VIDEO)
{
	win = mrpt::gui::CDisplayWindow3D::Create("Tracked features",800,600);

	mrpt::vision::CVideoFileWriter  vidWritter;

	bool 		hasResolution = false;
	TCamera		cameraParams; // For now, will only hold the image resolution on the arrive of the first frame.

	TSimpleFeatureList  trackedFeats;

	unsigned int	step_num = 0;

	bool  SHOW_FEAT_IDS = true;
	bool  SHOW_RESPONSES = true;
	bool  SHOW_FEAT_TRACKS = true;


	const double SAVE_VIDEO_FPS = 30; // If DO_SAVE_VIDEO=true, the FPS of the video file
	const char*  SAVE_VIDEO_CODEC = "XVID"; // "XVID", "PIM1", "MJPG"

	bool  DO_HIST_EQUALIZE_IN_GRAYSCALE = false;
	string VIDEO_OUTPUT_FILE = "./tracking_video.avi";

	const double MAX_FPS = 5000; // 5.0;  // Hz (to slow down visualization).

	CGenericFeatureTrackerAutoPtr  tracker;

	// "CFeatureTracker_KL" is by far the most robust implementation for now:
	tracker = CGenericFeatureTrackerAutoPtr( new CFeatureTracker_KL );

	tracker->enableTimeLogger(true); // Do time profiling.

	// Set of parameters common to any tracker implementation:
	// -------------------------------------------------------------
	// To see all the existing params and documentation, see mrpt::vision::CGenericFeatureTracker
	tracker->extra_params["remove_lost_features"]         = 1;   // automatically remove out-of-image and badly tracked features

	tracker->extra_params["add_new_features"]             = 1;   // track, AND ALSO, add new features
	tracker->extra_params["add_new_feat_min_separation"]  = 32;
	tracker->extra_params["minimum_KLT_response_to_add"]  = 10;
	tracker->extra_params["add_new_feat_max_features"]    = 350;
	tracker->extra_params["add_new_feat_patch_size"]      = 11;

	tracker->extra_params["update_patches_every"]		= 0;  // Don't update patches.

	tracker->extra_params["check_KLT_response_every"]	= 5;	// Re-check the KLT-response to assure features are in good points.
	tracker->extra_params["minimum_KLT_response"]	    = 5;

	// Specific params for "CFeatureTracker_KL"
	// ------------------------------------------------------
	tracker->extra_params["window_width"]  = 5;
	tracker->extra_params["window_height"] = 5;
	//tracker->extra_params["LK_levels"] = 3;
	//tracker->extra_params["LK_max_iters"] = 10;
	//tracker->extra_params["LK_epsilon"] = 0.1;
	//tracker->extra_params["LK_max_tracking_error"] = 150;


	// --------------------------------
	// The main loop
	// --------------------------------
	CImage		previous_image;

	TSequenceFeatureObservations    feat_track_history;
	bool							save_tracked_history = true; // Dump feat_track_history to a file at the end

	TCameraPoseID 					curCamPoseId = 0;

	cout << endl << "TO END THE PROGRAM: Close the window.\n";

	mrpt::opengl::COpenGLViewportPtr gl_view;
	{
		mrpt::opengl::COpenGLScenePtr scene = win->get3DSceneAndLock();
		gl_view = scene->getViewport("main");
		win->unlockAccess3DScene();
	}

	// Aux data for drawing the recent track of features:
	static const size_t FEATS_TRACK_LEN = 10;
	std::map<TFeatureID,std::list<TPixelCoord> >  feat_tracks;

	// infinite loop, until we close the win:
	while( win->isOpen() )
	{
		CObservationPtr obs;
		try
		{
			obs= cam->getNextFrame();
		}
		catch (CExceptionEOF &)
		{	// End of a rawlog file.
			break;
		}

		if (!obs)
		{
			cerr << "*Warning* getNextFrame() returned NULL!\n";
			mrpt::system::sleep(50);
			continue;
		}

		CImage theImg;  // The grabbed image:

		if (IS_CLASS(obs,CObservationImage))
		{
			CObservationImagePtr o = CObservationImagePtr(obs);
			theImg.copyFastFrom(o->image);
		}
		else if (IS_CLASS(obs,CObservationStereoImages))
		{
			CObservationStereoImagesPtr o = CObservationStereoImagesPtr(obs);
			theImg.copyFastFrom(o->imageLeft);
		}
		else if (IS_CLASS(obs,CObservation3DRangeScan))
		{
			CObservation3DRangeScanPtr o = CObservation3DRangeScanPtr(obs);
			if (o->hasIntensityImage)
				theImg.copyFastFrom(o->intensityImage);
		}
		else
		{
			continue; // Silently ignore non-image observations.
		}

		// Make sure the image is loaded (for the case it came from a rawlog file)
		if (theImg.isExternallyStored())
			theImg.loadFromFile( theImg.getExternalStorageFileAbsolutePath());

		// Take the resolution upon first valid frame.
		if (!hasResolution)
		{
			hasResolution = true;
			// cameraParams.scaleToResolution()...
			cameraParams.ncols = theImg.getWidth();
			cameraParams.nrows = theImg.getHeight();
		}

		// Do tracking:
		if (step_num>1)  // we need "previous_image" to be valid.
		{
			// This single call makes: detection, tracking, recalculation of KLT_response, etc.
			tracker->trackFeatures(previous_image, theImg, trackedFeats);
		}

		// Save the image for the next step:
		previous_image = theImg;

		// Save history of feature observations:
		tracker->getProfiler().enter("Save history");

		for (size_t i=0;i<trackedFeats.size();++i)
		{
			TSimpleFeature &f = trackedFeats[i];

			const TPixelCoordf pxRaw(f.pt.x,f.pt.y);
			TPixelCoordf  pxUndist;
			//mrpt::vision::pinhole::undistort_point(pxRaw,pxUndist, cameraParams);
			pxUndist = pxRaw;

			feat_track_history.push_back( TFeatureObservation(f.ID,curCamPoseId, pxUndist ) );
		}
		curCamPoseId++;

		tracker->getProfiler().leave("Save history");

		// now that we're done with the image, we can directly write onto it
		//  for the display
		// ----------------------------------------------------------------
		if (DO_HIST_EQUALIZE_IN_GRAYSCALE && !theImg.isColor())
			theImg.equalizeHistInPlace();
		// Convert to color so we can draw color marks, etc.
		theImg.colorImageInPlace();

		double extra_tim_to_wait=0;

		{	// FPS:
			static CTicTac tictac;
			const double T = tictac.Tac();
			tictac.Tic();
			const double fps = 1.0/(std::max(1e-5,T));
			//theImg.filledRectangle(1,1,175,25,TColor(0,0,0));

			const int current_adapt_thres = tracker->getDetectorAdaptiveThreshold();

			theImg.selectTextFont("6x13B");
			theImg.textOut(3,3,format("FPS: %.03f Hz", fps ),TColor(200,200,0) );
			theImg.textOut(3,22,format("# feats: %u - Adaptive threshold: %i", (unsigned int)trackedFeats.size(), current_adapt_thres ),TColor(200,200,0) );

			theImg.textOut(3,41,
				format("# raw feats: %u - Removed: %u",
					(unsigned int)tracker->last_execution_extra_info.raw_FAST_feats_detected,
					(unsigned int)tracker->last_execution_extra_info.num_deleted_feats ),
					TColor(200,200,0) );

			extra_tim_to_wait = 1.0/MAX_FPS - 1.0/fps;
		}

		// Draw feature tracks
		if (SHOW_FEAT_TRACKS)
		{
			// Update new feature coords:
			tracker->getProfiler().enter("drawFeatureTracks");

			std::set<TFeatureID> observed_IDs;

			for (size_t i=0;i<trackedFeats.size();++i)
			{
				const TSimpleFeature &ft = trackedFeats[i];
				std::list<TPixelCoord> & seq = feat_tracks[ft.ID];

				observed_IDs.insert(ft.ID);

				if (seq.size()>=FEATS_TRACK_LEN) seq.erase(seq.begin());
				seq.push_back(ft.pt);

				// Draw:
				if (seq.size()>1)
				{
					const std::list<TPixelCoord>::const_iterator it_end = seq.end();

					std::list<TPixelCoord>::const_iterator it      = seq.begin();
					std::list<TPixelCoord>::const_iterator it_prev = it++;

					for (;it!=it_end;++it)
					{
						theImg.line(it_prev->x,it_prev->y,it->x,it->y, TColor(190,190,190) );
						it_prev = it;
					}
				}
			}

			tracker->getProfiler().leave("drawFeatureTracks");

			// Purge old data:
			for (std::map<TFeatureID,std::list<TPixelCoord> >::iterator it=feat_tracks.begin();it!=feat_tracks.end(); )
			{
				if (observed_IDs.find(it->first)==observed_IDs.end())
				{
					std::map<TFeatureID,std::list<TPixelCoord> >::iterator next_it = it;
					next_it++;
					feat_tracks.erase(it);
					it = next_it;
				}
				else ++it;
			}
		}

		// Draw Tracked feats:
		{
			theImg.selectTextFont("5x7");
			tracker->getProfiler().enter("drawFeatures");
			theImg.drawFeatures(trackedFeats, TColor(0,0,255), SHOW_FEAT_IDS, SHOW_RESPONSES);
			tracker->getProfiler().leave("drawFeatures");
		}


		// Update window:
		win->get3DSceneAndLock();
			gl_view->setImageView(theImg);
		win->unlockAccess3DScene();
		win->repaint();

		// Save debug output video:
		// ----------------------------------
		if (DO_SAVE_VIDEO)
		{
			static bool first = true;
			if (first)
			{
				first=false;
				if (vidWritter.open(
						VIDEO_OUTPUT_FILE,
						SAVE_VIDEO_FPS /* fps */, theImg.getSize(),
						SAVE_VIDEO_CODEC,
						true /* force color video */ ) )
				{
					cout << "[track-video] Saving tracking video to: " << VIDEO_OUTPUT_FILE << endl;
				}
				else
					cerr << "ERROR: Trying to create output video: " << VIDEO_OUTPUT_FILE << endl;
			}

			vidWritter << theImg;
		}

		if (extra_tim_to_wait>0)
			mrpt::system::sleep(1000*extra_tim_to_wait);

		step_num++;
	} // end infinite loop

	// Save tracked feats:
	if (save_tracked_history)
	{
		cout << "Saving tracked features to: tracked_feats.txt..."; cout.flush();
		feat_track_history.saveToTextFile("./tracked_feats.txt");
		cout << "Done!\n"; cout.flush();

#if 0
		// SBA:
		cout << "Saving cams.txt & pts.txt files in SBA library format..."; cout.flush();

		feat_track_history.removeFewObservedFeatures(3);
		feat_track_history.decimateCameraFrames(20);
		feat_track_history.compressIDs();

		TLandmarkLocationsVec  locs;
		TFramePosesVec         cams;
		feat_track_history.saveAsSBAFiles(locs,"pts.txt", cams, "cams.txt");


		cout << "Done!\n"; cout.flush();
#endif
	}


	return 0; // End ok.
}

void showUsage(char *cmd);

// ------------------------------------------------------
//						MAIN
// ------------------------------------------------------
int main(int argc, char **argv)
{
	try
	{
		printf(" track-video-features - Part of MRPT\n");
		printf(" MRPT C++ Library: %s - Sources timestamp: %s\n", mrpt::system::MRPT_getVersion().c_str(), mrpt::system::MRPT_getCompilationDate().c_str());
		printf("-------------------------------------------------------------------\n");

		// The video source:
		CCameraSensorPtr  cam;

		// process cmd line arguments?

		if (argc<1 || argc>3)
		{
			cerr << "Incorrect number of arguments.\n";
			showUsage(argv[0]);
			return -1;
		}

		const bool last_arg_is_save_video = !strcmp("--save-video",argv[argc-1]);
		if (last_arg_is_save_video)
			argc--; // Discard last argument

		if (argc==2)
		{
			if (!strcmp(argv[1],"--help"))
			{
				showUsage(argv[0]);
				return 0;
			}
			if (!mrpt::system::fileExists(argv[1]))
			{
				cerr << "File does not exist: " << argv[1] << endl;
				return -1;
			}

			const string fil = string(argv[1]);
			const string ext = mrpt::system::lowerCase(mrpt::system::extractFileExtension(fil,true));

			if (ext=="rawlog")
			{
				// It's a rawlog:
				cout << "Interpreting '" << fil << "' as a rawlog file...\n";

				cam = CCameraSensorPtr(new CCameraSensor);

				CConfigFileMemory  cfg;
				cfg.write("CONFIG","grabber_type","rawlog");
				cfg.write("CONFIG","rawlog_file", fil );

				// For delayed-load images:
				CImage::IMAGES_PATH_BASE = CRawlog::detectImagesDirectory(fil);

				cam->loadConfig(cfg,"CONFIG");
				cam->initialize();	// This will raise an exception if neccesary
			}
			else
			{
				// Assume it's a video:
				cout << "Interpreting '" << fil << "' as a video file...\n";

				cam = CCameraSensorPtr(new CCameraSensor);

				CConfigFileMemory  cfg;
				cfg.write("CONFIG","grabber_type","ffmpeg");
				cfg.write("CONFIG","ffmpeg_url", fil );

				cam->loadConfig(cfg,"CONFIG");
				cam->initialize();	// This will raise an exception if neccesary
			}
		}




		if (!cam)
		{
			cout << "You didn't specify any video source in the command line.\n"
					"(You can run with --help to see usage).\n"
					"Showing a GUI window to select the video source...\n";
			// If no camera opened so far, ask the user for one:

			cam = mrpt::hwdrivers::prepareVideoSourceFromUserSelection();
			if (!cam)
			{
				cerr << "No images source was correctly initialized! Exiting.\n";
				return -1;
			}
		}

		// do it:
		const int ret = DoTrackingDemo(cam, last_arg_is_save_video);

		win.clear();
		mrpt::system::sleep(150); // give time to close GUI threads
		return ret;
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


void showUsage(char *cmd)
{
	cout <<
		"Usage:\n"
		"  " << cmd << " [--save-video]                -> Ask the user for video source.\n"
		"  " << cmd << " dataset.rawlog [--save-video]  -> Use a rawlog file.\n"
		"  " << cmd << " video.{avi,mpg}[--save-video]  -> Use a video file.\n"
		"  " << cmd << " --help          -> Show this information.\n"
		"  " << cmd << " If added --save-video, an video file will be created with results.\n"
		"\n";
}

