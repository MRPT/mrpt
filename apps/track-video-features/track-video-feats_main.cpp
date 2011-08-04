/* +---------------------------------------------------------------------------+
   |          The Mobile Robot Programming Toolkit (MRPT) C++ library          |
   |                                                                           |
   |                       http://www.mrpt.org/                                |
   |                                                                           |
   |   Copyright (C) 2005-2011  University of Malaga                           |
   |                                                                           |
   |    This software was written by the Machine Perception and Intelligent    |
   |      Robotics Lab, University of Malaga (Spain).                          |
   |    Contact: Jose-Luis Blanco  <jlblanco@ctima.uma.es>                     |
   |                                                                           |
   |  This file is part of the MRPT project.                                   |
   |                                                                           |
   |     MRPT is free software: you can redistribute it and/or modify          |
   |     it under the terms of the GNU General Public License as published by  |
   |     the Free Software Foundation, either version 3 of the License, or     |
   |     (at your option) any later version.                                   |
   |                                                                           |
   |   MRPT is distributed in the hope that it will be useful,                 |
   |     but WITHOUT ANY WARRANTY; without even the implied warranty of        |
   |     MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         |
   |     GNU General Public License for more details.                          |
   |                                                                           |
   |     You should have received a copy of the GNU General Public License     |
   |     along with MRPT.  If not, see <http://www.gnu.org/licenses/>.         |
   |                                                                           |
   +---------------------------------------------------------------------------+ */

/*---------------------------------------------------------------
    DEMO: track-video-features
    Started by: Jose Luis Blanco Claraco <jlblanco@ctima.uma.es>
                @ Jun 2010

    Purpose: Illustrate MRPT classes and functions for live
             video features detection and tracking, as well
             as real-time visualization of all that stuff.
  ---------------------------------------------------------------*/

#include <mrpt/vision.h> 	// For feature detection, etc.
#include <mrpt/gui.h>		// For visualization windows
#include <mrpt/hwdrivers.h>	// For capture of video from videos/cameras
#include <mrpt/base.h>
#include <mrpt/slam/CRawlog.h>

using namespace std;
using namespace mrpt;
using namespace mrpt::hwdrivers;
using namespace mrpt::utils;
using namespace mrpt::slam;
using namespace mrpt::vision;
using namespace mrpt::poses;

mrpt::gui::CDisplayWindowPtr win;  // This is global such as an exception within the main program do not abruptly closes the window

// ------------------------------------------------------
//		DoTrackingDemo
// ------------------------------------------------------
int DoTrackingDemo(CCameraSensorPtr  cam)
{
	win = mrpt::gui::CDisplayWindow::Create("Tracked features");

	mrpt::vision::CVideoFileWriter  vidWritter;

	bool 		hasResolution = false;
	TCamera		cameraParams; // For now, will only hold the image resolution on the arrive of the first frame.

	CFeatureList	trackedFeats;
	unsigned int	step_num = 0;

	bool  SHOW_FEAT_IDS = true;
	bool  SHOW_RESPONSES = true;

	bool  DO_SAVE_VIDEO = false;
	bool  DO_HIST_EQUALIZE_IN_GRAYSCALE = false;
	string VIDEO_OUTPUT_FILE = "./tracking_video.avi";

	const double MAX_FPS = 5000; // 5.0;  // Hz (to slow down visualization).

	CGenericFeatureTrackerAutoPtr  tracker;

	// "CFeatureTracker_KL" is by far the most robust implementation for now:
	tracker = CGenericFeatureTrackerAutoPtr( new CFeatureTracker_KL );
	//tracker = CGenericFeatureTrackerAutoPtr( new CFeatureTracker_FAST );
	//tracker = CGenericFeatureTrackerAutoPtr( new CFeatureTracker_PatchMatch );

	tracker->enableTimeLogger(true); // Do time profiling.

	// Set of parameters common to any tracker implementation:
	// To see all the existing params and documentation, see mrpt::vision::CGenericFeatureTracker
	tracker->extra_params["add_new_features"]             = 1;   // track, AND ALSO, add new features
	tracker->extra_params["add_new_feat_min_separation"]  = 25;
	tracker->extra_params["add_new_feat_max_features"]    = 150;
	tracker->extra_params["add_new_feat_patch_size"]      = 21;

	tracker->extra_params["update_patches_every"]		= 0;  // Don't update patches.

	tracker->extra_params["check_KLT_response_every"]	= 5;	// Re-check the KLT-response to assure features are in good points.

	// Specific params for "CFeatureTracker_KL"
	tracker->extra_params["window_width"]  = 5;
	tracker->extra_params["window_height"] = 5;



	// --------------------------------
	// The main loop
	// --------------------------------
	CImage		previous_image;

	TSequenceFeatureObservations    feat_track_history;
	bool							save_tracked_history = true; // Dump feat_track_history to a file at the end

	TCameraPoseID 					curCamPoseId = 0;

	cout << endl << "TO END THE PROGRAM: Close the window.\n";

	while( win->isOpen() ) // infinite loop, until we close the win
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
						x>(cameraParams.ncols-MIN_DIST_MARGIN_TO_STOP_TRACKING) ||
						y>(cameraParams.nrows-MIN_DIST_MARGIN_TO_STOP_TRACKING))
					{
						eras = true;
					}
				}
				if (eras)	// Erase or keep?
					itFeat = trackedFeats.erase(itFeat);
				else ++itFeat;
			}
		}

		// Save the image for the next step:
		previous_image = theImg;

		// Save history of feature observations:
		tracker->getProfiler().enter("Save history");

		for (CFeatureList::iterator itFeat = trackedFeats.begin();itFeat!=trackedFeats.end();++itFeat)
		{
			const CFeature &f = **itFeat;

			const TPixelCoordf pxRaw(f.x,f.y);
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

			theImg.textOut(3,22,format("# feats: %u", (unsigned int)trackedFeats.size()  ),TColor(200,200,0) );

			extra_tim_to_wait = 1.0/MAX_FPS - 1.0/fps;
		}
		{	// Tracked feats:
			theImg.selectTextFont("5x7");
			tracker->getProfiler().enter("drawFeatures");
			theImg.drawFeatures(trackedFeats, TColor(0,0,255), SHOW_FEAT_IDS, SHOW_RESPONSES);
			tracker->getProfiler().leave("drawFeatures");
		}

		win->showImage(theImg);

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
						30 /* fps */, theImg.getSize(),
						"XVID",  // PIM1: MPEG1
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
		printf(" MRPT C++ Library: %s - BUILD DATE %s\n", MRPT_getVersion().c_str(), MRPT_getCompilationDate().c_str());
		printf("-------------------------------------------------------------------\n");

		// The video source:
		CCameraSensorPtr  cam;

		// process cmd line arguments?

		if (argc!=1 && argc!=2)
		{
			cerr << "Incorrect number of arguments.\n";
			showUsage(argv[0]);
			return -1;
		}

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
		return DoTrackingDemo(cam);
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
		"  " << cmd << "                 -> Ask the user for video source.\n"
		"  " << cmd << " dataset.rawlog  -> Use a rawlog file.\n"
		"  " << cmd << " video.{avi,mpg} -> Use a video file.\n"
		"  " << cmd << " --help          -> Show this information.\n"
		"\n";
}
