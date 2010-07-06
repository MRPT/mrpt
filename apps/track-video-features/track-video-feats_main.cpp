/* +---------------------------------------------------------------------------+
   |          The Mobile Robot Programming Toolkit (MRPT) C++ library          |
   |                                                                           |
   |                   http://mrpt.sourceforge.net/                            |
   |                                                                           |
   |   Copyright (C) 2005-2010  University of Malaga                           |
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

using namespace std;
using namespace mrpt;
using namespace mrpt::hwdrivers;
using namespace mrpt::utils;
using namespace mrpt::slam;
using namespace mrpt::vision;
using namespace mrpt::poses;

// ------------------------------------------------------
//		DoTrackingDemo
// ------------------------------------------------------
int DoTrackingDemo(CCameraSensorPtr  cam)
{
	mrpt::gui::CDisplayWindowPtr win;

	win = mrpt::gui::CDisplayWindow::Create("Tracked features");

	CTimeLogger  timlog;

	bool 		hasResolution = false;
	TCamera		cameraParams; // For now, will only hold the image resolution on the arrive of the first frame.

	CFeatureList	trackedFeats;
	unsigned int	step_num = 0;

	mrpt::vision::CFeatureTracker_FAST   tracker;
	//mrpt::vision::CFeatureTracker_KL   tracker;

	tracker.extra_params["window_width"]  = 7;
	tracker.extra_params["window_height"] = 7;


	unsigned int 	NUM_FEATS_TO_DETECT = 100;

	CImage		previous_image; // the tracking

	while( win->isOpen() ) // infinite loop, until we close the win
	{
		CObservationPtr obs = cam->getNextFrame();
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
			cameraParams.ncols = theImg.getWidth();
			cameraParams.nrows = theImg.getHeight();
		}

		// Do tracking:
		if (!trackedFeats.empty())
		{
			timlog.enter("TRACKING");

			tracker.trackFeatures(previous_image, theImg, trackedFeats);

			timlog.leave("TRACKING");

			// Remove those now out of the image plane:
			CFeatureList::iterator itFeat = trackedFeats.begin();
			while (itFeat!=trackedFeats.end())
			{
				bool eras = (status_TRACKED!=(*itFeat)->track_status);
				if (!eras)
				{
					// Also, check if it's too close to the image border:
					const float x= (*itFeat)->x;
					const float y= (*itFeat)->y;
					static const float MIN_DIST_MARGIN_TO_STOP_TRACKING = 20;
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

		// At the beginning, look for new features:
		if (trackedFeats.empty() && step_num==5) // wait a bit to detect
		{
			//cout << "Detecting features...\n";
			timlog.enter("DETECT");

			CFeatureExtraction  FE;
			FE.options.featsType = featFAST;

			FE.options.patchSize = 11;
			FE.options.FASTOptions.threshold = 20;
			FE.options.FASTOptions.min_distance = 20;

			FE.detectFeatures(theImg, trackedFeats, 0 /* first ID */, NUM_FEATS_TO_DETECT /* # feats */ );

			timlog.leave("DETECT");

			ASSERT_(trackedFeats.size()>3)
		}


		// Save the image for the next step:
		previous_image = theImg;

		// now that we're done with the image, we can directly write onto it
		//  for the display
		// ----------------------------------------------------------------
		{	// FPS:
			static CTicTac tictac;
			const double T = tictac.Tac();
			tictac.Tic();
			const double fps = 1.0/(std::max(1e-5,T));
			theImg.filledRectangle(1,1,175,25,TColor(0,0,0));
			theImg.textOut(3,3,format("FPS: %.03f Hz", fps ),TColor(20,20,255) );
		}
		{	// Tracked feats:
			for (CFeatureList::const_iterator it=trackedFeats.begin();it!=trackedFeats.end();++it)
			{
				const float x = (*it)->x;
				const float y = (*it)->y;
				theImg.cross(x,y,TColor(0,0,255),'+',5);
			}
			theImg.filledRectangle(1,25,175,50,TColor(0,0,0));
			theImg.textOut(3,28,format("# feats: %u", (unsigned int)trackedFeats.size()  ),TColor(20,20,255) );
		}

		win->showImage(theImg);

		step_num++;
	} // end infinite loop

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
