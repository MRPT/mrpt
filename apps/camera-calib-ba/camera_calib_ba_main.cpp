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
    APPLICATION: camera-calib-ba
    AUTHOR: Jose Luis Blanco Claraco <jlblanco@ctima.uma.es>

	A camera calibration algorithm based on automatically tracked
	features, so it doesn't need any special pattern to calibrate a
	camera, just a short sequence of images.
  ---------------------------------------------------------------*/

#include <mrpt/vision.h>
#include <mrpt/gui.h>
#include <mrpt/hwdrivers.h>

using namespace std;
using namespace mrpt;
using namespace mrpt::hwdrivers;
using namespace mrpt::utils;
using namespace mrpt::slam;
using namespace mrpt::vision;
using namespace mrpt::poses;



// ------------------------------------------------------
//		DoCameraCalibBA
// ------------------------------------------------------
int DoCameraCalibBA(CCameraSensorPtr  cam)
{
	mrpt::gui::CDisplayWindowPtr win;
#if MRPT_HAS_WXWIDGETS
	win = mrpt::gui::CDisplayWindow::Create("Tracked features");
#endif

	bool 		hasResolution = false;
	TCamera		cameraParams; // For now, will only hold the image resolution on the arrive of the first frame.

	CFeatureList	trackedFeats;
	unsigned int	step_num = 0;

	unsigned int 	NUM_FRAMES_TO_GO 		= 50;
	unsigned int 	FRAMES_DECIMATION		= 5;  // actual # frames =  NUM_FRAMES_TO_GO / FRAMES_DECIMATION
	unsigned int 	NUM_FEATS_TO_DETECT 	= 100;

	// The list of tracked features:
	vector<map<TFeatureID,TPixelCoordf> >  lstTrackedFeats;


	CImage		previous_image; // the tracking

	for (;;) // infinite loop, until we break it
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
		double average_feats_displacement = 0;

		if (!trackedFeats.empty())
		{
			// Keep a copy of the previous features:
			vector<TPixelCoordf> prevFeats;
			for (size_t i=0;i<trackedFeats.size();i++)
				prevFeats.push_back( TPixelCoordf( trackedFeats[i]->x,trackedFeats[i]->y ) );

			mrpt::vision::trackFeatures(
				previous_image,
				theImg,
				trackedFeats);

			// Take a measure of "how much the camera has moved":
			vector_double featMovesSq;
			for (size_t i=0;i<prevFeats.size();i++)
			{
				if (statusKLT_TRACKED==trackedFeats[i]->KLT_status)
				{
					const double distSq = square( trackedFeats[i]->x-prevFeats[i].x) + square(trackedFeats[i]->y-prevFeats[i].y);
					featMovesSq.push_back(distSq);
				}
			}

			average_feats_displacement = std::sqrt(featMovesSq.mean());

			// Remove those now out of the image plane:
			CFeatureList::iterator itFeat = trackedFeats.begin();
			while (itFeat!=trackedFeats.end())
			{
				bool eras = (statusKLT_TRACKED!=(*itFeat)->KLT_status);
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

			// Save into the list of feats:
			//  Only if we have moved a bit!
			if (average_feats_displacement>1.0)
			{
				map<TFeatureID,TPixelCoordf> lstFeats;
				for (CFeatureList::const_iterator it=trackedFeats.begin();it!=trackedFeats.end();++it)
				{
					lstFeats[(*it)->ID] = TPixelCoordf( (*it)->x, (*it)->y );
				}
				lstTrackedFeats.push_back(lstFeats);
			}
		}

		// Condition to end this endless loop:
		//  Do we have enough tracked features?
		if (lstTrackedFeats.size()>NUM_FRAMES_TO_GO)
			break; // finish

		// At the beginning, look for new features:
		if (trackedFeats.empty() && step_num==10) // wait a bit to detect
		{
			cout << "Detecting features...\n";

			CFeatureExtraction  FE;
			FE.options.featsType = featKLT;  // use the KLT detector
			FE.options.patchSize = 21;
			FE.options.KLTOptions.min_distance = 20;
			FE.options.KLTOptions.threshold = 0.02;
			FE.options.KLTOptions.tile_image = true;

			FE.detectFeatures(theImg, trackedFeats, 0 /* first ID */, NUM_FEATS_TO_DETECT /* # feats */ );
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
			theImg.textOut(3,28,format("saved frames: %u", (unsigned int)lstTrackedFeats.size() ),TColor(20,20,255) );
		}
		{	// 	The average_feats_displacement
			theImg.filledRectangle(1,50,175,75,TColor(0,0,0));
			theImg.textOut(3,53,format("Avr move: %.03f px", average_feats_displacement ),TColor(20,20,255) );
		}

		win->showImage(theImg);

		step_num++;
	} // end infinite loop

	// Now, only keep the features that appear in all the frames:
	{
		set<TFeatureID>  knownFeats;
		//vector<map<TFeatureID,TPixelCoordf> >  lstTrackedFeats;
		for (vector<map<TFeatureID,TPixelCoordf> >::const_iterator itF = lstTrackedFeats.begin(); itF!=lstTrackedFeats.end();++itF )
			for (map<TFeatureID,TPixelCoordf>::const_iterator itF2 = itF->begin();itF2 != itF->end();++itF2 )
				knownFeats.insert( itF2->first );

		set<TFeatureID>  featThatGotLost;
		for (vector<map<TFeatureID,TPixelCoordf> >::const_iterator itF = lstTrackedFeats.begin(); itF!=lstTrackedFeats.end();++itF )
			for (set<TFeatureID>::const_iterator itID = knownFeats.begin();itID != knownFeats.end();++itID)
				if (itF->find(*itID)==itF->end())
					featThatGotLost.insert(*itID);

		// Delete those feats in all frames:
		for (vector<map<TFeatureID,TPixelCoordf> >::iterator itF = lstTrackedFeats.begin(); itF!=lstTrackedFeats.end();++itF )
			for (set<TFeatureID>::const_iterator itID = featThatGotLost.begin();itID != featThatGotLost.end();++itID)
				itF->erase(*itID);
	}

	// =================================================
	// CALL THE MAIN B.A. CALIBRATION FUNCTION
	// =================================================
	vector<vector<TPixelCoordf> >  calib_tracked_feats;

	for (size_t i=0;i<lstTrackedFeats.size();i+=FRAMES_DECIMATION)
	{
		//vector<map<TFeatureID,TPixelCoordf> >  lstTrackedFeats;
		// Add to "calib_tracked_feats" in the same order as they appear in
		//  in the map<> since they're ordered and will be in the same order:
		vector<TPixelCoordf> lstFs;

		for (map<TFeatureID,TPixelCoordf>::const_iterator itF=lstTrackedFeats[i].begin();itF!=lstTrackedFeats[i].end();++itF)
			lstFs.push_back( itF->second );

		if (!lstFs.empty())
			calib_tracked_feats.push_back(lstFs);
	}

	if (calib_tracked_feats.size()<5)
	{
		cerr << "ERROR: There are less than 5 frames with features tracked in all frames.\n";
		return -1;
	}

	if (calib_tracked_feats[0].size()<3)
	{
		cerr << "ERROR: There are less than 3 features tracked in all frames.\n";
		return -1;
	}

	cout << "# of frames to be calibrated: " << calib_tracked_feats.size() << endl;
	cout << "# of features tracked in all the frames: " << calib_tracked_feats[0].size() << endl;

	TCamCalibBAResults calib_extra_data;

	double avrg_err =
	mrpt::vision::camera_calib_ba(
		calib_tracked_feats,
		cameraParams.ncols,
		cameraParams.nrows,
		cameraParams,
		calib_extra_data
		);

	cout << "Calibration done! Avr. error = " << avrg_err << " px.\n";

	CConfigFileMemory  cfg;
	cameraParams.saveToConfigFile("CAMERA_PARAMS",cfg);
	cout << "Camera parameters:\n" << cfg.getContent();


	// ------------------------------------------------
	// Show a 3D view of the estimated scene geometry:
	// ------------------------------------------------
#if MRPT_HAS_WXWIDGETS
	mrpt::gui::CDisplayWindow3D  win3d("Reconstructed geometry",600,500);

	opengl::COpenGLScenePtr &scene = win3d.get3DSceneAndLock();

	scene->insert( opengl::CGridPlaneXY::Create() );

	for (size_t i=0;i<calib_extra_data.camera_poses.size();i++)
	{
		opengl::CSetOfObjectsPtr fr = opengl::stock_objects::CornerXYZSimple(0.4,2.0);
		fr->setName(format("Cam %u",(unsigned)i ));
		fr->enableShowName(true);
		fr->setPose( CPose3D(calib_extra_data.camera_poses[i]) );
		scene->insert( fr );
	}

	opengl::CPointCloudPtr pts = opengl::CPointCloud::Create();
	for (size_t i=0;i<calib_extra_data.landmark_positions.size();i++)
	{
		pts->insertPoint(
			calib_extra_data.landmark_positions[i].x,
			calib_extra_data.landmark_positions[i].y,
			calib_extra_data.landmark_positions[i].z );
	}
	pts->setPointSize(4.0);
	pts->setColor(0,0,1);
	scene->insert( pts );

	win3d.unlockAccess3DScene();
	win3d.repaint();

	mrpt::system::pause();
#endif  // MRPT_HAS_WXWIDGETS

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
		printf(" camera-calib-ba - Part of MRPT\n");
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
		return DoCameraCalibBA(cam);
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
