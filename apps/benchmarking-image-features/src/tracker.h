/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */

/*---------------------------------------------------------------
	APPLICATION: benchmarkingImageFeatures_gui
	FILE: tracker.h
	AUTHOR: Raghavender Sahdev <raghavendersahdev@gmail.com>
	See ReadMe.md for instructions.
  ---------------------------------------------------------------*/

//
// Created by raghavender on 23/07/17.
// this code has been build on top of that from existing tracking code in
// mrpt/apps/track-video-feats_main.cpp
//
#ifndef MRPT_TRACKER_H
#define MRPT_TRACKER_H

#endif  // MRPT_TRACKER_H

/// OpenCV includes
#include <mrpt/otherlibs/do_opencv_includes.h>

/// standard C++ includes
#include <iostream>
#include <ctype.h>
#include <algorithm>  // for copy
#include <iterator>  // for ostream_iterator
#include <vector>
#include <ctime>
#include <sstream>
#include <fstream>
#include <string>

/// MRPT includes
#include <mrpt/math/data_utils.h>
#include <mrpt/vision/CFeatureExtraction.h>
#include <mrpt/vision/tracking.h>
#include <mrpt/vision/CVideoFileWriter.h>
#include <mrpt/system/filesystem.h>
#include <mrpt/system/os.h>

using namespace mrpt::vision;
using namespace mrpt::math;
using namespace mrpt::img;
using namespace mrpt;
using namespace cv;
using namespace std;

class Tracker
{
   public:
	bool hasResolution;
	TCamera cameraParams;  // For now, will only hold the image resolution on
	// the arrive of the first frame.
	TSimpleFeatureList trackedFeats;
	unsigned int step_num;
	bool SHOW_FEAT_IDS;
	bool SHOW_RESPONSES;
	bool SHOW_FEAT_TRACKS;
	bool DO_HIST_EQUALIZE_IN_GRAYSCALE = false;
	const double MAX_FPS = 5000;  // 5.0;  // Hz (to slow down visualization).

	// "CFeatureTracker_KL" is by  far the most robust implementation for now:
	CGenericFeatureTrackerAutoPtr tracker;
	CImage previous_image;
	TSequenceFeatureObservations feat_track_history;
	bool save_tracked_history;  // Dump feat_track_history to a file at the end
	TCameraPoseID curCamPoseId;
	static const size_t FEATS_TRACK_LEN = 10;
	std::map<TFeatureID, std::list<TPixelCoord>> feat_tracks;

   public:
	/**
	 * Tracker constructor to initialize the varibales for the tracker
	 */
	Tracker();

	/**
	 * trackThemAll this function tracks the features based on the parameters
	 * specified by the user, the tracking process starts after the first 2
	 * frames. A KL tracker has been implemented here.
	 * default values are 1,1,350,11,5,5 for tracker parameters
	 * @param files_fullpath_tracking holds the path to the monocular dataset
	 * @param tracking_image_counter reads the given image from the vector of
	 * strings specified in files_fullpath_tracking variable
	 * @param remove_lost_feats variable to ask the user if wants to remove the
	 * lost features 1/0
	 * @param add_new_feats asks the user if they want to add new features as
	 * the trackers performs the tracking process
	 * @param max_feats specifies the maximum number of features to be detectd
	 * in a frame
	 * @param patch_size the patch size for mathing the key-points
	 * @param window_width width of the window
	 * @param window_height height of the window
	 * @return
	 */
	cv::Mat trackThemAll(
		vector<string> files_fullpath_tracking, int tracking_image_counter,
		int remove_lost_feats, int add_new_feats, int max_feats, int patch_size,
		int window_width, int window_height);
};
