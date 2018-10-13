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
	FILE: tracker.cpp
	AUTHOR: Raghavender Sahdev <raghavendersahdev@gmail.com>
	See ReadMe.md for instructions.
  ---------------------------------------------------------------*/

#include "tracker.h"
#include <set>

using mrpt::system::CTicTac;
using namespace mrpt::img;

/************************************************************************************************
 *					    Tracker Constructor *
 ************************************************************************************************/
Tracker::Tracker()
{
	hasResolution = false;
	step_num = 0;
	SHOW_FEAT_IDS = true;
	SHOW_RESPONSES = true;
	SHOW_FEAT_TRACKS = true;
	DO_HIST_EQUALIZE_IN_GRAYSCALE = false;

	// "CFeatureTracker_KL" is by  far the most robust implementation for now:
	tracker = CGenericFeatureTrackerAutoPtr(new CFeatureTracker_KL);
	CImage previous_image;
	TSequenceFeatureObservations feat_track_history;
	save_tracked_history =
		true;  // Dump feat_track_history to a file at the end
	curCamPoseId = 0;
}

/************************************************************************************************
 *					    Track Them All tracker *
 ************************************************************************************************/
cv::Mat Tracker::trackThemAll(
	vector<string> files_fullpath_tracking, int tracking_image_counter,
	int remove_lost_feats, int add_new_feats, int max_feats, int patch_size,
	int window_width, int window_height)
{
	tracker->enableTimeLogger(true);  // Do time profiling.

	// Set of parameters common to any tracker implementation:
	// -------------------------------------------------------------
	// To see all the existing params and documentation, see
	// mrpt::vision::CGenericFeatureTracker
	tracker->extra_params["remove_lost_features"] =
		remove_lost_feats;  //;1;   // automatically remove out-of-image and
	// badly tracked features

	tracker->extra_params["add_new_features"] =
		add_new_feats;  // 1;   // track, AND ALSO, add new features
	tracker->extra_params["add_new_feat_min_separation"] = 32;
	tracker->extra_params["minimum_KLT_response_to_add"] = 10;
	tracker->extra_params["add_new_feat_max_features"] = max_feats;  // 350;
	tracker->extra_params["add_new_feat_patch_size"] = patch_size;  // 11;

	tracker->extra_params["update_patches_every"] = 0;  // Don't update patches.

	tracker->extra_params["check_KLT_response_every"] =
		5;  // Re-check the KLT-response to assure features are in good points.
	tracker->extra_params["minimum_KLT_response"] = 5;

	// Specific params for "CFeatureTracker_KL"
	// ------------------------------------------------------
	tracker->extra_params["window_width"] = window_width;  // 5;
	tracker->extra_params["window_height"] = window_height;  // 5;
	// tracker->extra_params["LK_levels"] = 3;
	// tracker->extra_params["LK_max_iters"] = 10;
	// tracker->extra_params["LK_epsilon"] = 0.1;
	// tracker->extra_params["LK_max_tracking_error"] = 150;

	long current_num = tracking_image_counter % files_fullpath_tracking.size();
	CImage theImg;  // The grabbed image:
	theImg.loadFromFile(files_fullpath_tracking.at(current_num));

	// Take the resolution upon first valid frame.
	if (!hasResolution)
	{
		hasResolution = true;
		// cameraParams.scaleToResolution()...
		cameraParams.ncols = theImg.getWidth();
		cameraParams.nrows = theImg.getHeight();
	}

	// Do tracking:
	if (step_num > 1)  // we need "previous_image" to be valid.
	{
		// This single call makes: detection, tracking, recalculation of
		// KLT_response, etc.
		tracker->trackFeatures(previous_image, theImg, trackedFeats);
	}

	// Save the image for the next step:
	previous_image = theImg;

	// Save history of feature observations:
	tracker->getProfiler().enter("Save history");

	for (size_t i = 0; i < trackedFeats.size(); ++i)
	{
		TSimpleFeature& f = trackedFeats[i];

		const TPixelCoordf pxRaw(f.pt.x, f.pt.y);
		TPixelCoordf pxUndist;
		// mrpt::vision::pinhole::undistort_point(pxRaw,pxUndist, cameraParams);
		pxUndist = pxRaw;

		feat_track_history.push_back(
			TFeatureObservation(f.ID, curCamPoseId, pxUndist));
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

	{  // FPS:
		static CTicTac tictac;
		// const double T = tictac.Tac();
		tictac.Tic();
		// const double fps = 1.0 / (std::max(1e-5, T));
		// theImg.filledRectangle(1,1,175,25,TColor(0,0,0));

		// const int current_adapt_thres =
		tracker->getDetectorAdaptiveThreshold();
	}

	// Draw feature tracks
	if (SHOW_FEAT_TRACKS)
	{
		// Update new feature coords:
		tracker->getProfiler().enter("drawFeatureTracks");

		std::set<TFeatureID> observed_IDs;

		// cout << "tracked feats size" << trackedFeats.size() << endl;
		for (size_t i = 0; i < trackedFeats.size(); ++i)
		{
			const TSimpleFeature& ft = trackedFeats[i];
			std::list<TPixelCoord>& seq = feat_tracks[ft.ID];

			// drawMarker(cvImg1, Point(trackedFeats.getFeatureX(i),
			// trackedFeats.getFeatureY(i)),  Scalar(0, 255, 0), MARKER_CROSS,
			// CROSS_SIZE, CROSS_THICKNESS);

			observed_IDs.insert(ft.ID);

			if (seq.size() >= FEATS_TRACK_LEN) seq.erase(seq.begin());
			seq.push_back(ft.pt);

			// Draw:
			if (seq.size() > 1)
			{
				const auto it_end = seq.end();

				auto it = seq.begin();
				auto it_prev = it++;

				for (; it != it_end; ++it)
				{
					theImg.line(
						it_prev->x, it_prev->y, it->x, it->y,
						TColor(190, 190, 190));
					it_prev = it;
				}
			}
		}

		tracker->getProfiler().leave("drawFeatureTracks");

		// Purge old data:
		for (auto it = feat_tracks.begin(); it != feat_tracks.end();)
		{
			if (observed_IDs.find(it->first) == observed_IDs.end())
			{
				auto next_it = it;
				next_it++;
				feat_tracks.erase(it);
				it = next_it;
			}
			else
				++it;
		}
	}

	// Draw Tracked feats:
	{
		theImg.selectTextFont("5x7");
		tracker->getProfiler().enter("drawFeatures");
		theImg.drawFeatures(
			trackedFeats, TColor(0, 0, 255), SHOW_FEAT_IDS, SHOW_RESPONSES);
		tracker->getProfiler().leave("drawFeatures");
	}
	step_num++;

	// converting the cv::Mat to a QImage and changing the resolution of the
	// output images
	cv::Mat cvImg1 = cv::cvarrToMat(theImg.getAs<IplImage>());

	return cvImg1;
}
