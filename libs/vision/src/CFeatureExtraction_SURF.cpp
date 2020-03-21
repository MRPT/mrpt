/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2020, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#include "vision-precomp.h"  // Precompiled headers

#include <mrpt/vision/CFeatureExtraction.h>

// Universal include for all versions of OpenCV
#include <mrpt/3rdparty/do_opencv_includes.h>

#ifdef HAVE_OPENCV_NONFREE  // MRPT_HAS_OPENCV_NONFREE
#include <opencv2/nonfree/nonfree.hpp>
#endif
#ifdef HAVE_OPENCV_FEATURES2D
#include <opencv2/features2d/features2d.hpp>
#endif
#ifdef HAVE_OPENCV_XFEATURES2D
#include <opencv2/xfeatures2d.hpp>
#endif

using namespace mrpt;
using namespace mrpt::vision;
using namespace mrpt::img;
using namespace mrpt::img;
using namespace mrpt::system;
using namespace std;

// Was: MRPT_HAS_OPENCV_NONFREE
#if defined(HAVE_OPENCV_XFEATURES2D) || \
	(MRPT_OPENCV_VERSION_NUM < 0x300 && MRPT_OPENCV_VERSION_NUM >= 0x240)
#define HAVE_OPENCV_WITH_SURF 1
#else
#define HAVE_OPENCV_WITH_SURF 0
#endif

void CFeatureExtraction::extractFeaturesSURF(
	const mrpt::img::CImage& inImg, CFeatureList& feats, unsigned int init_ID,
	unsigned int nDesiredFeatures, const TImageROI& ROI)
{
#if HAVE_OPENCV_WITH_SURF
	mrpt::system::CTimeLoggerEntry tle(profiler, "extractFeaturesSURF");

	using namespace cv;

	const CImage img_grayscale(inImg, FAST_REF_OR_CONVERT_TO_GRAY);
	const Mat& img = img_grayscale.asCvMatRef();

	vector<KeyPoint> cv_feats;  // OpenCV keypoint output vector
	Mat cv_descs;  // OpenCV descriptor output

// gb redesign start -make sure that the Algorithm::create is used only for
// older openCV versions
#if MRPT_OPENCV_VERSION_NUM < 0x300 && MRPT_OPENCV_VERSION_NUM >= 0x240

	Ptr<Feature2D> surf = Algorithm::create<Feature2D>("Feature2D.SURF");
	if (surf.empty())
		CV_Error(
			CV_StsNotImplemented,
			"OpenCV <v3.0 was built without SURF support");

	surf->set("hessianThreshold", options.SURFOptions.hessianThreshold);
	surf->set("nOctaves", options.SURFOptions.nOctaves);
	surf->set("nOctaveLayers", options.SURFOptions.nLayersPerOctave);
	// surf->set("upright", params.upright != 0);
	surf->set("extended", options.SURFOptions.rotation_invariant);

	surf->operator()(img, Mat(), cv_feats, cv_descs);

#else
	Ptr<xfeatures2d::SURF> surf =
		xfeatures2d::SURF::create(100.0, 4, 3, 0, 0);  // gb

	if (surf.empty())
		CV_Error(
			CV_StsNotImplemented, "OpenCV 3.0 was built without SURF support");
	surf->setHessianThreshold(options.SURFOptions.hessianThreshold);
	surf->setNOctaves(options.SURFOptions.nOctaves);
	surf->setNOctaveLayers(options.SURFOptions.nLayersPerOctave);
	// surf->setUpright(params.upright != 0);
	surf->setExtended(options.SURFOptions.rotation_invariant);
	// gb redesign end
	surf->detectAndCompute(img, Mat(), cv_feats, cv_descs);
#endif

	// -----------------------------------------------------------------
	// MRPT Wrapping
	// -----------------------------------------------------------------
	feats.clear();
	unsigned int nCFeats = init_ID;
	int offset = (int)this->options.patchSize / 2 + 1;
	unsigned int imgH = inImg.getHeight();
	unsigned int imgW = inImg.getWidth();

	const size_t n_feats =
		(nDesiredFeatures == 0)
			? cv_feats.size()
			: std::min((size_t)nDesiredFeatures, cv_feats.size());

	for (size_t i = 0; i < n_feats; i++)
	{
		// Get the OpenCV SURF point
		CFeature ft;
		const KeyPoint& point = cv_feats[i];

		const int xBorderInf = (int)floor(point.pt.x - options.patchSize / 2);
		const int xBorderSup = (int)floor(point.pt.x + options.patchSize / 2);
		const int yBorderInf = (int)floor(point.pt.y - options.patchSize / 2);
		const int yBorderSup = (int)floor(point.pt.y + options.patchSize / 2);

		if (options.patchSize == 0 ||
			((xBorderSup < (int)imgW) && (xBorderInf > 0) &&
			 (yBorderSup < (int)imgH) && (yBorderInf > 0)))
		{
			ft.type = featSURF;
			ft.keypoint.pt.x = point.pt.x;  // X position
			ft.keypoint.pt.y = point.pt.y;  // Y position
			ft.orientation = point.angle;  // Orientation
			ft.keypoint.octave = point.size * 1.2 / 9;  // Scale
			ft.keypoint.ID = nCFeats++;  // Feature ID into extraction
			ft.patchSize = options.patchSize;  // The size of the feature patch

			// Image patch?
			if (options.patchSize > 0)
			{
				mrpt::img::CImage p;
				inImg.extract_patch(
					p, round(point.pt.x) - offset, round(point.pt.y) - offset,
					options.patchSize, options.patchSize);

				ft.patch = std::move(p);
			}

			// Get the SURF descriptor
			ft.descriptors.SURF.emplace();
			auto& desc = ft.descriptors.SURF.value();
			desc.resize(cv_descs.cols);
			for (int m = 0; m < cv_descs.cols; ++m)
				desc[m] = cv_descs.at<float>(i, m);

			feats.emplace_back(std::move(ft));

		}  // end if
	}  // end for

#else
	THROW_EXCEPTION(
		"Method not available: MRPT compiled without OpenCV, or against a "
		"version of OpenCV without SURF");
#endif  // MRPT_HAS_OPENCV
}  // end extractFeaturesSURF

void CFeatureExtraction::internal_computeSurfDescriptors(
	const mrpt::img::CImage& inImg, CFeatureList& in_features)
{
#if HAVE_OPENCV_WITH_SURF
	using namespace cv;
	mrpt::system::CTimeLoggerEntry tle(
		profiler, "internal_computeSurfDescriptors");

	if (in_features.empty()) return;

	const CImage img_grayscale(inImg, FAST_REF_OR_CONVERT_TO_GRAY);
	const Mat& img = img_grayscale.asCvMatRef();

	vector<KeyPoint> cv_feats;  // OpenCV keypoint output vector
	Mat cv_descs;  // OpenCV descriptor output

	// Fill in the desired key-points:
	cv_feats.resize(in_features.size());
	for (size_t i = 0; i < in_features.size(); ++i)
	{
		cv_feats[i].pt.x = in_features[i].keypoint.pt.x;
		cv_feats[i].pt.y = in_features[i].keypoint.pt.y;
		cv_feats[i].size = 1 << in_features[i].keypoint.octave;
	}

// Only computes the descriptors:
// gb redesign
#if MRPT_OPENCV_VERSION_NUM < 0x300 && MRPT_OPENCV_VERSION_NUM >= 0x240

	Ptr<Feature2D> surf = Algorithm::create<Feature2D>("Feature2D.SURF");
	if (surf.empty())
		CV_Error(CV_StsNotImplemented, "OpenCV was built without SURF support");
	surf->set("hessianThreshold", options.SURFOptions.hessianThreshold);
	surf->set("nOctaves", options.SURFOptions.nOctaves);
	surf->set("nOctaveLayers", options.SURFOptions.nLayersPerOctave);
	// surf->set("upright", params.upright != 0);
	surf->set("extended", options.SURFOptions.rotation_invariant);

	surf->compute(
		img, cv_feats,
		cv_descs);  // is that correct? the version from above is:
// surf->operator()(img, Mat(), cv_feats, cv_descs);
#else

	Ptr<xfeatures2d::SURF> surf =
		xfeatures2d::SURF::create(100.0, 4, 3, 0, 0);  // gb

	if (surf.empty())
		CV_Error(CV_StsNotImplemented, "OpenCV was built without SURF support");
	surf->setHessianThreshold(options.SURFOptions.hessianThreshold);
	surf->setNOctaves(options.SURFOptions.nOctaves);
	surf->setNOctaveLayers(options.SURFOptions.nLayersPerOctave);
	// surf->setUpright(params.upright != 0);
	surf->setExtended(options.SURFOptions.rotation_invariant);

	surf->detectAndCompute(img, Mat(), cv_feats, cv_descs);
#endif
	// gb redesign end
	// -----------------------------------------------------------------
	// MRPT Wrapping
	// -----------------------------------------------------------------
	int i = 0;
	for (auto& ft : in_features)
	{
		// Get the OpenCV SURF point
		const KeyPoint& point = cv_feats[i];

		ft.orientation = point.angle;
		ft.keypoint.octave = point.octave;

		// Get the SURF descriptor
		ft.descriptors.SURF.emplace();
		auto& desc = ft.descriptors.SURF.value();
		desc.resize(cv_descs.cols);
		for (int m = 0; m < cv_descs.cols; ++m)
			desc[m] = cv_descs.at<float>(i, m);

		i++;
	}

#else
	THROW_EXCEPTION(
		"Method not available: MRPT compiled without OpenCV, or against a "
		"version of OpenCV without SURF");
#endif  // MRPT_HAS_OPENCV
}  // end internal_computeSurfDescriptors
