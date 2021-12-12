/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2021, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#include "vision-precomp.h"	 // Precompiled headers
//
#include <mrpt/system/os.h>
#include <mrpt/vision/CFeatureExtraction.h>

// Universal include for all versions of OpenCV
#include <mrpt/3rdparty/do_opencv_includes.h>
#ifdef HAVE_OPENCV_NONFREE	// MRPT_HAS_OPENCV_NONFREE
#include <opencv2/nonfree/nonfree.hpp>
#endif
#ifdef HAVE_OPENCV_FEATURES2D
#if MRPT_OPENCV_VERSION_NUM >= 0x300
#include <opencv2/features2d.hpp>
#else
#include <opencv2/features2d/features2d.hpp>
#endif
#endif
#ifdef HAVE_OPENCV_XFEATURES2D
#include <opencv2/xfeatures2d.hpp>
#endif

using namespace mrpt;
using namespace mrpt::vision;
using namespace mrpt::img;
using namespace mrpt::system;
using namespace mrpt::math;
using namespace std;

#if MRPT_HAS_OPENCV
using namespace cv;
#endif

void CFeatureExtraction::extractFeaturesSIFT(
	const CImage& img, CFeatureList& feats, unsigned int init_ID,
	unsigned int nDesiredFeatures, const TImageROI& ROI)
{
	mrpt::system::CTimeLoggerEntry tle(profiler, "extractFeaturesSIFT");

	bool usingROI = false;
	if (ROI.xMin != 0 || ROI.xMax != 0 || ROI.yMin != 0 || ROI.yMax != 0)
		usingROI = true;  // A ROI has been defined

	// ROI can not be managed properly (yet) with these method, so we extract a
	// subimage

	// use a smart pointer so we just copy the pointer if the image is
	// grayscale, or we'll create a new one if it was RGB:
	CImage img_grayscale(img, FAST_REF_OR_CONVERT_TO_GRAY);
	if (usingROI)
	{
		ASSERT_(
			ROI.xMin < ROI.xMax && ROI.xMax < img.getWidth() &&
			ROI.yMax < img.getHeight() && ROI.yMin < ROI.yMax);
		CImage auximg;
		img_grayscale.extract_patch(
			auximg, ROI.xMin, ROI.yMin, ROI.xMax - ROI.xMin + 1,
			ROI.yMax - ROI.yMin + 1);  // Subimage in "auxImg"
		img_grayscale.swap(auximg);
	}

	ASSERT_(options.SIFTOptions.implementation == OpenCV);

	// in opencv 4.4 SIFT got into the main repo
#if defined(HAVE_OPENCV_NONFREE) || defined(HAVE_OPENCV_XFEATURES2D) ||        \
	(MRPT_OPENCV_VERSION_NUM >= 0x440)

#if MRPT_OPENCV_VERSION_NUM < 0x300
	SiftFeatureDetector SIFTDetector(
		options.SIFTOptions.threshold, options.SIFTOptions.edgeThreshold);

	SiftDescriptorExtractor SIFTDescriptor;
	vector<KeyPoint> cv_feats;	// The OpenCV output feature list
	const Mat& theImg = img_grayscale.asCvMatRef();
	SIFTDetector.detect(theImg, cv_feats);
	Mat desc;
	SIFTDescriptor.compute(theImg, cv_feats, desc);
#else
	// MRPT_OPENCV_VERSION_NUM >= 0x300
	using namespace cv;
	vector<KeyPoint> cv_feats;

#if MRPT_OPENCV_VERSION_NUM >= 0x440
	using sift_t = cv::SIFT;
#else
	using sift_t = cv::xfeatures2d::SIFT;
#endif

	auto sift = sift_t::create(
		nDesiredFeatures, options.SIFTOptions.octaveLayers,
		options.SIFTOptions.threshold, options.SIFTOptions.edgeThreshold, 1.6);
	const Mat& theImg = img_grayscale.asCvMatRef();
	Mat desc;
	sift->detectAndCompute(theImg, noArray() /*mask*/, cv_feats, desc);
#endif
	const size_t N = cv_feats.size();
	unsigned int nMax =
		nDesiredFeatures != 0 && N > nDesiredFeatures ? nDesiredFeatures : N;
	const int offset = (int)this->options.patchSize / 2 + 1;
	const size_t size_2 = options.patchSize / 2;
	const size_t imgH = img.getHeight();
	const size_t imgW = img.getWidth();
	unsigned int i = 0;
	unsigned int cont = 0;
	TFeatureID nextID = init_ID;
	feats.clear();

	while (cont != nMax && i != N)
	{
		const int xBorderInf = (int)floor(cv_feats[i].pt.x - size_2);
		const int xBorderSup = (int)floor(cv_feats[i].pt.x + size_2);
		const int yBorderInf = (int)floor(cv_feats[i].pt.y - size_2);
		const int yBorderSup = (int)floor(cv_feats[i].pt.y + size_2);

		if (options.patchSize == 0 ||
			((xBorderSup < (int)imgW) && (xBorderInf > 0) &&
			 (yBorderSup < (int)imgH) && (yBorderInf > 0)))
		{
			CFeature ft;
			ft.type = featSIFT;
			ft.keypoint.ID = nextID++;
			ft.keypoint.pt.x = cv_feats[i].pt.x;
			ft.keypoint.pt.y = cv_feats[i].pt.y;
			ft.keypoint.response = cv_feats[i].response;
			ft.orientation = cv_feats[i].angle;
			ft.keypoint.octave = mrpt::round(std::log2(cv_feats[i].size));
			ft.patchSize = options.patchSize;

			// The descriptor
			ft.descriptors.SIFT.emplace();
			auto& out_desc = ft.descriptors.SIFT.value();
			out_desc.resize(128);
			memcpy(
				&out_desc[0], &desc.data[128 * i], 128 * sizeof(out_desc[0]));

			if (options.patchSize > 0)
			{
				mrpt::img::CImage p;
				img.extract_patch(
					p, round(ft.keypoint.pt.x) - offset,
					round(ft.keypoint.pt.y) - offset, options.patchSize,
					options.patchSize);
				ft.patch = std::move(p);
			}
			feats.push_back(ft);
			++cont;
		}
		++i;
	}
	feats.resize(cont);
#else
	THROW_EXCEPTION(
		"This method requires OpenCV >= 2.1.1 with nonfree module or OpenCV "
		">=4.4");
#endif

}  // end extractFeaturesSIFT

// Compute SIFT descriptors on a set of already localized points
void CFeatureExtraction::internal_computeSiftDescriptors(
	const CImage& img, CFeatureList& in_features)
{
	mrpt::system::CTimeLoggerEntry tle(
		profiler, "internal_computeSiftDescriptors");

	ASSERT_(in_features.size() > 0);

	// use a smart pointer so we just copy the pointer if the image is
	// grayscale, or we'll create a new one if it was RGB:
	CImage img_grayscale(img, FAST_REF_OR_CONVERT_TO_GRAY);

	ASSERT_(options.SIFTOptions.implementation == OpenCV);

	// in opencv 4.4 SIFT got into the main repo
#if MRPT_OPENCV_VERSION_NUM >= 0x440
	using namespace cv;
	vector<KeyPoint> cv_feats;

	// Fill in the desired key-points:
	cv_feats.resize(in_features.size());
	for (size_t i = 0; i < in_features.size(); ++i)
	{
		cv_feats[i].pt.x = in_features[i].keypoint.pt.x;
		cv_feats[i].pt.y = in_features[i].keypoint.pt.y;
		cv_feats[i].size = 1 << in_features[i].keypoint.octave;
	}

	using sift_t = cv::SIFT;

	auto sift = sift_t::create(
		0, options.SIFTOptions.octaveLayers, options.SIFTOptions.threshold,
		options.SIFTOptions.edgeThreshold, 1.6);
	const Mat& theImg = img_grayscale.asCvMatRef();
	Mat cv_descs;
	sift->compute(theImg, cv_feats, cv_descs);

	int i = 0;
	for (auto& ft : in_features)
	{
		// Get the OpenCV SIFT point
		const KeyPoint& point = cv_feats[i];

		ft.orientation = point.angle;
		ft.keypoint.octave = point.octave;

		// Get the SIFT descriptor
		ft.descriptors.SIFT.emplace();
		auto& desc = ft.descriptors.SIFT.value();
		desc.resize(cv_descs.cols);
		for (int m = 0; m < cv_descs.cols; ++m)
			desc[m] = cv_descs.at<float>(i, m);

		i++;
	}
#else
	THROW_EXCEPTION("This method requires OpenCV >=4.4");
#endif

}  // end computeSiftDescriptors
