/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */

/*---------------------------------------------------------------
	CLASS: CFeatureExtraction
	FILE: CFeatureExtraction_LSD_BLD.cpp
	AUTHOR: Raghavender Sahdev <raghavendersahdev@gmail.com>
  ---------------------------------------------------------------*/

#include "vision-precomp.h"  // Precompiled headers
#include <mrpt/system/os.h>
#include <mrpt/vision/CFeatureExtraction.h>  // important import
#include <mrpt/io/CMemoryStream.h>
#include <mrpt/otherlibs/do_opencv_includes.h>

#ifdef HAVE_OPENCV_XFEATURES2D
#include <opencv2/xfeatures2d.hpp>
#endif
#ifdef HAVE_OPENCV_LINE_DESCRIPTOR
#include <opencv2/line_descriptor.hpp>
using namespace cv::line_descriptor;
#endif

using namespace mrpt::vision;
using namespace mrpt::img;
using namespace mrpt::math;
using namespace mrpt::img;
using namespace mrpt;
using namespace std;

#if defined(HAVE_OPENCV_XFEATURES2D) && defined(HAVE_OPENCV_LINE_DESCRIPTOR)
#define HAVE_OPENCV_WITH_LSD 1
#else
#define HAVE_OPENCV_WITH_LSD 0
#endif

void CFeatureExtraction::extractFeaturesLSD(
	const mrpt::img::CImage& inImg, CFeatureList& feats, unsigned int init_ID,
	unsigned int nDesiredFeatures, const TImageROI& ROI) const
{
	MRPT_START
#if (!HAVE_OPENCV_WITH_LSD)
	THROW_EXCEPTION(
		"This function requires OpenCV modules: xfeatures2d, line_descriptor");
#else
	using namespace cv;

	vector<KeyPoint> cv_feats;  // The opencv keypoint output vector
	vector<KeyLine> cv_line;

	// Make sure we operate on a gray-scale version of the image:
	const CImage inImg_gray(inImg, FAST_REF_OR_CONVERT_TO_GRAY);

	const Mat theImg = cvarrToMat(inImg_gray.getAs<IplImage>());
	/* create a random binary mask */
	cv::Mat mask = Mat::ones(theImg.size(), CV_8UC1);

	Ptr<LSDDetector> bd = LSDDetector::createLSDDetector();

	/* extract lines */
	cv::Mat output = theImg.clone();
	bd->detect(
		theImg, cv_line, options.LSDOptions.scale, options.LSDOptions.nOctaves,
		mask);

	// *All* the features have been extracted.
	const size_t N = cv_line.size();

	// Now:
	//  1) Sort them by "response":
	// sort the LSD features by line length
	for (size_t i = 0; i < N; i++)
	{
		for (size_t j = i + 1; j < N; j++)
		{
			if (cv_line.at(j).lineLength > cv_line.at(i).lineLength)
			{
				KeyLine temp_line = cv_line.at(i);
				cv_line.at(i) = cv_line.at(j);
				cv_line.at(j) = temp_line;
			}
		}
	}

	//  2) Filter by "min-distance" (in options.FASTOptions.min_distance)  //
	//  NOT REQUIRED FOR LSD Features
	//  3) Convert to MRPT CFeatureList format.
	// Steps 2 & 3 are done together in the while() below.
	// The "min-distance" filter is done by means of a 2D binary matrix where
	// each cell is marked when one
	// feature falls within it. This is not exactly the same than a pure
	// "min-distance" but is pretty close
	// and for large numbers of features is much faster than brute force search
	// of kd-trees.
	// (An intermediate approach would be the creation of a mask image updated
	// for each accepted feature, etc.)

	unsigned int nMax =
		(nDesiredFeatures != 0 && N > nDesiredFeatures) ? nDesiredFeatures : N;
	const int offset = (int)this->options.patchSize / 2 + 1;
	const size_t size_2 = options.patchSize / 2;
	const size_t imgH = inImg.getHeight();
	const size_t imgW = inImg.getWidth();
	unsigned int i = 0;
	unsigned int cont = 0;
	TFeatureID nextID = init_ID;

	if (!options.addNewFeatures) feats.clear();

	/* draw lines extracted from octave 0 */
	if (output.channels() == 1) cvtColor(output, output, COLOR_GRAY2BGR);

	while (cont != nMax && i != N)
	{
		KeyLine kl = cv_line[i];
		KeyPoint kp;

		if (kl.octave == 0)
		{
			Point pt1 = Point2f(kl.startPointX, kl.startPointY);
			Point pt2 = Point2f(kl.endPointX, kl.endPointY);

			kp.pt.x = (pt1.x + pt2.x) / 2;
			kp.pt.y = (pt1.y + pt2.y) / 2;
			i++;
			// Patch out of the image??
			const int xBorderInf = (int)floor(kp.pt.x - size_2);
			const int xBorderSup = (int)floor(kp.pt.x + size_2);
			const int yBorderInf = (int)floor(kp.pt.y - size_2);
			const int yBorderSup = (int)floor(kp.pt.y + size_2);

			if (!(xBorderSup < (int)imgW && xBorderInf > 0 &&
				  yBorderSup < (int)imgH && yBorderInf > 0))
				continue;  // nope, skip.

			// All tests passed: add new feature:
			CFeature::Ptr ft = std::make_shared<CFeature>();
			ft->type = featLSD;
			ft->ID = nextID++;
			ft->x = kp.pt.x;
			ft->y = kp.pt.y;
			ft->x2[0] = pt1.x;
			ft->x2[1] = pt2.x;
			ft->y2[0] = pt1.y;
			ft->y2[1] = pt2.y;
			ft->response = kl.response;
			// ft->orientation		= kp.angle;
			ft->scale = kl.octave;
			// ft->patchSize		= options.patchSize;		// The size of
			// the
			// feature
			// patch

			if (options.patchSize > 0)
			{
				inImg.extract_patch(
					ft->patch, round(ft->x) - offset, round(ft->y) - offset,
					options.patchSize,
					options.patchSize);  // Image patch surronding the feature
			}
			feats.push_back(ft);
		}
		++cont;
		// cout << ft->x << "  " << ft->y << endl;
	}

#endif
	MRPT_END
}

/************************************************************************************************
 *						internal_computeBLDDescriptors
 ************************************************************************************************/
void CFeatureExtraction::internal_computeBLDLineDescriptors(
	const mrpt::img::CImage& in_img, CFeatureList& in_features) const
{
#if (!HAVE_OPENCV_WITH_LSD)
	THROW_EXCEPTION(
		"This function requires OpenCV modules: xfeatures2d, line_descriptor");
#else
	using namespace cv;

	if (in_features.empty()) return;

	const CImage img_grayscale(in_img, FAST_REF_OR_CONVERT_TO_GRAY);
	const Mat img = cvarrToMat(img_grayscale.getAs<IplImage>());

	vector<KeyPoint> cv_feats;  // OpenCV keypoint output vector
	Mat cv_descs;  // OpenCV descriptor output

	cv::Mat mask = Mat::ones(img.size(), CV_8UC1);

	BinaryDescriptor::Params params;
	params.ksize_ = options.BLDOptions.ksize_;
	params.reductionRatio = options.BLDOptions.reductionRatio;
	params.numOfOctave_ = options.BLDOptions.numOfOctave;
	params.widthOfBand_ = options.BLDOptions.widthOfBand;

	Ptr<BinaryDescriptor> bd2 =
		BinaryDescriptor::createBinaryDescriptor(params);
	/* compute lines */
	std::vector<KeyLine> keylines;

	bd2->detect(img, keylines, mask);

	/* compute descriptors */
	bd2->compute(img, keylines, cv_descs);
	keylines.resize(in_features.size());

	// -----------------------------------------------------------------
	// MRPT Wrapping
	// -----------------------------------------------------------------
	CFeatureList::iterator itList;
	int i;
	for (i = 0, itList = in_features.begin(); itList != in_features.end();
		 itList++, i++)
	{
		CFeature::Ptr ft = *itList;

		// Get the BLD descriptor
		ft->descriptors.BLD.resize(cv_descs.cols);
		for (int m = 0; m < cv_descs.cols; ++m)
			ft->descriptors.BLD[m] =
				cv_descs.at<int>(i, m);  // Get the SURF descriptor
	}  // end for

#endif  // end of opencv3 version check
}  // end internal_computeBLDDescriptors
