/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */

/*---------------------------------------------------------------
	APPLICATION: CFeatureExtraction
	FILE: CFeatureExtraction_AKAZE.cpp
	AUTHOR: Raghavender Sahdev <raghavendersahdev@gmail.com>
  ---------------------------------------------------------------*/

#include "vision-precomp.h"  // Precompiled headers
// Universal include for all versions of OpenCV
#include <mrpt/otherlibs/do_opencv_includes.h>

#include <mrpt/system/os.h>
#include <mrpt/vision/CFeatureExtraction.h>  // important import
#include <mrpt/io/CMemoryStream.h>

using namespace mrpt::vision;
using namespace mrpt::img;
using namespace mrpt::math;
using namespace mrpt::img;
using namespace mrpt;
using namespace std;

void CFeatureExtraction::extractFeaturesAKAZE(
	const mrpt::img::CImage& inImg, CFeatureList& feats, unsigned int init_ID,
	unsigned int nDesiredFeatures, const TImageROI& ROI) const
{
	MRPT_UNUSED_PARAM(ROI);
	MRPT_START
#if MRPT_HAS_OPENCV
#if MRPT_OPENCV_VERSION_NUM < 0x300
	THROW_EXCEPTION("This function requires OpenCV > 3.0.0");
#else

	using namespace cv;
	vector<KeyPoint> cv_feats;  // The opencv keypoint output vector
	// Make sure we operate on a gray-scale version of the image:
	const CImage inImg_gray(inImg, FAST_REF_OR_CONVERT_TO_GRAY);

#if MRPT_OPENCV_VERSION_NUM >= 0x300

	const Mat theImg = cvarrToMat(inImg_gray.getAs<IplImage>());
	Ptr<AKAZE> akaze = AKAZE::create(
		options.AKAZEOptions.descriptor_type,
		options.AKAZEOptions.descriptor_size,
		options.AKAZEOptions.descriptor_channels,
		options.AKAZEOptions.threshold, options.AKAZEOptions.nOctaves,
		options.AKAZEOptions.nOctaveLayers, options.AKAZEOptions.diffusivity);

	akaze->detect(theImg, cv_feats);

	// *All* the features have been extracted.
	const size_t N = cv_feats.size();

#endif
	// sort the AKAZE features by line length
	for (size_t i = 0; i < N; i++)
	{
		for (size_t j = i + 1; j < N; j++)
		{
			if (cv_feats.at(j).response > cv_feats.at(i).response)
			{
				KeyPoint temp_point = cv_feats.at(i);
				cv_feats.at(i) = cv_feats.at(j);
				cv_feats.at(j) = temp_point;
			}
		}
	}

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

	while (cont != nMax && i != N)
	{
		// Take the next feature from the ordered list of good features:
		const KeyPoint& kp = cv_feats[i];
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
		ft->type = featAKAZE;
		ft->ID = nextID++;
		ft->x = kp.pt.x;
		ft->y = kp.pt.y;
		ft->response = kp.response;
		ft->orientation = kp.angle;
		ft->scale = kp.octave;
		ft->patchSize = options.patchSize;  // The size of the feature patch

		if (options.patchSize > 0)
		{
			inImg.extract_patch(
				ft->patch, round(ft->x) - offset, round(ft->y) - offset,
				options.patchSize,
				options.patchSize);  // Image patch surronding the feature
		}
		feats.push_back(ft);
		++cont;
		// cout << ft->x << "  " << ft->y << endl;
	}

#endif
#endif
	MRPT_END
}
