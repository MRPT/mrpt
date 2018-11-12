/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */

#include "vision-precomp.h"  // Precompiled headers

#include <mrpt/system/os.h>
#include <mrpt/vision/CFeatureExtraction.h>

// Universal include for all versions of OpenCV
#include <mrpt/otherlibs/do_opencv_includes.h>
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
using namespace mrpt::system;
using namespace mrpt::math;
using namespace std;

#if MRPT_HAS_OPENCV
using namespace cv;
#endif

/************************************************************************************************
 *								extractFeaturesSIFT *
 ************************************************************************************************/
void CFeatureExtraction::extractFeaturesSIFT(
	const CImage& img, CFeatureList& feats, unsigned int init_ID,
	unsigned int nDesiredFeatures, const TImageROI& ROI) const
{
	bool usingROI = false;
	if (ROI.xMin != 0 || ROI.xMax != 0 || ROI.yMin != 0 || ROI.yMax != 0)
		usingROI = true;  // A ROI has been defined

	// ROI can not be managed properly (yet) with these method, so we extract a
	// subimage

	// use a smart pointer so we just copy the pointer if the image is
	// grayscale, or we'll create a new one if it was RGB:
	CImage img_grayscale(
		img, FAST_REF_OR_CONVERT_TO_GRAY);  // Was: auxImg::Ptr;
	if (usingROI)
	{
		ASSERT_(
			ROI.xMin >= 0 && ROI.xMin < ROI.xMax && ROI.xMax < img.getWidth() &&
			ROI.yMin >= 0 && ROI.yMax < img.getHeight() && ROI.yMin < ROI.yMax);
		CImage auximg;
		img_grayscale.extract_patch(
			auximg, ROI.xMin, ROI.yMin, ROI.xMax - ROI.xMin + 1,
			ROI.yMax - ROI.yMin + 1);  // Subimage in "auxImg"
		img_grayscale.swap(auximg);
	}

	switch (options.SIFTOptions.implementation)
	{
		case CSBinary:
		{
			THROW_EXCEPTION(
				"CSBinary SIFT not available since MRPT 1.9.9: "
				"Use `OpenCV` version");
			break;
		}  // end case Binary in C#
		case VedaldiBinary:
		{
			THROW_EXCEPTION(
				"Vedaldi SIFT not available since MRPT 1.9.9: "
				"Use `OpenCV` version");
			break;
		}  // end case Binary by Vedaldi
		case LoweBinary:  // Binary by David Lowe
		{
			THROW_EXCEPTION(
				"LoweBinary not available since MRPT 1.9.9: "
				"Use `OpenCV` version");
			break;
		}  // end case Binary by Lowe
		case Hess:  // Implementation by Robert Hess
		{
			THROW_EXCEPTION(
				"Hess SIFT not available since MRPT 1.9.9: "
				"Use `OpenCV` version");
			break;
		}  // end case Hess
		//***********************************************************************************************
		// USING OPENCV
		//***********************************************************************************************
		case OpenCV:
		{
#if defined(HAVE_OPENCV_NONFREE) || \
	defined(HAVE_OPENCV_XFEATURES2D)  // MRPT_HAS_OPENCV_NONFREE

#if MRPT_OPENCV_VERSION_NUM < 0x300

			SiftFeatureDetector SIFTDetector(
				options.SIFTOptions
					.threshold,  // SIFT::DetectorParams::GET_DEFAULT_THRESHOLD(),
				options.SIFTOptions
					.edgeThreshold  // SIFT::DetectorParams::GET_DEFAULT_EDGE_THRESHOLD()
				// );
			);

			SiftDescriptorExtractor SIFTDescriptor;

			vector<KeyPoint> cv_feats;  // The OpenCV output feature list

			const IplImage* cGrey = img_grayscale.getAs<IplImage>();

			Mat theImg = cvarrToMat(cGrey);
			SIFTDetector.detect(theImg, cv_feats);

			Mat desc;
			SIFTDescriptor.compute(theImg, cv_feats, desc);

			// fromOpenCVToMRPT( theImg, cv_feats, desc, nDesiredFeatures,
			// outList );
			const size_t N = cv_feats.size();
			unsigned int nMax = nDesiredFeatures != 0 && N > nDesiredFeatures
									? nDesiredFeatures
									: N;
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
					CFeature::Ptr ft = mrpt::make_aligned_shared<CFeature>();
					ft->type = featSIFT;
					ft->ID = nextID++;
					ft->x = cv_feats[i].pt.x;
					ft->y = cv_feats[i].pt.y;
					ft->response = cv_feats[i].response;
					ft->orientation = cv_feats[i].angle;
					ft->scale = cv_feats[i].size;
					ft->patchSize =
						options.patchSize;  // The size of the feature patch
					ft->descriptors.SIFT.resize(128);
					memcpy(
						&(ft->descriptors.SIFT[0]), &desc.data[128 * i],
						128 *
							sizeof(ft->descriptors.SIFT[0]));  // The descriptor

					if (options.patchSize > 0)
					{
						img.extract_patch(
							ft->patch, round(ft->x) - offset,
							round(ft->y) - offset, options.patchSize,
							options.patchSize);  // Image patch surronding the
						// feature
					}
					feats.push_back(ft);
					++cont;
				}
				++i;
			}
			feats.resize(cont);
#else
			// MRPT_OPENCV_VERSION_NUM >= 0x300
			using namespace cv;
			vector<KeyPoint> cv_feats;

			cv::Ptr<cv::xfeatures2d::SIFT> sift = cv::xfeatures2d::SIFT::create(
				nDesiredFeatures, 3, options.SIFTOptions.threshold,
				options.SIFTOptions.edgeThreshold, 1.6);  // gb
			const IplImage* cGrey = img_grayscale.getAs<IplImage>();
			Mat theImg = cvarrToMat(cGrey);
			// SIFTDetector.detect(theImg, cv_feats);
			sift->detect(theImg, cv_feats);  // gb
			Mat desc;
			// SIFTDescriptor.compute(theImg, cv_feats, desc);
			sift->compute(theImg, cv_feats, desc);

			// fromOpenCVToMRPT( theImg, cv_feats, desc, nDesiredFeatures,
			// outList );
			const size_t N = cv_feats.size();
			unsigned int nMax = nDesiredFeatures != 0 && N > nDesiredFeatures
									? nDesiredFeatures
									: N;
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
					CFeature::Ptr ft = mrpt::make_aligned_shared<CFeature>();
					ft->type = featSIFT;
					ft->ID = nextID++;
					ft->x = cv_feats[i].pt.x;
					ft->y = cv_feats[i].pt.y;
					ft->response = cv_feats[i].response;
					ft->orientation = cv_feats[i].angle;
					ft->scale = cv_feats[i].size;
					ft->patchSize =
						options.patchSize;  // The size of the feature patch
					ft->descriptors.SIFT.resize(128);
					memcpy(
						&(ft->descriptors.SIFT[0]), &desc.data[128 * i],
						128 *
							sizeof(ft->descriptors.SIFT[0]));  // The descriptor

					if (options.patchSize > 0)
					{
						img.extract_patch(
							ft->patch, round(ft->x) - offset,
							round(ft->y) - offset, options.patchSize,
							options.patchSize);  // Image patch surronding the
						// feature
					}
					feats.push_back(ft);
					++cont;
				}
				++i;
			}
			feats.resize(cont);
#endif
#else
			THROW_EXCEPTION(
				"This method requires OpenCV >= 2.1.1 with nonfree module")
#endif
			break;
		}  // end case OpenCV
			return;
		default:
		{
			break;
		}  // end default
	}  // end switch
}  // end extractFeaturesSIFT

// Compute SIFT descriptors on a set of already localized points
void CFeatureExtraction::internal_computeSiftDescriptors(
	const CImage& in_img, CFeatureList& in_features) const
{
	ASSERT_(in_features.size() > 0);
	switch (options.SIFTOptions.implementation)
	{
		default:
		{
			cerr << "SIFT Extraction method not supported for features with "
					"already known image coordinates"
				 << endl;
			break;
		}
	}  // end switch

}  // end computeSiftDescriptors
