/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2019, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#include "vision-precomp.h"  // Precompiled headers

#include <mrpt/3rdparty/do_opencv_includes.h>
#include <mrpt/maps/CLandmarksMap.h>
#include <mrpt/math/geometry.h>
#include <mrpt/math/ops_matrices.h>
#include <mrpt/math/ops_vectors.h>
#include <mrpt/math/utils.h>
#include <mrpt/obs/CObservationBearingRange.h>
#include <mrpt/obs/CObservationStereoImages.h>
#include <mrpt/obs/CObservationVisualLandmarks.h>
#include <mrpt/poses/CPoint3D.h>
#include <mrpt/system/CTicTac.h>
#include <mrpt/system/filesystem.h>
#include <mrpt/vision/CFeature.h>
#include <mrpt/vision/CFeatureExtraction.h>
#include <mrpt/vision/pinhole.h>
#include <mrpt/vision/utils.h>
#include <Eigen/Dense>

using namespace mrpt;
using namespace mrpt::vision;
using namespace mrpt::img;
using namespace mrpt::config;
using namespace mrpt::maps;
using namespace mrpt::tfest;
using namespace mrpt::math;
using namespace mrpt::system;
using namespace mrpt::poses;
using namespace mrpt::obs;
using namespace std;
const int FEAT_FREE = -1;
// const int NOT_ASIG = 0;
// const int ASG_FEAT = 1;
// const int AMB_FEAT = 2;

/*-------------------------------------------------------------
					openCV_cross_correlation
-------------------------------------------------------------*/
void vision::openCV_cross_correlation(
	const CImage& img, const CImage& patch_img, size_t& x_max, size_t& y_max,
	double& max_val, int x_search_ini, int y_search_ini, int x_search_size,
	int y_search_size)
{
	MRPT_START

#if MRPT_HAS_OPENCV
	bool entireImg =
		(x_search_ini < 0 || y_search_ini < 0 || x_search_size < 0 ||
		 y_search_size < 0);

	CImage im, patch_im;

	if (img.isColor() && patch_img.isColor())
	{
		img.grayscale(im);
		patch_img.grayscale(patch_im);
	}
	else
	{
		ASSERT_(!img.isColor() && !patch_img.isColor());
		im = img.makeShallowCopy();
		patch_im = patch_img.makeShallowCopy();
	}

	const int im_w = im.getWidth();
	const int im_h = im.getHeight();
	const int patch_w = patch_im.getWidth();
	const int patch_h = patch_im.getHeight();

	if (entireImg)
	{
		x_search_size = im_w - patch_w;
		y_search_size = im_h - patch_h;
	}

	// JLBC: Perhaps is better to raise the exception always??
	if ((x_search_ini + x_search_size + patch_w) > im_w)
		x_search_size -= (x_search_ini + x_search_size + patch_w) - im_w;

	if ((y_search_ini + y_search_size + patch_h) > im_h)
		y_search_size -= (y_search_ini + y_search_size + patch_h) - im_h;

	ASSERT_((x_search_ini + x_search_size + patch_w) <= im_w);
	ASSERT_((y_search_ini + y_search_size + patch_h) <= im_h);
	CImage img_region_to_search;

	if (entireImg)
	{
		img_region_to_search = im.makeShallowCopy();
	}
	else
	{
		im.extract_patch(
			img_region_to_search,
			x_search_ini,  // start corner
			y_search_ini,
			patch_w + x_search_size,  // sub-image size
			patch_h + y_search_size);
	}

	cv::Mat result(cvSize(x_search_size + 1, y_search_size + 1), CV_32FC1);

	// Compute cross correlation:
	cv::matchTemplate(
		img_region_to_search.asCvMat<cv::Mat>(SHALLOW_COPY),
		patch_im.asCvMat<cv::Mat>(SHALLOW_COPY), result, CV_TM_CCORR_NORMED);

	// Find the max point:
	double mini;
	cv::Point min_point, max_point;

	cv::minMaxLoc(
		result, &mini, &max_val, &min_point, &max_point, cv::noArray());
	x_max = max_point.x + x_search_ini + (mrpt::round(patch_w - 1) >> 1);
	y_max = max_point.y + y_search_ini + (mrpt::round(patch_h - 1) >> 1);

#else
	THROW_EXCEPTION("The MRPT has been compiled with MRPT_HAS_OPENCV=0 !");
#endif

	MRPT_END
}

/*-------------------------------------------------------------
					pixelTo3D
-------------------------------------------------------------*/
TPoint3D vision::pixelTo3D(const TPixelCoordf& xy, const CMatrixDouble33& A)
{
	TPoint3D res;

	// Build the vector:
	res.x = xy.x - A(0, 2);
	res.y = xy.y - A(1, 2);
	res.z = A(0, 0);

	// Normalize:
	const double u = res.norm();
	ASSERT_(u != 0);
	res *= 1.0 / u;

	return res;
}

/*-------------------------------------------------------------
					buildIntrinsicParamsMatrix
-------------------------------------------------------------*/
CMatrixDouble33 vision::buildIntrinsicParamsMatrix(
	const double focalLengthX, const double focalLengthY, const double centerX,
	const double centerY)
{
	CMatrixDouble33 A;

	A(0, 0) = focalLengthX;
	A(1, 1) = focalLengthY;
	A(2, 2) = 1;

	A(0, 2) = centerX;
	A(1, 2) = centerY;

	return A;
}

/*-------------------------------------------------------------
					defaultIntrinsicParamsMatrix
-------------------------------------------------------------*/
CMatrixDouble33 vision::defaultIntrinsicParamsMatrix(
	unsigned int camIndex, unsigned int resX, unsigned int resY)
{
	float fx, fy, cx, cy;

	switch (camIndex)
	{
		case 0:
			// Bumblebee:
			fx = 0.79345f;
			fy = 1.05793f;
			cx = 0.55662f;
			cy = 0.52692f;
			break;

		case 1:
			// Sony:
			fx = 0.95666094f;
			fy = 1.3983423f;
			cx = 0.54626328f;
			cy = 0.4939191f;
			break;

		default:
		{
			THROW_EXCEPTION_FMT(
				"Unknown camera index!! for 'camIndex'=%u", camIndex);
		}
	}

	return buildIntrinsicParamsMatrix(
		resX * fx, resY * fy, resX * cx, resY * cy);
}

/*-------------------------------------------------------------
						computeMsd
-------------------------------------------------------------*/
double vision::computeMsd(
	const TMatchingPairList& feat_list, const poses::CPose3D& Rt)
{
	CMatrixDouble44 mat;
	Rt.getHomogeneousMatrix(mat);
	double acum = 0.0;

	TMatchingPairList::const_iterator it;
	TPoint3D err;
	for (it = feat_list.begin(); it != feat_list.end(); it++)
	{
		err.x = it->other_x - (it->this_x * mat(0, 0) + it->this_y * mat(0, 1) +
							   it->this_z * mat(0, 2) + Rt.x());
		err.y = it->other_y - (it->this_x * mat(1, 0) + it->this_y * mat(1, 1) +
							   it->this_z * mat(1, 2) + Rt.y());
		err.z = it->other_z - (it->this_x * mat(2, 0) + it->this_y * mat(2, 1) +
							   it->this_z * mat(2, 2) + Rt.z());

		acum += err.norm();

	}  // end for
	return (acum / feat_list.size());
}  // end msd

/*-------------------------------------------------------------
					cloudsToMatchedList
-------------------------------------------------------------*/
void vision::cloudsToMatchedList(
	const CObservationVisualLandmarks& cloud1,
	const CObservationVisualLandmarks& cloud2, TMatchingPairList& outList)
{
	CLandmarksMap::TCustomSequenceLandmarks::const_iterator itLand1, itLand2;
	TMatchingPair pair;

	for (itLand1 = cloud1.landmarks.landmarks.begin();
		 itLand1 != cloud1.landmarks.landmarks.end(); itLand1++)
		for (itLand2 = cloud2.landmarks.landmarks.begin();
			 itLand2 != cloud2.landmarks.landmarks.end(); itLand2++)
			if (itLand1->ID == itLand2->ID)
			{
				// Match found!
				pair.this_idx = pair.other_idx = (unsigned int)itLand1->ID;

				pair.this_x = itLand1->pose_mean.x;
				pair.this_y = itLand1->pose_mean.y;
				pair.this_z = itLand1->pose_mean.z;

				pair.other_x = itLand2->pose_mean.x;
				pair.other_y = itLand2->pose_mean.y;
				pair.other_z = itLand2->pose_mean.z;

				outList.push_back(pair);
			}  // end if
}  // end-cloudsToMatchedList

/*-------------------------------------------------------------
					computeMainOrientation
-------------------------------------------------------------*/
float vision::computeMainOrientation(
	const CImage& image, unsigned int x, unsigned int y)
{
	MRPT_START
	float orientation = 0;
	if ((int(x) - 1 >= 0) && (int(y) - 1 >= 0) && (x + 1 < image.getWidth()) &&
		(y + 1 < image.getHeight()))
		orientation = (float)atan2(
			(double)*image(x, y + 1) - (double)*image(x, y - 1),
			(double)*image(x + 1, y) - (double)*image(x - 1, y));

	// Convert from [-pi,pi] to [0,2pi]

	return orientation;

	MRPT_END
}  // end vision::computeMainOrientation

/*-------------------------------------------------------------
					normalizeImage
-------------------------------------------------------------*/
void vision::normalizeImage(const CImage& image, CImage& nimage)
{
	ASSERT_(image.getChannelCount() == 1);
	nimage.resize(image.getWidth(), image.getHeight(), image.getChannelCount());

	CMatrixFloat im, nim;
	nim.resize(image.getHeight(), image.getWidth());

	image.getAsMatrix(im);

	double m, s;
	mrpt::math::meanAndStd(im, m, s);

	for (int k1 = 0; k1 < (int)nim.cols(); ++k1)
		for (int k2 = 0; k2 < (int)nim.rows(); ++k2)
			nim(k2, k1) = (im(k2, k1) - m) / s;

	nimage.setFromMatrix(nim);
}

/*-------------------------------------------------------------
						matchFeatures
-------------------------------------------------------------*/
size_t vision::matchFeatures(
	const CFeatureList& list1, const CFeatureList& list2,
	CMatchedFeatureList& matches, const TMatchingOptions& options,
	const TStereoSystemParams& params)
{
	// Clear the output structure
	MRPT_START
	// matches.clear();

	// Preliminary comprobations
	size_t sz1 = list1.size(), sz2 = list2.size();

	ASSERT_((sz1 > 0) && (sz2 > 0));  // Both lists have features within it
	ASSERT_(
		list1.get_type() ==
		list2.get_type());  // Both lists must be of the same type

	CFeatureList::const_iterator itList1, itList2;  // Iterators for the lists

	// For SIFT & SURF
	float distDesc;  // EDD or EDSD
	float minDist1;  // Minimum EDD or EDSD
	float minDist2;  // Second minimum EDD or EDSD

	// For Harris
	double maxCC1;  // Maximum CC
	double maxCC2;  // Second maximum CC

	// For SAD
	double minSAD1, minSAD2;

	vector<int> idxLeftList, idxRightList;
	idxLeftList.resize(sz1, FEAT_FREE);
	idxRightList.resize(sz2, FEAT_FREE);
	vector<double> distCorrs(sz1);
	int lFeat, rFeat;
	int minLeftIdx = 0, minRightIdx;
	int nMatches = 0;

	// For each feature in list1 ...
	for (lFeat = 0, itList1 = list1.begin(); itList1 != list1.end();
		 ++itList1, ++lFeat)
	{
		// For SIFT & SURF
		minDist1 = 1e5;
		minDist2 = 1e5;

		// For Harris
		maxCC1 = 0;
		maxCC2 = 0;

		// For SAD
		minSAD1 = 1e5;
		minSAD2 = 1e5;

		// For all the cases
		minRightIdx = 0;

		for (rFeat = 0, itList2 = list2.begin(); itList2 != list2.end();
			 ++itList2, ++rFeat)  // ... compare with all the features in list2.
		{
			// Filter out by epipolar constraint
			double d = 0.0;  // Distance to the epipolar line
			if (options.useEpipolarRestriction)
			{
				if (options.parallelOpticalAxis)
					d = itList1->keypoint.pt.y - itList2->keypoint.pt.y;
				else
				{
					ASSERT_(options.hasFundamentalMatrix);

					// Compute epipolar line Ax + By + C = 0
					TLine2D epiLine;
					TPoint2D oPoint(
						itList2->keypoint.pt.x, itList2->keypoint.pt.y);

					CMatrixDouble31 l, p;
					p(0, 0) = itList1->keypoint.pt.x;
					p(1, 0) = itList1->keypoint.pt.y;
					p(2, 0) = 1;

					l = params.F * p;

					epiLine.coefs[0] = l(0, 0);
					epiLine.coefs[1] = l(1, 0);
					epiLine.coefs[2] = l(2, 0);

					d = epiLine.distance(oPoint);
				}  // end else
			}  // end if

			// Use epipolar restriction
			bool c1 = options.useEpipolarRestriction
						  ? fabs(d) < options.epipolar_TH
						  : true;
			// Use x-coord restriction
			bool c2 = options.useXRestriction
						  ? itList1->keypoint.pt.x - itList2->keypoint.pt.x > 0
						  : true;

			if (c1 && c2)
			{
				switch (options.matching_method)
				{
					case TMatchingOptions::mmDescriptorSIFT:
					{
						// Ensure that both features have SIFT descriptors
						ASSERT_(
							itList1->descriptors.hasDescriptorSIFT() &&
							itList2->descriptors.hasDescriptorSIFT());

						// Compute the Euclidean distance between descriptors
						distDesc = itList1->descriptorSIFTDistanceTo(*itList2);

						// Search for the two minimum values
						if (distDesc < minDist1)
						{
							minDist2 = minDist1;
							minDist1 = distDesc;
							minLeftIdx = lFeat;
							minRightIdx = rFeat;
						}
						else if (distDesc < minDist2)
							minDist2 = distDesc;

						break;
					}  // end mmDescriptorSIFT

					case TMatchingOptions::mmCorrelation:
					{
						size_t u, v;  // Coordinates of the peak
						double res;  // Value of the peak

						// Ensure that both features have patches
						ASSERT_(
							itList1->patchSize > 0 && itList2->patchSize > 0);
						vision::openCV_cross_correlation(
							*itList1->patch, *itList2->patch, u, v, res);

						// Search for the two maximum values
						if (res > maxCC1)
						{
							maxCC2 = maxCC1;
							maxCC1 = res;
							minLeftIdx = lFeat;
							minRightIdx = rFeat;
						}
						else if (res > maxCC2)
							maxCC2 = res;

						break;
					}  // end mmCorrelation

					case TMatchingOptions::mmDescriptorSURF:
					{
						// Ensure that both features have SURF descriptors
						ASSERT_(
							itList1->descriptors.hasDescriptorSURF() &&
							itList2->descriptors.hasDescriptorSURF());

						// Compute the Euclidean distance between descriptors
						distDesc = itList1->descriptorSURFDistanceTo(*itList2);

						// Search for the two minimum values
						if (distDesc < minDist1)
						{
							minDist2 = minDist1;
							minDist1 = distDesc;
							minLeftIdx = lFeat;
							minRightIdx = rFeat;
						}
						else if (distDesc < minDist2)
							minDist2 = distDesc;

						break;  // end case featSURF
					}  // end mmDescriptorSURF

					case TMatchingOptions::mmDescriptorORB:
					{
						// Ensure that both features have SURF descriptors
						ASSERT_(
							itList1->descriptors.hasDescriptorORB() &&
							itList2->descriptors.hasDescriptorORB());
						distDesc = itList1->descriptorORBDistanceTo(*itList2);

						// Search for the two minimum values
						if (distDesc < minDist1)
						{
							minDist2 = minDist1;
							minDist1 = distDesc;
							minLeftIdx = lFeat;
							minRightIdx = rFeat;
						}
						else if (distDesc < minDist2)
							minDist2 = distDesc;

						break;
					}  // end mmDescriptorORB

					case TMatchingOptions::mmSAD:
					{
						// Ensure that both features have patches
						ASSERT_(
							itList1->patchSize > 0 &&
							itList2->patchSize == itList1->patchSize);
#if !MRPT_HAS_OPENCV
						THROW_EXCEPTION(
							"MRPT has been compiled without OpenCV");
#else
						const CImage aux1(
							*itList1->patch, FAST_REF_OR_CONVERT_TO_GRAY);
						const CImage aux2(
							*itList2->patch, FAST_REF_OR_CONVERT_TO_GRAY);
						const auto h = aux1.getHeight(), w = aux1.getWidth();

						double res = 0;
						for (unsigned int ii = 0; ii < h; ++ii)
							for (unsigned int jj = 0; jj < w; ++jj)
								res += std::abs(
									static_cast<double>(
										aux1.at<uint8_t>(jj, ii)) -
									static_cast<double>(
										aux2.at<uint8_t>(jj, ii)));
						res = res / (255.0 * w * h);

						if (res < minSAD1)
						{
							minSAD2 = minSAD1;
							minSAD1 = res;
							minLeftIdx = lFeat;
							minRightIdx = rFeat;
						}
						else if (res < minSAD2)
							minSAD2 = res;
#endif
						break;
					}  // end mmSAD
				}  // end switch
			}  // end if
		}  // end for 'list2' (right features)

		bool cond1 = false, cond2 = false;
		double minVal = 1.0;
		switch (options.matching_method)
		{
			case TMatchingOptions::mmDescriptorSIFT:
				cond1 = minDist1 < options.maxEDD_TH;  // Maximum Euclidean
				// Distance between SIFT
				// descriptors (EDD)
				cond2 = (minDist1 / minDist2) <
						options.EDD_RATIO;  // Ratio between the two lowest EDSD
				minVal = minDist1;
				break;
			case TMatchingOptions::mmCorrelation:
				cond1 = maxCC1 >
						options.minCC_TH;  // Minimum cross correlation value
				cond2 = (maxCC2 / maxCC1) <
						options.rCC_TH;  // Ratio between the two highest cross
				// correlation values
				minVal = 1 - maxCC1;
				break;
			case TMatchingOptions::mmDescriptorSURF:
				cond1 = minDist1 < options.maxEDSD_TH;  // Maximum Euclidean
				// Distance between SURF
				// descriptors (EDSD)
				cond2 =
					(minDist1 / minDist2) <
					options.EDSD_RATIO;  // Ratio between the two lowest EDSD
				minVal = minDist1;
				break;
			case TMatchingOptions::mmSAD:
				cond1 = minSAD1 < options.maxSAD_TH;
				cond2 = (minSAD1 / minSAD2) < options.SAD_RATIO;
				minVal = minSAD1;
				break;
			case TMatchingOptions::mmDescriptorORB:
				cond1 = minDist1 < options.maxORB_dist;
				cond2 = true;
				minVal = minDist1;
				break;
			default:
				THROW_EXCEPTION("Invalid value of 'matching_method'");
		}

		// PROCESS THE RESULTS
		if (cond1 && cond2)  // The minimum distance must be below a threshold
		{
			int auxIdx = idxRightList[minRightIdx];
			if (auxIdx != FEAT_FREE)
			{
				if (distCorrs[auxIdx] > minVal)
				{
					// We've found a better match
					distCorrs[minLeftIdx] = minVal;
					idxLeftList[minLeftIdx] = minRightIdx;
					idxRightList[minRightIdx] = minLeftIdx;

					distCorrs[auxIdx] = 1.0;
					idxLeftList[auxIdx] = FEAT_FREE;
				}  // end-if
			}  // end-if
			else
			{
				idxRightList[minRightIdx] = minLeftIdx;
				idxLeftList[minLeftIdx] = minRightIdx;
				distCorrs[minLeftIdx] = minVal;
				nMatches++;
			}
		}  // end if
	}  // end for 'list1' (left features)

	if (!options.addMatches) matches.clear();

	TFeatureID idLeft = 0, idRight = 0;
	if (!matches.empty()) matches.getMaxID(bothLists, idLeft, idRight);

	for (int vCnt = 0; vCnt < (int)idxLeftList.size(); ++vCnt)
	{
		if (idxLeftList[vCnt] != FEAT_FREE)
		{
			std::pair<CFeature, CFeature> thisMatch;

			bool isGood = true;
			double dp1 = -1.0, dp2 = -1.0;
			TPoint3D p3D = TPoint3D();
			if (options.estimateDepth && options.parallelOpticalAxis)
			{
				projectMatchedFeature(
					list1[vCnt], list2[idxLeftList[vCnt]], p3D, params);
				dp1 = sqrt(p3D.x * p3D.x + p3D.y * p3D.y + p3D.z * p3D.z);
				dp2 = sqrt(
					(p3D.x - params.baseline) * (p3D.x - params.baseline) +
					p3D.y * p3D.y + p3D.z * p3D.z);

				if (dp1 > options.maxDepthThreshold ||
					dp2 > options.maxDepthThreshold)
					isGood = false;
			}  // end-if

			if (isGood)
			{
				// Set the features
				thisMatch.first = list1[vCnt];
				thisMatch.second = list2[idxLeftList[vCnt]];

				// Update the max ID value
				if (matches.empty())
				{
					idLeft = thisMatch.first.keypoint.ID;
					idRight = thisMatch.second.keypoint.ID;
				}
				else
				{
					keep_max(idLeft, thisMatch.first.keypoint.ID);
					matches.setLeftMaxID(idLeft);

					keep_max(idRight, thisMatch.second.keypoint.ID);
					matches.setRightMaxID(idRight);
				}

				// Set the depth and the 3D position of the feature
				if (options.estimateDepth && options.parallelOpticalAxis)
				{
					thisMatch.first.initialDepth = dp1;
					thisMatch.first.p3D = p3D;

					thisMatch.second.initialDepth = dp2;
					thisMatch.second.p3D =
						TPoint3D(p3D.x - params.baseline, p3D.y, p3D.z);
				}  // end-if

				// Insert the match into the matched list
				matches.push_back(thisMatch);
			}  // end-if-isGood
		}  // end-if
	}  // end-for-matches
	return matches.size();

	MRPT_END
}

/*-------------------------------------------------------------
			generateMask
-------------------------------------------------------------*/
// Insert zeros around the points in mList according to wSize
void vision::generateMask(
	const CMatchedFeatureList& mList, CMatrixBool& mask1, CMatrixBool& mask2,
	int wSize)
{
	ASSERT_(mList.size() > 0);

	//    cv::Mat *mask1 = static_cast<cv::Mat*>(_mask1);
	//    cv::Mat *mask2 = static_cast<cv::Mat*>(_mask2);

	int hwsize = (int)(0.5 * wSize);
	int mx = mask1.cols(), my = mask1.rows();

	int idx, idy;
	CMatchedFeatureList::const_iterator it;
	for (it = mList.begin(); it != mList.end(); ++it)
	{
		for (int ii = -hwsize; ii < hwsize; ++ii)
			for (int jj = -hwsize; jj < hwsize; ++jj)
			{
				idx = (int)(it->first.keypoint.pt.x) + ii;
				idy = (int)(it->first.keypoint.pt.y) + jj;
				if (idx >= 0 && idy >= 0 && idx < mx && idy < my)
					mask1(idy, idx) = false;
			}

		for (int ii = -hwsize; ii < hwsize; ++ii)
			for (int jj = -hwsize; jj < hwsize; ++jj)
			{
				idx = (int)(it->second.keypoint.pt.x) + ii;
				idy = (int)(it->second.keypoint.pt.y) + jj;
				if (idx >= 0 && idy >= 0 && idx < mx && idy < my)
					mask2(idy, idx) = false;
			}
	}  // end-for
}  // end generateMask

double vision::computeSAD(const CImage& p1, const CImage& p2)
{
	MRPT_START
#if MRPT_HAS_OPENCV
	ASSERT_(p1.getSize() == p2.getSize());
	const auto w = p1.getWidth(), h = p1.getHeight();
	double res = 0.0;
	for (unsigned int ii = 0; ii < h; ++ii)
		for (unsigned int jj = 0; jj < w; ++jj)
			res += std::abs(
				static_cast<double>(p1.at<uint8_t>(jj, ii)) -
				static_cast<double>(p2.at<uint8_t>(jj, ii)));

	return res / (255.0 * w * h);
#else
	THROW_EXCEPTION(
		"MRPT compiled without OpenCV, can't compute SAD of images!");
#endif
	MRPT_END
}

void vision::addFeaturesToImage(
	const CImage& inImg, const CFeatureList& theList, CImage& outImg)
{
	outImg = inImg;  // Create a copy of the input image
	for (const auto& it : theList)
		outImg.rectangle(
			it.keypoint.pt.x - 5, it.keypoint.pt.y - 5, it.keypoint.pt.x + 5,
			it.keypoint.pt.y + 5, TColor(255, 0, 0));
}

/*-------------------------------------------------------------
					projectMatchedFeatures
-------------------------------------------------------------*/
void vision::projectMatchedFeatures(
	const CMatchedFeatureList& matches,
	const mrpt::img::TStereoCamera& stereo_camera, vector<TPoint3D>& out_points)
{
	out_points.clear();
	out_points.reserve(matches.size());
	for (const auto& match : matches)
	{
		const auto& pt1 = match.first.keypoint.pt;
		const auto& pt2 = match.second.keypoint.pt;

		const double disp = pt1.x - pt2.x;
		if (disp < 1) continue;

		const double b_d = stereo_camera.rightCameraPose.x / disp;
		out_points.emplace_back(
			(pt1.x - stereo_camera.leftCamera.cx()) * b_d,
			(pt1.y - stereo_camera.leftCamera.cy()) * b_d,
			stereo_camera.leftCamera.fx() * b_d);
	}
}
/*-------------------------------------------------------------
					projectMatchedFeatures
-------------------------------------------------------------*/
void vision::projectMatchedFeatures(
	const CFeatureList& leftList, const CFeatureList& rightList,
	vector<TPoint3D>& vP3D, const TStereoSystemParams& params)
{
	vP3D.reserve(leftList.size());
	CFeatureList::const_iterator it1, it2;
	for (it1 = leftList.begin(), it2 = rightList.begin(); it1 != leftList.end();
		 ++it1, ++it2)
	{
		TPoint3D p3D;
		projectMatchedFeature(*it1, *it2, p3D, params);
		if (p3D.z < params.maxZ && p3D.z > params.minZ && p3D.y < params.maxY)
			vP3D.push_back(p3D);
	}
}  // end projectMatchedFeatures

/*-------------------------------------------------------------
					projectMatchedFeatures
-------------------------------------------------------------*/
void vision::projectMatchedFeature(
	const CFeature& leftFeat, const CFeature& rightFeat, TPoint3D& p3D,
	const TStereoSystemParams& params)
{
	const double f0 = 600;
	double nfx1 = leftFeat.keypoint.pt.x, nfy1 = leftFeat.keypoint.pt.y,
		   nfx2 = rightFeat.keypoint.pt.x, nfy2 = rightFeat.keypoint.pt.y;

	const double x = nfx1 * f0;  // x  = (x  / f0) * f0   x  = x
	const double y = nfy1 * f0;  // y  = (y  / f0) * f0   y  = y
	const double xd = nfx2 * f0;  // x' = (x' / f0) * f0   x' = x'
	const double yd = nfy2 * f0;  // y' = (y' / f0) * f0   y' = y'

	const double f2 = f0 * f0;
	const double p9 = f2 * params.F(2, 2);
	const double Q00 =
		f2 *
		(params.F(0, 2) * params.F(0, 2) + params.F(1, 2) * params.F(1, 2) +
		 params.F(2, 0) * params.F(2, 0) + params.F(2, 1) * params.F(2, 1));

	double Jh = (std::numeric_limits<double>::max)();  // J hat = 
	double xh = x;  // x hat = x
	double yh = y;  // y hat = y
	double xhd = xd;  // x hat dash = x'
	double yhd = yd;  // y hat dash = y'
	double
		xt = 0,
		yt = 0, xtd = 0,
		ytd =
			0;  // x tilde = 0, y tilde = 0, x tilde dash = 0, y tilde dash = 0
	for (;;)
	{
		const double p1 = (xh * xhd + xhd * xt + xh * xtd) * params.F(0, 0);
		const double p2 = (xh * yhd + yhd * xt + xh * ytd) * params.F(0, 1);
		const double p3 = (f0 * (xh + xt)) * params.F(0, 2);
		const double p4 = (yh * xhd + xhd * yt + yh * xtd) * params.F(1, 0);
		const double p5 = (yh * yhd + yhd * yt + yh * ytd) * params.F(1, 1);
		const double p6 = (f0 * (yh + yt)) * params.F(1, 2);
		const double p7 = (f0 * (xhd + xtd)) * params.F(2, 0);
		const double p8 = (f0 * (yhd + ytd)) * params.F(2, 1);

		const double udotxi = p1 + p2 + p3 + p4 + p5 + p6 + p7 + p8 + p9;

		const double Q11 =
			(xh * xh + xhd * xhd) * params.F(0, 0) * params.F(0, 0);
		const double Q22 =
			(xh * xh + yhd * yhd) * params.F(0, 1) * params.F(0, 1);
		const double Q44 =
			(yh * yh + xhd * xhd) * params.F(1, 0) * params.F(1, 0);
		const double Q55 =
			(yh * yh + yhd * yhd) * params.F(1, 1) * params.F(1, 1);
		const double Q12 = xhd * yhd * params.F(0, 0) * params.F(0, 1);
		const double Q13 = f0 * xhd * params.F(0, 0) * params.F(0, 2);
		const double Q14 = xh * yh * params.F(0, 0) * params.F(1, 0);
		const double Q17 = f0 * xh * params.F(0, 0) * params.F(2, 0);
		const double Q23 = f0 * yhd * params.F(0, 1) * params.F(0, 2);
		const double Q25 = xh * yh * params.F(0, 1) * params.F(1, 1);
		const double Q28 = f0 * xh * params.F(0, 1) * params.F(2, 1);
		const double Q45 = xhd * yhd * params.F(1, 0) * params.F(1, 1);
		const double Q46 = f0 * xhd * params.F(1, 0) * params.F(1, 2);
		const double Q47 = f0 * yh * params.F(1, 0) * params.F(2, 0);
		const double Q56 = f0 * yhd * params.F(1, 1) * params.F(1, 2);
		const double Q58 = f0 * yh * params.F(1, 1) * params.F(2, 1);

		const double udotV0xiu = Q00 + Q11 + Q22 + Q44 + Q55 +
								 2.0 * (Q12 + Q13 + Q14 + Q17 + Q23 + Q25 +
										Q28 + Q45 + Q46 + Q47 + Q56 + Q58);

		ASSERT_(fabs(udotV0xiu) > 1e-5);

		const double C = udotxi / udotV0xiu;

		xt = C * (params.F(0, 0) * xhd + params.F(0, 1) * yhd +
				  f0 * params.F(0, 2));
		yt = C * (params.F(1, 0) * xhd + params.F(1, 1) * yhd +
				  f0 * params.F(1, 2));
		xtd = C *
			  (params.F(0, 0) * xh + params.F(1, 0) * yh + f0 * params.F(2, 0));
		ytd = C *
			  (params.F(0, 1) * xh + params.F(1, 1) * yh + f0 * params.F(2, 1));

		const double Jt = xt * xt + yt * yt + xtd * xtd + ytd * ytd;
		//        cout << "Jt:" << Jt << " and Jh: " << Jh << endl;
		if ((std::abs)(Jt - Jh) <= 1e-5)
		{
			nfx1 = xh / f0;
			nfy1 = yh / f0;
			nfx2 = xhd / f0;
			// nfy2 = yhd / f0;

			break;
		}
		else
		{
			Jh = Jt;  // J hat      = J tilde
			xh = x - xt;  // x hat      = x  - x tilde
			yh = y - yt;  // y hat      = y  - y tilde
			xhd = xd - xtd;  // x hat dash = x' - x tilde dash
			yhd = yd - ytd;  // y hat dash = y' - y tilde dash
		}
	}  // end for

	double disp = nfx1 - nfx2;
	double aux = params.baseline / disp;
	p3D.x = (nfx1 - params.K(0, 2)) * aux;
	p3D.y = (nfy1 - params.K(1, 2)) * aux;
	p3D.z = params.K(0, 0) * aux;
}  // end projectMatchedFeature

/*-------------------------------------------------------------
					projectMatchedFeatures
-------------------------------------------------------------*/
void vision::projectMatchedFeatures(
	CMatchedFeatureList& mfList, const TStereoSystemParams& param,
	CLandmarksMap& landmarks)
{
	MRPT_START

	landmarks.clear();  // Assert that the output CLandmarksMap is clear

	float stdPixel2 = square(param.stdPixel);
	float stdDisp2 = square(param.stdDisp);

	// Main loop
	for (auto itList = mfList.begin(); itList != mfList.end();)
	{
		const auto& pt1 = itList->first.keypoint.pt;
		const auto& pt2 = itList->second.keypoint.pt;

		const float disp = pt1.x - pt2.x;  // Disparity
		if (disp < 1e-9)
		{
			// Filter out too far points
			// Erase the match
			itList = mfList.erase(itList);
			continue;
		}

		float x3D = (pt1.x - param.K(0, 2)) * ((param.baseline)) / disp;
		float y3D = (pt1.y - param.K(1, 2)) * ((param.baseline)) / disp;
		float z3D = (param.K(0, 0)) * ((param.baseline)) / disp;

		// Filter out bad points
		if ((z3D < param.minZ) || (z3D > param.maxZ))
		{
			itList = mfList.erase(itList);
		}
		else
		{
			TPoint3D p3D(x3D, y3D, z3D);

			// STORE THE OBTAINED LANDMARK
			CLandmark lm;

			TPoint3D norm3D = p3D;
			norm3D *= -1 / norm3D.norm();

			lm.normal = norm3D;
			lm.pose_mean = p3D;
			lm.ID = itList->first.keypoint.ID;

			// If the matched landmarks has a (SIFT or SURF) descriptor,
			// asign the left one to the landmark.
			// TO DO: Assign the mean value of the descriptor (between the
			// matches)
			lm.features.resize(1);
			lm.features[0] = (*itList).first;

			// Compute the covariance matrix for the landmark
			switch (param.uncPropagation)
			{
				case TStereoSystemParams::Prop_Linear:
				{
					float foc2 = square(param.K(0, 0));
					float c0 = param.K(0, 2);
					float r0 = param.K(1, 2);
					float base2 = square(param.baseline);
					float disp2 = square(disp);

					lm.pose_cov_11 =
						stdPixel2 * base2 / disp2 +
						stdDisp2 * base2 * square(pt1.x - c0) / square(disp2);
					lm.pose_cov_12 = stdDisp2 * base2 * (pt1.x - c0) *
									 (pt1.y - r0) / square(disp2);
					lm.pose_cov_13 = stdDisp2 * base2 * sqrt(foc2) *
									 (pt1.x - c0) / square(disp2);
					lm.pose_cov_22 =
						stdPixel2 * base2 / disp2 +
						stdDisp2 * base2 * square(pt1.y - r0) / square(disp2);
					lm.pose_cov_23 = stdDisp2 * base2 * sqrt(foc2) *
									 (pt1.y - r0) / square(disp2);
					lm.pose_cov_33 = stdDisp2 * foc2 * base2 / square(disp2);
				}  // end case 'Prop_Linear'
				break;

				case TStereoSystemParams::Prop_UT:
				{
					// Parameters
					unsigned int Na = 3;
					unsigned int i;

					float k = param.factor_k;

					float w0 = k / (Na + k);
					float w1 = 1 / (2 * (Na + k));

					CMatrixF Pa(3, 3);
					CMatrixF L(3, 3);

					Pa.fill(0);
					Pa(0, 0) = Pa(1, 1) = (Na + k) * square(param.stdPixel);
					Pa(2, 2) = (Na + k) * square(param.stdDisp);

					// Cholesky decomposition
					Pa.chol(L);  // math::chol(Pa,L);

					vector<TPoint3D> B;  // B group
					TPoint3D meanB;  // Mean value of the B group
					CMatrixF Pb;  // Covariance of the B group

					B.resize(2 * Na + 1);  // Set of output values
					Pb.fill(0);  // Reset the output covariance

					CVectorFloat vAux, myPoint;  // Auxiliar vectors
					CVectorFloat meanA;  // Mean value of the A group

					vAux.resize(3);  // Set the variables size
					meanA.resize(3);
					myPoint.resize(3);

					// Mean input value: (c,r,d)
					meanA[0] = pt1.x;
					meanA[1] = pt1.y;
					meanA[2] = disp;

					// Output mean
					meanB.x = w0 * x3D;
					meanB.y = w0 * y3D;
					meanB.z = w0 * z3D;  // Add to the mean
					B[0].x = x3D;
					B[0].y = y3D;
					B[0].z = z3D;  // Insert into B

					for (i = 1; i <= 2 * Na; i++)
					{
						// Form the Ai value
						if (i <= Na)
						{
							// Extract the proper row
							vAux.asEigen() = L.asEigen().row(i - 1);
							myPoint[0] = meanA[0] + vAux[0];
							myPoint[1] = meanA[1] + vAux[1];
							myPoint[2] = meanA[2] + vAux[2];
						}
						else
						{
							vAux.asEigen() = L.asEigen().row((i - Na) - 1);
							myPoint[0] = meanA[0] - vAux[0];
							myPoint[1] = meanA[1] - vAux[1];
							myPoint[2] = meanA[2] - vAux[2];
						}

						// Pass the Ai through the functions:
						x3D = (myPoint[0] - param.K(0, 2)) *
							  ((param.baseline)) / myPoint[2];
						y3D = (myPoint[1] - param.K(1, 2)) *
							  ((param.baseline)) / myPoint[2];
						z3D = (param.K(0, 0)) * ((param.baseline)) / myPoint[2];

						// Add to the B mean computation and the B vector
						meanB.x = meanB.x + w1 * x3D;
						meanB.y = meanB.y + w1 * y3D;
						meanB.z = meanB.z + w1 * z3D;

						B[i].x = x3D;
						B[i].y = y3D;
						B[i].z = z3D;

					}  // end for 'i'

					// Output covariance
					for (i = 0; i <= 2 * Na; i++)
					{
						float weight = w1;
						CMatrixF v(3, 1);

						if (i == 0)  // The weight for the mean value of A is w0
							weight = w0;

						v(0, 0) = B[i].x - meanB.x;
						v(1, 0) = B[i].y - meanB.y;
						v(2, 0) = B[i].z - meanB.z;

						Pb.asEigen() +=
							(weight * (v.asEigen() * v.transpose())).eval();
					}  // end for 'i'

					// Store it in the landmark
					lm.pose_cov_11 = Pb(0, 0);
					lm.pose_cov_12 = Pb(0, 1);
					lm.pose_cov_13 = Pb(0, 2);
					lm.pose_cov_22 = Pb(1, 1);
					lm.pose_cov_23 = Pb(1, 2);
					lm.pose_cov_33 = Pb(2, 2);
				}  // end case 'Prop_UT'
				break;

				case TStereoSystemParams::Prop_SUT:
				{
					// Parameters
					unsigned int Na = 3;
					unsigned int i;

					float a = param.factor_a;
					float b = param.factor_b;
					float k = param.factor_k;

					float lambda = square(a) * (Na + k) - Na;

					float w0_m = lambda / (Na + lambda);
					float w0_c = w0_m + (1 - square(a) + b);
					float w1 = 1 / (2 * (Na + lambda));

					CMatrixF Pa(3, 3);
					CMatrixF L(3, 3);

					Pa.fill(0);
					Pa(0, 0) = Pa(1, 1) =
						(Na + lambda) * square(param.stdPixel);
					Pa(2, 2) = (Na + lambda) * square(param.stdDisp);

					// Cholesky decomposition
					Pa.chol(L);  // math::chol(Pa,L);

					vector<TPoint3D> B;  // B group
					TPoint3D meanB;  // Mean value of the B group
					CMatrixF Pb;  // Covariance of the B group

					B.resize(2 * Na + 1);  // Set of output values
					Pb.fill(0);  // Reset the output covariance

					CVectorFloat vAux, myPoint;  // Auxiliar vectors
					CVectorFloat meanA;  // Mean value of the A group

					vAux.resize(3);  // Set the variables size
					meanA.resize(3);
					myPoint.resize(3);

					// Mean input value: (c,r,d)
					meanA[0] = pt1.x;
					meanA[1] = pt1.y;
					meanA[2] = disp;

					// Output mean
					meanB.x = w0_m * x3D;
					meanB.y = w0_m * y3D;
					meanB.z = w0_m * z3D;  // Add to the mean
					B[0].x = x3D;
					B[0].y = y3D;
					B[0].z = z3D;  // Insert into B

					for (i = 1; i <= 2 * Na; i++)
					{
						// Form the Ai value
						if (i <= Na)
						{
							// Extract the proper row
							vAux.asEigen() = L.row(i - 1);
							myPoint = meanA + vAux;
							// myPoint[0] = meanA[0] + vAux[0];
							// myPoint[1] = meanA[1] + vAux[1];
							// myPoint[2] = meanA[2] + vAux[2];
						}
						else
						{
							vAux = L.row((i - Na) - 1);
							myPoint = meanA - vAux;
							// myPoint[0] = meanA[0] - vAux[0];
							// myPoint[1] = meanA[1] - vAux[1];
							// myPoint[2] = meanA[2] - vAux[2];
						}

						// Pass the Ai through the functions:
						x3D = (myPoint[0] - param.K(0, 2)) *
							  ((param.baseline)) / myPoint[2];
						y3D = (myPoint[1] - param.K(1, 2)) *
							  ((param.baseline)) / myPoint[2];
						z3D = (param.K(0, 0)) * ((param.baseline)) / myPoint[2];

						// Add to the B mean computation and the B vector
						meanB.x = meanB.x + w1 * x3D;
						meanB.y = meanB.y + w1 * y3D;
						meanB.z = meanB.z + w1 * z3D;

						B[i].x = x3D;
						B[i].y = y3D;
						B[i].z = z3D;

					}  // end for 'i'

					// Output covariance
					for (i = 0; i <= 2 * Na; i++)
					{
						float weight = w1;
						CMatrixF v(3, 1);

						if (i == 0)  // The weight for the mean value of A is w0
							weight = w0_c;

						v(0, 0) = B[i].x - meanB.x;
						v(1, 0) = B[i].y - meanB.y;
						v(2, 0) = B[i].z - meanB.z;

						Pb.asEigen() +=
							(weight * (v.asEigen() * v.transpose())).eval();
					}  // end for 'i'

					// Store it in the landmark
					lm.pose_cov_11 = Pb(0, 0);
					lm.pose_cov_12 = Pb(0, 1);
					lm.pose_cov_13 = Pb(0, 2);
					lm.pose_cov_22 = Pb(1, 1);
					lm.pose_cov_23 = Pb(1, 2);
					lm.pose_cov_33 = Pb(2, 2);
				}  // end case 'Prop_SUT'
				break;

			}  // end switch
			landmarks.landmarks.push_back(lm);
			itList++;
		}  // end else ( (z3D > param.minZ) && (z3D < param.maxZ) )
	}  // end for 'i'

	MRPT_END
}  // end of projectMatchedFeatures

/*-------------------------------------------------------------
					projectMatchedFeatures
-------------------------------------------------------------*/
void vision::projectMatchedFeatures(
	CFeatureList& leftList, CFeatureList& rightList,
	const vision::TStereoSystemParams& param,
	mrpt::maps::CLandmarksMap& landmarks)
{
	MRPT_START
	ASSERT_(leftList.size() == rightList.size());

	landmarks.clear();  // Assert that the output CLandmarksMap is clear

	CFeatureList::iterator itListL, itListR;
	float stdPixel2 = square(param.stdPixel);
	float stdDisp2 = square(param.stdDisp);

	// Main loop
	for (itListL = leftList.begin(), itListR = rightList.begin();
		 itListL != leftList.end();)
	{
		const auto& ptL = itListL->keypoint.pt;
		const auto& ptR = itListR->keypoint.pt;

		float disp = ptL.x - ptR.x;  // Disparity
		if (disp < 1e-9)  // Filter out too far points
		{
			itListL = leftList.erase(itListL);
			itListR = rightList.erase(itListR);
			continue;
		}

		// Too much distant features are not taken into account
		float x3D = (ptL.x - param.K(0, 2)) * param.baseline / disp;
		float y3D = (ptL.y - param.K(1, 2)) * param.baseline / disp;
		float z3D = (param.K(0, 0)) * param.baseline / disp;

		// Filter out bad points
		if ((z3D < param.minZ) || (z3D > param.maxZ))
		{
			itListL = leftList.erase(itListL);
			itListR = rightList.erase(itListR);
		}
		else
		{
			TPoint3D p3D(x3D, y3D, z3D);

			// STORE THE OBTAINED LANDMARK
			CLandmark lm;

			TPoint3D norm3D = p3D;
			norm3D *= -1. / norm3D.norm();

			lm.normal = norm3D;
			lm.pose_mean = p3D;
			lm.ID = itListL->keypoint.ID;

			// If the matched landmarks has a (SIFT or SURF) descriptor,
			// asign the left one to the landmark.
			// TO DO: Assign the mean value of the descriptor (between the
			// matches)
			lm.features.resize(2);
			lm.features[0] = *itListL;
			lm.features[1] = *itListR;

			// Compute the covariance matrix for the landmark
			switch (param.uncPropagation)
			{
				case TStereoSystemParams::Prop_Linear:
				{
					float foc2 = square(param.K(0, 0));
					float c0 = param.K(0, 2);
					float r0 = param.K(1, 2);
					float base2 = square(param.baseline);
					float disp2 = square(ptL.x - ptR.x);

					lm.pose_cov_11 =
						stdPixel2 * base2 / disp2 +
						stdDisp2 * base2 * square(ptL.x - c0) / square(disp2);
					lm.pose_cov_12 = stdDisp2 * base2 * (ptL.x - c0) *
									 (ptL.y - r0) / square(disp2);
					lm.pose_cov_13 = stdDisp2 * base2 * sqrt(foc2) *
									 (ptL.x - c0) / square(disp2);
					lm.pose_cov_22 =
						stdPixel2 * base2 / disp2 +
						stdDisp2 * base2 * square(ptL.y - r0) / square(disp2);
					lm.pose_cov_23 = stdDisp2 * base2 * sqrt(foc2) *
									 (ptL.y - r0) / square(disp2);
					lm.pose_cov_33 = stdDisp2 * foc2 * base2 / square(disp2);
				}  // end case 'Prop_Linear'
				break;

				case TStereoSystemParams::Prop_UT:
				{
					// Parameters
					unsigned int Na = 3;
					unsigned int i;

					float k = param.factor_k;

					float w0 = k / (Na + k);
					float w1 = 1 / (2 * (Na + k));

					CMatrixF Pa(3, 3);
					CMatrixF L(3, 3);

					Pa.fill(0);
					Pa(0, 0) = Pa(1, 1) = (Na + k) * square(param.stdPixel);
					Pa(2, 2) = (Na + k) * square(param.stdDisp);

					// Cholesky decomposition
					Pa.chol(L);  // math::chol(Pa,L);

					vector<TPoint3D> B;  // B group
					TPoint3D meanB;  // Mean value of the B group
					CMatrixF Pb;  // Covariance of the B group

					B.resize(2 * Na + 1);  // Set of output values
					Pb.fill(0);  // Reset the output covariance

					CVectorFloat vAux, myPoint;  // Auxiliar vectors
					CVectorFloat meanA;  // Mean value of the A group

					vAux.resize(3);  // Set the variables size
					meanA.resize(3);
					myPoint.resize(3);

					// Mean input value: (c,r,d)
					meanA[0] = ptL.x;
					meanA[1] = ptL.y;
					meanA[2] = disp;

					// Output mean
					meanB.x = w0 * x3D;
					meanB.y = w0 * y3D;
					meanB.z = w0 * z3D;  // Add to the mean
					B[0].x = x3D;
					B[0].y = y3D;
					B[0].z = z3D;  // Insert into B

					for (i = 1; i <= 2 * Na; i++)
					{
						// Form the Ai value
						if (i <= Na)
						{
							vAux.asEigen() = L.col(i - 1);
							myPoint[0] = meanA[0] + vAux[0];
							myPoint[1] = meanA[1] + vAux[1];
							myPoint[2] = meanA[2] + vAux[2];
						}
						else
						{
							vAux = L.col((i - Na) - 1);
							myPoint[0] = meanA[0] - vAux[0];
							myPoint[1] = meanA[1] - vAux[1];
							myPoint[2] = meanA[2] - vAux[2];
						}

						// Pass the Ai through the functions:
						x3D = (myPoint[0] - param.K(0, 2)) *
							  ((param.baseline)) / myPoint[2];
						y3D = (myPoint[1] - param.K(1, 2)) *
							  ((param.baseline)) / myPoint[2];
						z3D = (param.K(0, 0)) * ((param.baseline)) / myPoint[2];

						// Add to the B mean computation and the B vector
						meanB.x = meanB.x + w1 * x3D;
						meanB.y = meanB.y + w1 * y3D;
						meanB.z = meanB.z + w1 * z3D;

						B[i].x = x3D;
						B[i].y = y3D;
						B[i].z = z3D;

					}  // end for 'i'

					// Output covariance
					for (i = 0; i <= 2 * Na; i++)
					{
						float weight = w1;
						CMatrixF v(3, 1);

						if (i == 0)  // The weight for the mean value of A is w0
							weight = w0;

						v(0, 0) = B[i].x - meanB.x;
						v(1, 0) = B[i].y - meanB.y;
						v(2, 0) = B[i].z - meanB.z;

						Pb.asEigen() +=
							(weight * (v.asEigen() * v.transpose())).eval();
					}  // end for 'i'

					// Store it in the landmark
					lm.pose_cov_11 = Pb(0, 0);
					lm.pose_cov_12 = Pb(0, 1);
					lm.pose_cov_13 = Pb(0, 2);
					lm.pose_cov_22 = Pb(1, 1);
					lm.pose_cov_23 = Pb(1, 2);
					lm.pose_cov_33 = Pb(2, 2);
				}  // end case 'Prop_UT'
				break;

				case TStereoSystemParams::Prop_SUT:
				{
					// Parameters
					unsigned int Na = 3;
					unsigned int i;

					float a = param.factor_a;
					float b = param.factor_b;
					float k = param.factor_k;

					float lambda = square(a) * (Na + k) - Na;

					float w0_m = lambda / (Na + lambda);
					float w0_c = w0_m + (1 - square(a) + b);
					float w1 = 1 / (2 * (Na + lambda));

					CMatrixF Pa(3, 3);
					CMatrixF L(3, 3);

					Pa.fill(0);
					Pa(0, 0) = Pa(1, 1) =
						(Na + lambda) * square(param.stdPixel);
					Pa(2, 2) = (Na + lambda) * square(param.stdDisp);

					// Cholesky decomposition
					Pa.chol(L);  // math::chol(Pa,L);

					vector<TPoint3D> B;  // B group
					TPoint3D meanB;  // Mean value of the B group
					CMatrixF Pb;  // Covariance of the B group

					B.resize(2 * Na + 1);  // Set of output values
					Pb.fill(0);  // Reset the output covariance

					CVectorFloat vAux, myPoint;  // Auxiliar vectors
					CVectorFloat meanA;  // Mean value of the A group

					vAux.resize(3);  // Set the variables size
					meanA.resize(3);
					myPoint.resize(3);

					// Mean input value: (c,r,d)
					meanA[0] = ptL.x;
					meanA[1] = ptL.y;
					meanA[2] = disp;

					// Output mean
					meanB.x = w0_m * x3D;
					meanB.y = w0_m * y3D;
					meanB.z = w0_m * z3D;  // Add to the mean
					B[0].x = x3D;
					B[0].y = y3D;
					B[0].z = z3D;  // Insert into B

					for (i = 1; i <= 2 * Na; i++)
					{
						// Form the Ai value
						if (i <= Na)
						{
							vAux = L.row(i - 1);
							myPoint = meanA + vAux;
						}
						else
						{
							vAux = L.col((i - Na) - 1);
							myPoint = meanA - vAux;
						}

						// Pass the Ai through the functions:
						x3D = (myPoint[0] - param.K(0, 2)) *
							  ((param.baseline)) / myPoint[2];
						y3D = (myPoint[1] - param.K(1, 2)) *
							  ((param.baseline)) / myPoint[2];
						z3D = (param.K(0, 0)) * ((param.baseline)) / myPoint[2];

						// Add to the B mean computation and the B vector
						meanB.x = meanB.x + w1 * x3D;
						meanB.y = meanB.y + w1 * y3D;
						meanB.z = meanB.z + w1 * z3D;

						B[i].x = x3D;
						B[i].y = y3D;
						B[i].z = z3D;

					}  // end for 'i'

					// Output covariance
					for (i = 0; i <= 2 * Na; i++)
					{
						float weight = w1;
						CMatrixF v(3, 1);

						if (i == 0)  // The weight for the mean value of A is w0
							weight = w0_c;

						v(0, 0) = B[i].x - meanB.x;
						v(1, 0) = B[i].y - meanB.y;
						v(2, 0) = B[i].z - meanB.z;

						Pb.asEigen() +=
							(weight * (v.asEigen() * v.transpose())).eval();
					}  // end for 'i'

					// Store it in the landmark
					lm.pose_cov_11 = Pb(0, 0);
					lm.pose_cov_12 = Pb(0, 1);
					lm.pose_cov_13 = Pb(0, 2);
					lm.pose_cov_22 = Pb(1, 1);
					lm.pose_cov_23 = Pb(1, 2);
					lm.pose_cov_33 = Pb(2, 2);
				}  // end case 'Prop_SUT'
				break;

			}  // end switch
			landmarks.landmarks.push_back(lm);
			itListL++;
			itListR++;
		}  // end else ( (z3D > param.minZ) && (z3D < param.maxZ) )
	}  // end for 'i'

	MRPT_END
}

/* -------------------------------------------------------
				StereoObs2RBObs #1
   ------------------------------------------------------- */
void vision::StereoObs2BRObs(
	const CMatchedFeatureList& inMatches,
	const CMatrixDouble33& intrinsicParams, double baseline,
	const CPose3D& sensorPose, const vector<double>& sg,
	CObservationBearingRange& outObs)
{
	// Compute the range and bearing
	double f = intrinsicParams(0, 0);  // Focal length in pixels
	double x0 = intrinsicParams(0, 2);  // Principal point column
	double y0 = intrinsicParams(1, 2);  // Principal point row
	double b = baseline;  // Stereo camera baseline
	double sg_c2 = square(sg[0]);  // Sigma of the column variable
	double sg_r2 = square(sg[1]);  // Sigma of the row variable
	double sg_d2 = square(sg[2]);  // Sigma of the disparity

	for (const auto& inMatche : inMatches)
	{
		double x = inMatche.first.keypoint.pt.x;  // Column of the feature
		double y = inMatche.first.keypoint.pt.y;  // Row of the feature

		double d = x - inMatche.second.keypoint.pt.x;  // Disparity
		double d2 = square(d);
		double k = square(b / d);

		// Projection equations according to a standard camera coordinate axis
		// (+Z forward & +Y downwards)
		// -------------------------------------------------------------------------------------------------------
		double X = (x - x0) * b / d;
		double Y = (y - y0) * b / d;
		double Z = f * b / d;

		// Projection equations according to a standard coordinate axis (+X
		// forward & +Z upwards)
		// -------------------------------------------------------------------------------------------------------
		// double X	= f * b / d;
		// double Y	= ( x0 - x ) * b / d;
		// double Z	= ( y0 - y ) * b / d;

		CObservationBearingRange::TMeasurement m;
		m.range = sqrt(square(X) + square(Y) + square(Z));
		m.yaw = atan2(Y, X);
		m.pitch = -asin(Z / m.range);
		m.landmarkID = inMatche.first.keypoint.ID;

		// Compute the covariance
		// Formula: S_BR = JG * (JF * diag(sg_c^2, sg_r^2, sg_d^2) * JF') * JG'
		//						\---------------------------------------/
		//											aux

		CMatrixDouble33 aux;

		// Jacobian equations according to a standard CAMERA coordinate axis (+Z
		// forward & +Y downwards)
		// -------------------------------------------------------------------------------------------------------
		aux(0, 0) = k * (sg_c2 + sg_d2 * square(x - x0) / d2);
		aux(0, 1) = aux(1, 0) = k * (sg_d2 * (x - x0) * (y - y0) / d2);
		aux(0, 2) = aux(2, 0) = k * (sg_d2 * (x - x0) * f / d2);

		aux(1, 1) = k * (sg_r2 + sg_d2 * square(y - y0) / d2);
		aux(1, 2) = aux(2, 1) = k * (sg_d2 * (y - y0) * f / d2);

		aux(2, 2) = k * (sg_d2 * square(f) / d2);

		// Jacobian equations according to a standard coordinate axis (+X
		// forward & +Z upwards)
		// -------------------------------------------------------------------------------------------------------
		CMatrixDouble33 JG;
		JG(0, 0) = X / m.range;
		JG(0, 1) = Y / m.range;
		JG(0, 2) = Z / m.range;

		JG(1, 0) = -Y / (square(X) + square(Y));
		JG(1, 1) = X / (square(X) + square(Y));
		JG(1, 2) = 0;

		JG(2, 0) = Z * X / (square(m.range) * sqrt(square(X) + square(Y)));
		JG(2, 1) = Z * Y / (square(m.range) * sqrt(square(X) + square(Y)));
		JG(2, 2) = -sqrt(square(X) + square(Y)) / square(m.range);

		m.covariance = mrpt::math::multiply_HCHt(JG, aux);

		outObs.sensedData.push_back(m);

	}  // end for

	// Indicate that the covariances have been calculated (for compatibility
	// with earlier versions)
	outObs.validCovariances = true;
	outObs.setSensorPose(sensorPose);
}  // end-StereoObs2BRObs

/* -------------------------------------------------------
				StereoObs2RBObs #2
   ------------------------------------------------------- */
void vision::StereoObs2BRObs(
	const CObservationStereoImages& inObs, const vector<double>& sg,
	CObservationBearingRange& outObs)
{
	// Local variables
	CFeatureExtraction fExt;
	CFeatureList leftList, rightList;
	CMatchedFeatureList matchList;
	unsigned int id = 0;

	// Extract features
	fExt.detectFeatures(inObs.imageLeft, leftList);
	fExt.detectFeatures(inObs.imageRight, rightList);

	// DEBUG:
	// CDisplayWindow		win1, win2;
	// win1.showImageAndPoints( inObs.imageLeft, leftList );
	// win2.showImageAndPoints( inObs.imageRight, rightList );

	// Match features
	size_t nMatches = vision::matchFeatures(leftList, rightList, matchList);
	MRPT_UNUSED_PARAM(nMatches);

	// Compute the range and bearing
	double f = inObs.leftCamera.fx();  // Focal length in pixels
	double x0 = inObs.leftCamera.cx();  // Principal point column
	double y0 = inObs.leftCamera.cy();  // Principal point row
	double b = inObs.rightCameraPose.x();  // Stereo camera baseline
	double sg_c2 = square(sg[0]);  // Sigma of the column variable
	double sg_r2 = square(sg[1]);  // Sigma of the row variable
	double sg_d2 = square(sg[2]);  // Sigma of the disparity

	for (auto itMatchList = matchList.begin(); itMatchList != matchList.end();
		 itMatchList++, id++)
	{
		double x = itMatchList->first.keypoint.pt.x;  // Column of the feature
		double y = itMatchList->first.keypoint.pt.y;  // Row of the feature

		double d = x - itMatchList->second.keypoint.pt.x;  // Disparity
		double d2 = square(d);
		double k = square(b / d);

		// Projection equations according to a standard camera coordinate axis
		// (+Z forward & +Y downwards)
		// -------------------------------------------------------------------------------------------------------
		double X = (x - x0) * b / d;
		double Y = (y - y0) * b / d;
		double Z = f * b / d;

		// Projection equations according to a standard coordinate axis (+X
		// forward & +Z upwards)
		// -------------------------------------------------------------------------------------------------------
		// double X	= f * b / d;
		// double Y	= ( x0 - x ) * b / d;
		// double Z	= ( y0 - y ) * b / d;

		CObservationBearingRange::TMeasurement m;
		m.range = sqrt(square(X) + square(Y) + square(Z));
		// m.yaw	= atan2( Y,X );
		// m.pitch = -asin( Z/m.range );
		m.yaw = atan2(X, Z);
		m.pitch = atan2(Y, Z);

		// Compute the covariance
		// Formula: S_BR = JG * (JF * diag(sg_c^2, sg_r^2, sg_d^2) * JF') * JG'
		//						\---------------------------------------/
		//											aux

		CMatrixDouble33 aux;

		// Jacobian equations according to a standard CAMERA coordinate axis (+Z
		// forward & +Y downwards)
		// -------------------------------------------------------------------------------------------------------
		aux(0, 0) = k * (sg_c2 + sg_d2 * square(x - x0) / d2);
		aux(0, 1) = aux(1, 0) = k * (sg_d2 * (x - x0) * (y - y0) / d2);
		aux(0, 2) = aux(2, 0) = k * (sg_d2 * (x - x0) * f / d2);

		aux(1, 1) = k * (sg_r2 + sg_d2 * square(y - y0) / d2);
		aux(1, 2) = aux(2, 1) = k * (sg_d2 * (y - y0) * f / d2);

		aux(2, 2) = k * (sg_d2 * square(f) / d2);

		// Jacobian equations according to a standard coordinate axis (+X
		// forward & +Z upwards)
		// -------------------------------------------------------------------------------------------------------
		CMatrixDouble33 JG;
		JG(0, 0) = X / m.range;
		JG(0, 1) = Y / m.range;
		JG(0, 2) = Z / m.range;

		JG(1, 0) = -Y / (square(X) + square(Y));
		JG(1, 1) = X / (square(X) + square(Y));
		JG(1, 2) = 0;

		JG(2, 0) = Z * X / (square(m.range) * sqrt(square(X) + square(Y)));
		JG(2, 1) = Z * Y / (square(m.range) * sqrt(square(X) + square(Y)));
		JG(2, 2) = -sqrt(square(X) + square(Y)) / square(m.range);

		// JF.multiply_HCHt( diag, aux );
		m.covariance = mrpt::math::multiply_HCHt(JG, aux);

		m.landmarkID = id;
		outObs.sensedData.push_back(m);
		outObs.fieldOfView_yaw = 2 * fabs(atan2(-x0, f));
		outObs.fieldOfView_pitch = 2 * fabs(atan2(-y0, f));

	}  // end for

	// Indicate that the covariances have been calculated (for compatibility
	// with earlier versions)
	outObs.validCovariances = true;
	outObs.setSensorPose(mrpt::poses::CPose3D(inObs.cameraPose));

}  // end StereoObs2BRObs

/* -------------------------------------------------------
				StereoObs2RBObs #3
   ------------------------------------------------------- */
void vision::StereoObs2BRObs(
	const CObservationVisualLandmarks& inObs, CObservationBearingRange& outObs)
{
	// For each of the 3D landmarks [X,Y,Z] we compute their range and bearing
	// representation.
	// The reference system is assumed to be that typical of cameras: +Z forward
	// and +X to the right.
	CLandmarksMap::TCustomSequenceLandmarks::const_iterator itCloud;
	for (itCloud = inObs.landmarks.landmarks.begin();
		 itCloud != inObs.landmarks.landmarks.end(); ++itCloud)
	{
		CObservationBearingRange::TMeasurement m;
		m.range = sqrt(
			square(itCloud->pose_mean.x) + square(itCloud->pose_mean.y) +
			square(itCloud->pose_mean.z));

		// The reference system is assumed to be that typical robot operation:
		// +X forward and +Z upwards.
		m.yaw = atan2(itCloud->pose_mean.y, itCloud->pose_mean.x);
		m.pitch = -sin(itCloud->pose_mean.z / m.range);
		m.landmarkID = itCloud->ID;

		outObs.sensedData.push_back(m);
	}  // end for
}  // end StereoObs2BRObs

/* -------------------------------------------------------
				computeStereoRectificationMaps
   ------------------------------------------------------- */
void vision::computeStereoRectificationMaps(
	const TCamera& cam1, const TCamera& cam2,
	const poses::CPose3D& rightCameraPose, void* outMap1x, void* outMap1y,
	void* outMap2x, void* outMap2y)
{
	ASSERT_(cam1.ncols == cam2.ncols && cam1.nrows == cam2.nrows);

#if MRPT_HAS_OPENCV

	cv::Mat *mapx1, *mapy1, *mapx2, *mapy2;
	mapx1 = static_cast<cv::Mat*>(outMap1x);
	mapy1 = static_cast<cv::Mat*>(outMap1y);
	mapx2 = static_cast<cv::Mat*>(outMap2x);
	mapy2 = static_cast<cv::Mat*>(outMap2y);

	const int resX = cam1.ncols;
	const int resY = cam1.nrows;

	CMatrixDouble44 hMatrix;
	rightCameraPose.getHomogeneousMatrix(hMatrix);

	double rcTrans[3];
	rcTrans[0] = hMatrix(0, 3);
	rcTrans[1] = hMatrix(1, 3);
	rcTrans[2] = hMatrix(2, 3);

	double m1[3][3];
	for (unsigned int i = 0; i < 3; ++i)
		for (unsigned int j = 0; j < 3; ++j) m1[i][j] = hMatrix(i, j);

	double ipl[3][3], ipr[3][3], dpl[5], dpr[5];
	for (unsigned int i = 0; i < 3; ++i)
		for (unsigned int j = 0; j < 3; ++j)
		{
			ipl[i][j] = cam1.intrinsicParams(i, j);
			ipr[i][j] = cam2.intrinsicParams(i, j);
		}

	for (unsigned int i = 0; i < 5; ++i)
	{
		dpl[i] = cam1.dist[i];
		dpr[i] = cam2.dist[i];
	}

	cv::Mat R(3, 3, CV_64F, &m1);
	cv::Mat T(3, 1, CV_64F, &rcTrans);

	cv::Mat K1(3, 3, CV_64F, ipl);
	cv::Mat K2(3, 3, CV_64F, ipr);
	cv::Mat D1(1, 5, CV_64F, dpl);
	cv::Mat D2(1, 5, CV_64F, dpr);

	double _R1[3][3], _R2[3][3], _P1[3][4], _P2[3][4], _Q[4][4];
	cv::Mat R1(3, 3, CV_64F, _R1);
	cv::Mat R2(3, 3, CV_64F, _R2);
	cv::Mat P1(3, 4, CV_64F, _P1);
	cv::Mat P2(3, 4, CV_64F, _P2);
	cv::Mat Q(4, 4, CV_64F, _Q);

	cv::Size nSize(resX, resY);
	double alpha = 0.0;  // alpha value: 0.0 = zoom and crop the image so that
	// there's not black areas

	// OpenCV 2.3+ has this signature:
	cv::stereoRectify(
		K1, D1, K2, D2, nSize, R, T, R1, R2, P1, P2, Q,
		cv::CALIB_ZERO_DISPARITY, alpha);
	// Rest of arguments -> default

	cv::Size sz1, sz2;
	cv::initUndistortRectifyMap(
		K1, D1, R1, P1, cv::Size(resX, resY), CV_32FC1, *mapx1, *mapy1);
	cv::initUndistortRectifyMap(
		K2, D2, R2, P2, cv::Size(resX, resY), CV_32FC1, *mapx2, *mapy2);
/**/
#else
	THROW_EXCEPTION("The MRPT has been compiled without OPENCV!");
#endif
}  // end computeStereoRectificationMaps

TStereoSystemParams::TStereoSystemParams()

{
	K = defaultIntrinsicParamsMatrix(0, 640, 480);
	F.setZero();
	F(1, 2) = -1;
	F(2, 1) = 1;
}

/*-------------------------------------------------------------
			TStereoSystemParams: loadFromConfigFile
-------------------------------------------------------------*/
void TStereoSystemParams::loadFromConfigFile(
	const CConfigFileBase& iniFile, const string& section)
{
	int unc;
	unc = iniFile.read_int(section.c_str(), "uncPropagation", uncPropagation);
	switch (unc)
	{
		case 0:
			uncPropagation = Prop_Linear;
			break;
		case 1:
			uncPropagation = Prop_UT;
			break;
		case 2:
			uncPropagation = Prop_SUT;
			break;
	}  // end switch

	CVectorDouble k_vec(9);
	iniFile.read_vector(
		section.c_str(), "k_vec", CVectorDouble(), k_vec, false);
	for (unsigned int ii = 0; ii < 3; ++ii)
		for (unsigned int jj = 0; jj < 3; ++jj) K(ii, jj) = k_vec[ii * 3 + jj];

	CVectorDouble f_vec(9);
	iniFile.read_vector(
		section.c_str(), "f_vec", CVectorDouble(), f_vec, false);
	for (unsigned int ii = 0; ii < 3; ++ii)
		for (unsigned int jj = 0; jj < 3; ++jj) F(ii, jj) = f_vec[ii * 3 + jj];

	baseline = iniFile.read_float(section.c_str(), "baseline", baseline);
	stdPixel = iniFile.read_float(section.c_str(), "stdPixel", stdPixel);
	stdDisp = iniFile.read_float(section.c_str(), "stdDisp", stdDisp);
	maxZ = iniFile.read_float(section.c_str(), "maxZ", maxZ);
	minZ = iniFile.read_float(section.c_str(), "minZ", minZ);
	maxY = iniFile.read_float(section.c_str(), "maxY", maxY);
	factor_k = iniFile.read_float(section.c_str(), "factor_k", factor_k);
	factor_a = iniFile.read_float(section.c_str(), "factor_a", factor_a);
	factor_b = iniFile.read_float(section.c_str(), "factor_b", factor_b);
}  // end of loadFromConfigFile

/*---------------------------------------------------------------
					TStereoSystemParams: dumpToTextStream
  ---------------------------------------------------------------*/
void TStereoSystemParams::dumpToTextStream(std::ostream& out) const
{
	out << "\n----------- [vision::TStereoSystemParams] ------------ \n";
	out << "Method for 3D Uncert. \t= ";
	switch (uncPropagation)
	{
		case Prop_Linear:
			out << "Linear propagation\n";
			break;
		case Prop_UT:
			out << "Unscented Transform\n";
			break;
		case Prop_SUT:
			out << "Scaled Unscented Transform\n";
			break;
	}  // end switch

	out << mrpt::format("K\t\t\t= [%f\t%f\t%f]\n", K(0, 0), K(0, 1), K(0, 2));
	out << mrpt::format(" \t\t\t  [%f\t%f\t%f]\n", K(1, 0), K(1, 1), K(1, 2));
	out << mrpt::format(" \t\t\t  [%f\t%f\t%f]\n", K(2, 0), K(2, 1), K(2, 2));

	out << mrpt::format("F\t\t\t= [%f\t%f\t%f]\n", F(0, 0), F(0, 1), F(0, 2));
	out << mrpt::format(" \t\t\t  [%f\t%f\t%f]\n", F(1, 0), F(1, 1), F(1, 2));
	out << mrpt::format(" \t\t\t  [%f\t%f\t%f]\n", F(2, 0), F(2, 1), F(2, 2));

	out << mrpt::format("Baseline \t\t= %f\n", baseline);
	out << mrpt::format("Pixel std \t\t= %f\n", stdPixel);
	out << mrpt::format("Disparity std\t\t= %f\n", stdDisp);
	out << mrpt::format("Z maximum\t\t= %f\n", maxZ);
	out << mrpt::format("Z minimum\t\t= %f\n", minZ);
	out << mrpt::format("Y maximum\t\t= %f\n", maxY);

	out << mrpt::format("k Factor [UT]\t\t= %f\n", factor_k);
	out << mrpt::format("a Factor [UT]\t\t= %f\n", factor_a);
	out << mrpt::format("b Factor [UT]\t\t= %f\n", factor_b);
	out << "-------------------------------------------------------- \n";
}

/*-------------------------------------------------------------
			TMatchingOptions: constructor
-------------------------------------------------------------*/
TMatchingOptions::TMatchingOptions()

	= default;  // end constructor TMatchingOptions

/*-------------------------------------------------------------
			TMatchingOptions: loadFromConfigFile
-------------------------------------------------------------*/
void TMatchingOptions::loadFromConfigFile(
	const CConfigFileBase& iniFile, const string& section)
{
	int mm =
		iniFile.read_int(section.c_str(), "matching_method", matching_method);
	switch (mm)
	{
		case 0:
			matching_method = mmCorrelation;
			break;
		case 1:
			matching_method = mmDescriptorSIFT;
			break;
		case 2:
			matching_method = mmDescriptorSURF;
			break;
		case 3:
			matching_method = mmSAD;
			break;
		case 4:
			matching_method = mmDescriptorORB;
			break;
	}  // end switch

	useEpipolarRestriction = iniFile.read_bool(
		section.c_str(), "useEpipolarRestriction", useEpipolarRestriction);
	hasFundamentalMatrix = iniFile.read_bool(
		section.c_str(), "hasFundamentalMatrix", hasFundamentalMatrix);
	parallelOpticalAxis = iniFile.read_bool(
		section.c_str(), "parallelOpticalAxis", parallelOpticalAxis);
	useXRestriction =
		iniFile.read_bool(section.c_str(), "useXRestriction", useXRestriction);
	addMatches = iniFile.read_bool(section.c_str(), "addMatches", addMatches);
	useDisparityLimits = iniFile.read_bool(
		section.c_str(), "useDisparityLimits", useDisparityLimits);

	min_disp = iniFile.read_float(section.c_str(), "min_disp", min_disp);
	max_disp = iniFile.read_float(section.c_str(), "max_disp", max_disp);

	epipolar_TH =
		iniFile.read_float(section.c_str(), "epipolar_TH", epipolar_TH);
	maxEDD_TH = iniFile.read_float(section.c_str(), "maxEDD_TH", maxEDD_TH);
	EDD_RATIO = iniFile.read_float(section.c_str(), "minDIF_TH", EDD_RATIO);
	minCC_TH = iniFile.read_float(section.c_str(), "minCC_TH", minCC_TH);
	minDCC_TH = iniFile.read_float(section.c_str(), "minDCC_TH", minDCC_TH);
	rCC_TH = iniFile.read_float(section.c_str(), "rCC_TH", rCC_TH);
	maxEDSD_TH = iniFile.read_float(section.c_str(), "maxEDSD_TH", maxEDSD_TH);
	EDSD_RATIO = iniFile.read_float(section.c_str(), "EDSD_RATIO", EDSD_RATIO);
	maxSAD_TH = iniFile.read_float(section.c_str(), "maxSAD_TH", maxSAD_TH);
	SAD_RATIO = iniFile.read_float(section.c_str(), "SAD_RATIO", SAD_RATIO);
	maxORB_dist =
		iniFile.read_float(section.c_str(), "maxORB_dist", maxORB_dist);

	estimateDepth =
		iniFile.read_bool(section.c_str(), "estimateDepth", estimateDepth);
	maxDepthThreshold = iniFile.read_float(
		section.c_str(), "maxDepthThreshold", maxDepthThreshold);
	//	fx                  = iniFile.read_float(section.c_str(),"fx",fx);
	//	cx                  = iniFile.read_float(section.c_str(),"cx",cx);
	//	cy                  = iniFile.read_float(section.c_str(),"cy",cy);
	//	baseline            =
	// iniFile.read_float(section.c_str(),"baseline",baseline);
}  // end TMatchingOptions::loadFromConfigFile

/*---------------------------------------------------------------
					TMatchingOptions: dumpToTextStream
  ---------------------------------------------------------------*/
void TMatchingOptions::dumpToTextStream(std::ostream& out) const
{
	out << "\n----------- [vision::TMatchingOptions] ------------ \n";
	out << "Matching method:                ";
	switch (matching_method)
	{
		case mmCorrelation:
			out << "Cross Correlation\n";
			out << mrpt::format(
				"· Min. CC. Threshold:           %f\n", minCC_TH);
			out << mrpt::format(
				"· Min. Dif. CC Threshold:       %f\n", minDCC_TH);
			out << mrpt::format("· Max. Ratio CC Threshold:      %f\n", rCC_TH);
			break;
		case mmDescriptorSIFT:
			out << "SIFT descriptor\n";
			out << mrpt::format(
				"· Max. EDD Threshold:           %f\n", maxEDD_TH);
			out << mrpt::format(
				"· EDD Ratio:                    %f\n", EDD_RATIO);
			break;
		case mmDescriptorSURF:
			out << "SURF descriptor\n";
			out << mrpt::format(
				"· EDD Ratio:                    %f\n", maxEDSD_TH);
			out << mrpt::format(
				"· Min. CC Threshold:            %f\n", EDSD_RATIO);
			break;
		case mmSAD:
			out << "SAD\n";
			out << mrpt::format(
				"· Max. Dif. SAD Threshold:      %f\n", maxSAD_TH);
			out << mrpt::format(
				"· Ratio SAD Threshold:          %f\n", SAD_RATIO);
			break;
		case mmDescriptorORB:
			out << "ORB\n";
			out << mrpt::format(
				"· Max. distance between desc:	%f\n", maxORB_dist);
			break;
	}  // end switch
	out << mrpt::format(
		"Epipolar Thres:                 %.2f px\n", epipolar_TH);
	out << "Using epipolar restriction?:    "
		<< (useEpipolarRestriction ? "Yes\n" : "No\n");
	out << "Has Fundamental Matrix?:        "
		<< (hasFundamentalMatrix ? "Yes\n" : "No\n");
	out << "Are camera axis parallel?:      "
		<< (parallelOpticalAxis ? "Yes\n" : "No\n");
	out << "Use X-coord restriction?:       "
		<< (useXRestriction ? "Yes\n" : "No\n");
	out << "Use disparity limits?:       "
		<< (useDisparityLimits ? "Yes\n" : "No\n");
	if (useDisparityLimits)
		out << mrpt::format(
			"· Min/max disp limits:          %.2f/%.2f px\n", min_disp,
			max_disp);
	out << "Estimate depth?:                "
		<< (estimateDepth ? "Yes\n" : "No\n");
	if (estimateDepth)
	{
		out << mrpt::format(
			"· Maximum depth allowed:        %f m\n", maxDepthThreshold);
	}
	out << "Add matches to list?:           ";
	out << (addMatches ? "Yes\n" : "No\n");
	out << "-------------------------------------------------------- \n";
}  // end TMatchingOptions::dumpToTextStream
