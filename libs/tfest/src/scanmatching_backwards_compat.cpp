/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include "tfest-precomp.h"  // Precompiled headers

#include <mrpt/tfest.h>

#define MRPT_SCANMATCHING_SUPRESS_BACKCOMPAT_WARNING
#include <mrpt/scanmatching/scan_matching.h>

#include <mrpt/poses/CPose3D.h>

using namespace mrpt;
using namespace mrpt::utils;
using namespace mrpt::poses;
using namespace mrpt::math;
using namespace std;

double backwards_compat_HornMethod(
	const std::vector<double>      &input,
	mrpt::poses::CPose3DQuat &outQuat,
	bool                      forceScaleToUnity)
{
	// Convert input data:
	const size_t nMatches = input.size()/6;
	ASSERT_EQUAL_(input.size()%6, 0)
	vector<TPoint3D> inThis(nMatches), inOther(nMatches);
	for(size_t i = 0; i < nMatches; i++ )
	{
		inThis[i].x = input[i*6+0];
		inThis[i].y = input[i*6+1];
		inThis[i].z = input[i*6+2];

		inOther[i].x = input[i*6+3];
		inOther[i].y = input[i*6+4];
		inOther[i].z = input[i*6+5];
	}

	// Call:
	double scale=.0;
	mrpt::tfest::se3_l2(inThis,inOther,outQuat,scale,forceScaleToUnity);

	return scale;
}


// Deprecated: Use mrpt::tfest::se3_l2() instead
double mrpt::scanmatching::HornMethod(
	const std::vector<double>  &inPoints,
	std::vector<double>        &outVector,
	bool                 forceScaleToUnity)
{
	mrpt::poses::CPose3DQuat outQuat;
	const double scale = backwards_compat_HornMethod(inPoints,outQuat,forceScaleToUnity);
	// Output:
	outVector.resize( 7 );
	for (int i=0;i<7;i++) outVector[i] = outQuat[i];
	return scale;
}

// Deprecated: Use mrpt::tfest::se3_l2() instead
double mrpt::scanmatching::HornMethod(
	const std::vector<double>      &inPoints,
	mrpt::poses::CPose3DQuat &outQuat,
	bool                      forceScaleToUnity)
{
	return backwards_compat_HornMethod(inPoints,outQuat,forceScaleToUnity);
}

// Deprecated: Use mrpt::tfest::se3_l2() instead
bool mrpt::scanmatching::leastSquareErrorRigidTransformation6D(
	const mrpt::utils::TMatchingPairList	&in_correspondences,
	mrpt::poses::CPose3DQuat							&out_transformation,
	double								&out_scale,
	const bool 							forceScaleToUnity)
{
	return mrpt::tfest::se3_l2(in_correspondences,out_transformation,out_scale,forceScaleToUnity);
}

// Deprecated: Use mrpt::tfest::se3_l2() instead
bool mrpt::scanmatching::leastSquareErrorRigidTransformation6D(
	const mrpt::utils::TMatchingPairList	&in_correspondences,
	mrpt::poses::CPose3D					&out_transformation,
	double								&out_scale,
	const bool 							forceScaleToUnity)
{
	mrpt::poses::CPose3DQuat tf;
	const bool ret = mrpt::tfest::se3_l2(in_correspondences,tf,out_scale,forceScaleToUnity);
	out_transformation = mrpt::poses::CPose3D(tf);
	return ret;
}

// Deprecated: Use mrpt::tfest::se3_l2_robust() instead
bool mrpt::scanmatching::leastSquareErrorRigidTransformation6DRANSAC(
	const mrpt::utils::TMatchingPairList	&in_correspondences,
	mrpt::poses::CPose3D								&out_transformation,
	double								&out_scale,
	vector_int							&out_inliers_idx,
	const unsigned int					ransac_minSetSize,
	const unsigned int					ransac_nmaxSimulations,
	const double						ransac_maxSetSizePct,
	const bool							forceScaleToUnity)
{
	mrpt::tfest::TSE3RobustParams params;
	params.forceScaleToUnity = forceScaleToUnity;
	params.ransac_maxSetSizePct = ransac_maxSetSizePct;
	params.ransac_minSetSize = ransac_minSetSize;
	params.ransac_nmaxSimulations = ransac_nmaxSimulations;

	mrpt::tfest::TSE3RobustResult results;
	const bool ret = mrpt::tfest::se3_l2_robust(in_correspondences,params,results);

	out_inliers_idx = results.inliers_idx;
	out_scale = results.scale;
	out_transformation = results.transformation;

	return ret;
}

// Deprecated: Use mrpt::tfest::se2_l2() instead
bool mrpt::scanmatching::leastSquareErrorRigidTransformation(
	mrpt::utils::TMatchingPairList	&in_correspondences,
	mrpt::poses::CPose2D							&out_transformation,
	mrpt::math::CMatrixDouble33					*out_estimateCovariance)
{
	mrpt::math::TPose2D estPose;
	const bool ret = mrpt::tfest::se2_l2(in_correspondences,estPose,out_estimateCovariance);
	out_transformation = estPose;
	return ret;
}

// Deprecated: Use mrpt::tfest::se2_l2() instead
bool mrpt::scanmatching::leastSquareErrorRigidTransformation(
	mrpt::utils::TMatchingPairList	&in_correspondences,
	mrpt::poses::CPosePDFGaussian				&out_transformation )
{
	return mrpt::tfest::se2_l2(in_correspondences,out_transformation);
}

// Deprecated: Use mrpt::tfest::se2_l2_robust() instead
void mrpt::scanmatching::robustRigidTransformation(
	mrpt::utils::TMatchingPairList	&in_correspondences,
	mrpt::poses::CPosePDFSOG				&out_transformation,
	float							normalizationStd,
	unsigned int					ransac_minSetSize,
	unsigned int					ransac_maxSetSize,
	float							ransac_mahalanobisDistanceThreshold,
	unsigned int					ransac_nSimulations,
	mrpt::utils::TMatchingPairList		*out_largestSubSet,
	bool						ransac_fuseByCorrsMatch,
	float						ransac_fuseMaxDiffXY,
	float						ransac_fuseMaxDiffPhi,
	bool						ransac_algorithmForLandmarks,
	double 						probability_find_good_model,
	unsigned int				ransac_min_nSimulations,
	const bool                  verbose,
	double                      max_rmse_to_end
	)
{
	mrpt::tfest::TSE2RobustParams params;
	params.ransac_minSetSize = ransac_minSetSize;
	params.ransac_maxSetSize = ransac_maxSetSize;
	params.ransac_mahalanobisDistanceThreshold = ransac_mahalanobisDistanceThreshold;
	params.ransac_nSimulations = ransac_nSimulations;
	params.ransac_fuseByCorrsMatch = ransac_fuseByCorrsMatch;
	params.ransac_fuseMaxDiffXY = ransac_fuseMaxDiffXY;
	params.ransac_fuseMaxDiffPhi  = ransac_fuseMaxDiffPhi;
	params.ransac_algorithmForLandmarks = ransac_algorithmForLandmarks;
	params.probability_find_good_model = probability_find_good_model;
	params.ransac_min_nSimulations = ransac_min_nSimulations;
	params.max_rmse_to_end = max_rmse_to_end;
	params.verbose = verbose;

	mrpt::tfest::TSE2RobustResult results;
	//const bool ret = 
	mrpt::tfest::se2_l2_robust(in_correspondences,normalizationStd,params,results);

	out_transformation = results.transformation;
	if (out_largestSubSet) *out_largestSubSet = results.largestSubSet;
}

