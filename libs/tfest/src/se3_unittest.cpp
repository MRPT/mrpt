/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2020, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#include <gtest/gtest.h>
#include <mrpt/math/ops_vectors.h>
#include <mrpt/poses/CPose3D.h>
#include <mrpt/poses/CPose3DQuat.h>
#include <mrpt/random.h>
#include <mrpt/tfest.h>

using namespace mrpt;
using namespace mrpt::tfest;
using namespace mrpt::math;
using namespace mrpt::random;
using namespace mrpt::poses;
using namespace std;

using TPoints = std::vector<std::vector<double>>;

// ------------------------------------------------------
//				Generate both sets of points
// ------------------------------------------------------
CPose3DQuat generate_points(TPoints& pA, TPoints& pB)
{
	const double Dx = 0.5;
	const double Dy = 1.5;
	const double Dz = 0.75;

	const double yaw = 10.0_deg;
	const double pitch = 20.0_deg;
	const double roll = 5.0_deg;

	pA.resize(5);  // A set of points at "A" reference system
	pB.resize(5);  // A set of points at "B" reference system

	pA[0].resize(3);
	pA[0][0] = 0.0;
	pA[0][1] = 0.5;
	pA[0][2] = 0.4;
	pA[1].resize(3);
	pA[1][0] = 1.0;
	pA[1][1] = 1.5;
	pA[1][2] = -0.1;
	pA[2].resize(3);
	pA[2][0] = 1.2;
	pA[2][1] = 1.1;
	pA[2][2] = 0.9;
	pA[3].resize(3);
	pA[3][0] = 0.7;
	pA[3][1] = 0.3;
	pA[3][2] = 3.4;
	pA[4].resize(3);
	pA[4][0] = 1.9;
	pA[4][1] = 2.5;
	pA[4][2] = -1.7;

	CPose3DQuat qPose = CPose3DQuat(CPose3D(Dx, Dy, Dz, yaw, pitch, roll));
	for (unsigned int i = 0; i < 5; ++i)
	{
		pB[i].resize(3);
		qPose.inverseComposePoint(
			pA[i][0], pA[i][1], pA[i][2], pB[i][0], pB[i][1], pB[i][2]);
	}

	return qPose;

}  // end generate_points

// ------------------------------------------------------
//				Generate a list of matched points
// ------------------------------------------------------
template <typename T>
void generate_list_of_points(
	const TPoints& pA, const TPoints& pB, TMatchingPairListTempl<T>& list)
{
	TMatchingPairTempl<T> pair;
	for (unsigned int i = 0; i < 5; ++i)
	{
		pair.this_idx = pair.other_idx = i;
		pair.this_x = d2f(pA[i][0]);
		pair.this_y = d2f(pA[i][1]);
		pair.this_z = d2f(pA[i][2]);

		pair.other_x = d2f(pB[i][0]);
		pair.other_y = d2f(pB[i][1]);
		pair.other_z = d2f(pB[i][2]);

		list.push_back(pair);
	}
}  // end generate_list_of_points

// ------------------------------------------------------
//				Genreate a vector of matched points
// ------------------------------------------------------
void generate_vector_of_points(
	const TPoints& pA, const TPoints& pB, vector<mrpt::math::TPoint3D>& ptsA,
	vector<mrpt::math::TPoint3D>& ptsB)
{
	// The input vector: inV = [pA1x, pA1y, pA1z, pB1x, pB1y, pB1z, ... ]
	ptsA.resize(pA.size());
	ptsB.resize(pA.size());
	for (unsigned int i = 0; i < pA.size(); ++i)
	{
		ptsA[i] = mrpt::math::TPoint3D(pA[i][0], pA[i][1], pA[i][2]);
		ptsB[i] = mrpt::math::TPoint3D(pB[i][0], pB[i][1], pB[i][2]);
	}
}  // end generate_vector_of_points

template <typename T>
void se3_l2_MatchList_test()
{
	TPoints pA, pB;  // The input points
	CPose3DQuat qPose = generate_points(pA, pB);

	TMatchingPairListTempl<T> list;
	generate_list_of_points(pA, pB, list);  // Generate a list of matched points

	CPose3DQuat outQuat;  // Output CPose3DQuat for the LSRigidTransformation
	double scale;  // Output scale value

	bool res = mrpt::tfest::se3_l2(list, outQuat, scale);
	EXPECT_TRUE(res);

	double err = 0.0;
	if ((qPose[3] * outQuat[3] > 0 && qPose[4] * outQuat[4] > 0 &&
		 qPose[5] * outQuat[5] > 0 && qPose[6] * outQuat[6] > 0) ||
		(qPose[3] * outQuat[3] < 0 && qPose[4] * outQuat[4] < 0 &&
		 qPose[5] * outQuat[5] < 0 && qPose[6] * outQuat[6] < 0))
	{
		for (unsigned int i = 0; i < 7; ++i)
			err += square(std::fabs(qPose[i]) - std::fabs(outQuat[i]));
		err = sqrt(err);
		EXPECT_TRUE(err < 1e-6) << "Applied quaternion: " << endl
								<< qPose << endl
								<< "Out CPose3DQuat: " << endl
								<< outQuat << " [Err: " << err << "]" << endl;
	}
	else
	{
		GTEST_FAIL() << "Applied quaternion: " << endl
					 << qPose << endl
					 << "Out CPose3DQuat: " << endl
					 << outQuat << endl;
	}
}

TEST(tfest, se3_l2_MatchList_float) { se3_l2_MatchList_test<float>(); }
TEST(tfest, se3_l2_MatchList_double) { se3_l2_MatchList_test<double>(); }

TEST(tfest, se3_l2_PtsLists)
{
	TPoints pA, pB;  // The input points
	CPose3DQuat qPose = generate_points(pA, pB);

	vector<mrpt::math::TPoint3D> ptsA, ptsB;
	generate_vector_of_points(
		pA, pB, ptsA, ptsB);  // Generate a vector of matched points

	mrpt::poses::CPose3DQuat qu;
	double scale;
	mrpt::tfest::se3_l2(
		ptsA, ptsB, qu, scale);  // Output quaternion for the Horn Method

	double err = 0.0;
	if ((qPose[3] * qu[3] > 0 && qPose[4] * qu[4] > 0 && qPose[5] * qu[5] > 0 &&
		 qPose[6] * qu[6] > 0) ||
		(qPose[3] * qu[3] < 0 && qPose[4] * qu[4] < 0 && qPose[5] * qu[5] < 0 &&
		 qPose[6] * qu[6] < 0))
	{
		for (unsigned int i = 0; i < 7; ++i)
			err += square(std::fabs(qPose[i]) - std::fabs(qu[i]));
		err = sqrt(err);
		EXPECT_TRUE(err < 1e-6) << "Applied quaternion: " << endl
								<< qPose << endl
								<< "Out CPose3DQuat: " << endl
								<< qu << " [Err: " << err << "]" << endl;
	}
	else
	{
		GTEST_FAIL() << "Applied quaternion: " << endl
					 << qPose << endl
					 << "Out CPose3DQuat: " << endl
					 << qu << endl;
	}
}  // end

TEST(tfest, se3_l2_robust)
{
	TPoints pA, pB;  // The input points
	CPose3DQuat qPose = generate_points(pA, pB);

	TMatchingPairList list;
	generate_list_of_points(pA, pB, list);  // Generate a list of matched points

	mrpt::tfest::TSE3RobustResult estim_result;
	mrpt::tfest::TSE3RobustParams params;
	params.ransac_minSetSize = 3;
	params.ransac_maxSetSizePct = 3.0 / list.size();
	mrpt::tfest::se3_l2_robust(list, params, estim_result);

	EXPECT_GT(estim_result.inliers_idx.size(), 0u);
	const CPose3DQuat& outQuat = estim_result.transformation;
	double err = 0.0;
	if ((qPose[3] * outQuat[3] > 0 && qPose[4] * outQuat[4] > 0 &&
		 qPose[5] * outQuat[5] > 0 && qPose[6] * outQuat[6] > 0) ||
		(qPose[3] * outQuat[3] < 0 && qPose[4] * outQuat[4] < 0 &&
		 qPose[5] * outQuat[5] < 0 && qPose[6] * outQuat[6] < 0))
	{
		for (unsigned int i = 0; i < 7; ++i)
			err += square(std::fabs(qPose[i]) - std::fabs(outQuat[i]));
		err = sqrt(err);
		EXPECT_TRUE(err < 1e-6) << "Applied quaternion: " << endl
								<< qPose << endl
								<< "Out CPose3DQuat: " << endl
								<< outQuat << " [Err: " << err << "]" << endl;
	}
	else
	{
		GTEST_FAIL() << "Applied quaternion: " << endl
					 << qPose << endl
					 << "Out CPose3DQuat: " << endl
					 << outQuat << endl;
	}
}
